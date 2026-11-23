import socket
import struct
import time
from pathlib import Path
from datetime import datetime
import numpy as np

# ==============================================================================
# Core Stream Specs (1-to-1 Match with FreeRTOS udp_perf_client.h)
# ==============================================================================

# 1440 - Nominal Fixed max UDP payload size (bytes), 1040 - adjust sz for spectrum output byte alignment
MAX_UDP_BUFFER_SZ  = 1040 # 1440       # Fixed max UDP payload size (bytes)
DATA_OFFSET        = 16         # Header size: pkt_id (4) + sync (4) + timestamp (8)
DATAWIDTH          = 32         # Bits per channel sample: 16, 32, or 64
NUM_SDR_CHANNELS   = 2          # Parallel channels packed per beat (Ch1, Ch2)

# Data Type Configuration:
#   "uint"  -> unsigned fixed-point (e.g., uint32 spectrometer power)
#   "int"   -> signed fixed-point   (e.g., int16/int32 ADC or FIR output)
#   "float" -> IEEE-754 float       (e.g., float32)
DATA_FORMAT        = "uint"
PAYLOAD_BIG_ENDIAN = False      # True if SWAP_PAYLOAD_ENDIANNESS == 1 in C

# Optional Fixed-Point Scaling:
#   Set FRAC_BITS > 0 if your fixed-point data is Q-format (e.g., Q1.15 -> 15)
#   Set CONVERT_TO_FLOAT32 = True to save scaled float32 to disk (e.g., for GNU Radio)
FRAC_BITS          = 0
CONVERT_TO_FLOAT32 = True

# ==============================================================================
# Derived Constants (Auto-calculated identically to FreeRTOS C macros)
# ==============================================================================
SYNC_WORD          = 0xDEADBEEF
MAX_DMA_PKT_LEN    = MAX_UDP_BUFFER_SZ - DATA_OFFSET            # 1424 bytes
BYTES_PER_SAMPLE   = DATAWIDTH // 8                             # 4 bytes
BYTES_PER_BEAT     = BYTES_PER_SAMPLE * NUM_SDR_CHANNELS        # 8 bytes
NUM_DATABEATS      = MAX_DMA_PKT_LEN // BYTES_PER_BEAT          # 178 beats
PAYLOAD_SZ         = NUM_DATABEATS * BYTES_PER_BEAT             # 1424 bytes
EXPECTED_PACKET_LEN = DATA_OFFSET + PAYLOAD_SZ                  # 1440 bytes


def build_numpy_dtype(data_format: str, datawidth: int, big_endian: bool) -> np.dtype:
    fmt_map = {"float": "f", "int": "i", "uint": "u"}
    if data_format not in fmt_map:
        raise ValueError(f"Unsupported DATA_FORMAT '{data_format}'. Use 'float', 'int', or 'uint'.")
    if data_format == "float" and datawidth not in (32, 64):
        raise ValueError("Float format requires DATAWIDTH of 32 or 64.")
    if datawidth not in (8, 16, 32, 64):
        raise ValueError(f"Unsupported DATAWIDTH {datawidth}. Use 8, 16, 32, or 64.")

    endian = ">" if big_endian else "<"
    return np.dtype(f"{endian}{fmt_map[data_format]}{datawidth // 8}")


PAYLOAD_DTYPE = build_numpy_dtype(DATA_FORMAT, DATAWIDTH, PAYLOAD_BIG_ENDIAN)
FIXED_SCALE   = float(1 << FRAC_BITS) if (DATA_FORMAT != "float" and FRAC_BITS > 0) else 1.0

# --- Output File Configuration ---
SAVE_FOLDER = "data"
this_dir = Path(__file__).parent.resolve()
parent_dir = this_dir.parent
run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

SAVE_PATH = parent_dir / SAVE_FOLDER
SAVE_PATH.mkdir(parents=True, exist_ok=True)

OUTPUT_FILES_CH = [
    SAVE_PATH / f"sdr_ch{ch + 1}_{DATA_FORMAT}{DATAWIDTH}_{run_ts}.bin"
    for ch in range(NUM_SDR_CHANNELS)
]
OUTPUT_FILE_TS = SAVE_PATH / f"sdr_timestamps_{run_ts}.bin"


def run_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    sock.bind(("0.0.0.0", 5001))
    sock.settimeout(5.0)

    print("[INFO] Listening for SDR data on UDP port 5001...")
    print(f"[INFO] Format: {DATA_FORMAT}{DATAWIDTH} ({PAYLOAD_DTYPE}) | "
          f"Channels: {NUM_SDR_CHANNELS} | Beats/Pkt: {NUM_DATABEATS} | "
          f"Expected Pkt: {EXPECTED_PACKET_LEN} B")
    for ch, path in enumerate(OUTPUT_FILES_CH):
        print(f"[INFO] Saving Ch{ch + 1} to: {path}")

    prev_packet_id = None
    total_packets = 0
    total_dropped_packets = 0
    invalid_sync_cnt = 0
    skipped_events = []

    start_time = None
    last_report_time = None

    ch_files = [open(p, "wb", buffering=1024 * 1024) for p in OUTPUT_FILES_CH]
    f_ts = open(OUTPUT_FILE_TS, "wb", buffering=64 * 1024)

    try:
        while True:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                if total_packets > 0:
                    print("\n[INFO] Socket timeout (no packets for 5s). Finishing capture.")
                    break
                continue

            if len(data) < EXPECTED_PACKET_LEN:
                continue

            now = time.monotonic()
            if start_time is None:
                start_time = now
                last_report_time = now
                print("[INFO] First packet received! Capturing stream...")

            # 1. Unpack 16-byte Header (Big-Endian)
            packet_id, sync_word, hw_timestamp = struct.unpack(">iIQ", data[:16])

            if packet_id == -1:
                print("\n[INFO] Received end-of-stream marker (packet_id = -1) from board.")
                break

            # 2. Validate Sync Word
            if sync_word != SYNC_WORD:
                invalid_sync_cnt += 1
                continue

            # 3. Track sequence numbers for dropped packets
            if prev_packet_id is not None:
                gap = packet_id - prev_packet_id
                if gap != 1:
                    dropped = gap - 1 if gap > 1 else 0
                    total_dropped_packets += dropped
                    skipped_events.append((prev_packet_id, packet_id, dropped))
            prev_packet_id = packet_id
            total_packets += 1

            # 4. Unpack Payload -> shape (NUM_DATABEATS, NUM_SDR_CHANNELS)
            raw_payload = memoryview(data)[DATA_OFFSET:EXPECTED_PACKET_LEN]
            beats = np.frombuffer(raw_payload, dtype=PAYLOAD_DTYPE).reshape(
                NUM_DATABEATS, NUM_SDR_CHANNELS
            )

            # 5. Write each channel to disk
            for ch in range(NUM_SDR_CHANNELS):
                ch_data = beats[:, ch]
                if CONVERT_TO_FLOAT32 and DATA_FORMAT != "float":
                    ch_out = (ch_data.astype(np.float32) / FIXED_SCALE)
                    ch_files[ch].write(ch_out.tobytes())
                else:
                    ch_files[ch].write(ch_data.tobytes())

            f_ts.write(struct.pack("<iQ", packet_id, hw_timestamp))

            # 6. Periodic 1 Hz status report
            if (now - last_report_time) >= 1.0:
                elapsed = now - start_time
                mb_received = (total_packets * PAYLOAD_SZ) / (1024 * 1024)
                mbps = (total_packets * EXPECTED_PACKET_LEN * 8) / (elapsed * 1e6)

                if DATA_FORMAT == "float" or FRAC_BITS > 0:
                    ch_str = ", ".join(
                        f"Ch{c+1}[0]: {beats[0, c] / FIXED_SCALE:+1.5f}"
                        for c in range(NUM_SDR_CHANNELS)
                    )
                else:
                    ch_str = ", ".join(
                        f"Ch{c+1}[0]: {beats[0, c]:d}"
                        for c in range(NUM_SDR_CHANNELS)
                    )

                print(
                    f"\r[STATUS] Pkts: {total_packets:7d} | "
                    f"Data: {mb_received:6.2f} MB | "
                    f"Rate: {mbps:6.2f} Mbps | "
                    f"Dropped: {total_dropped_packets:4d} | {ch_str}",
                    end="",
                    flush=True,
                )
                last_report_time = now

    except KeyboardInterrupt:
        print("\n[INFO] Capture stopped by user.")
    finally:
        sock.close()
        for f in ch_files:
            f.close()
        f_ts.close()

    # --- Final Capture Summary ---
    total_samples = total_packets * NUM_DATABEATS
    print("\n" + "=" * 60)
    print("CAPTURE SUMMARY")
    print("=" * 60)
    print(f"Total Packets Captured : {total_packets}")
    print(f"Total Samples / Channel: {total_samples}")
    print(f"Total Dropped Packets  : {total_dropped_packets} ({len(skipped_events)} gap events)")
    print(f"Invalid Sync Words     : {invalid_sync_cnt}")
    if skipped_events:
        print(f"First 10 Gap Events (prev_id -> curr_id, missed): {skipped_events[:10]}")
    print("=" * 60)


if __name__ == "__main__":
    run_receiver()