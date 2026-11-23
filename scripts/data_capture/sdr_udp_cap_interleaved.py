import socket
import struct
import time
from pathlib import Path
from datetime import datetime
import numpy as np

# --- PROJECT FOLDER ---
PROJECT_FOLDER = "03_iq_mixer" # leave blank as "" if you don't want to save to project folder

# --- Header Offsets (Matching C Macros) ---
PACKET_ID_SZ = 4
SYNC_WORD_SZ = 4
TIMESTAMP_SZ = 8
HEADER_SZ = PACKET_ID_SZ + SYNC_WORD_SZ + TIMESTAMP_SZ  # 16 bytes

SYNC_WORD = 0xDEADBEEF

# --- Payload Specs (Dual-Channel Interleaved Complex IQ) ---
# Each 64-bit AXI beat = 1 complex sample [I_float32, Q_float32] (8 bytes)
#   Beat 0 (Even): [Ch1_I, Ch1_Q]
#   Beat 1 (Odd) : [Ch2_I, Ch2_Q]
# Each 16-byte Frame = 2 beats = [Ch1_I, Ch1_Q, Ch2_I, Ch2_Q]
BYTES_PER_BEAT = 8
NUM_CHANNELS = 2
BYTES_PER_FRAME = BYTES_PER_BEAT * NUM_CHANNELS          # 16 bytes per dual-ch frame

NUM_FRAMES_PER_PKT = 89                                  # 89 complex samples/ch/pkt
NUM_DATABEATS = NUM_FRAMES_PER_PKT * NUM_CHANNELS        # 178 beats per UDP packet
PAYLOAD_SZ = NUM_DATABEATS * BYTES_PER_BEAT              # 1424 bytes
EXPECTED_PACKET_LEN = HEADER_SZ + PAYLOAD_SZ             # 1440 bytes

# Set to True if SWAP_PAYLOAD_ENDIANNESS is 1 in udp_perf_client.h ('>f4'),
# or False if SWAP_PAYLOAD_ENDIANNESS is 0 in udp_perf_client.h ('<f4')
PAYLOAD_BIG_ENDIAN = False
PAYLOAD_DTYPE = '>f4' if PAYLOAD_BIG_ENDIAN else '<f4'

# --- Output File Configuration ---
SAVE_FOLDER = "data"
this_dir = Path(__file__).parent.resolve()
grandparent_dir = this_dir.parent.parent
run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

SAVE_PATH = grandparent_dir / SAVE_FOLDER
SAVE_PATH.mkdir(parents=True, exist_ok=True)

OUTPUT_FILE_CH1 = SAVE_PATH / f"sdr_ch1_iq_{run_ts}.bin"
OUTPUT_FILE_CH2 = SAVE_PATH / f"sdr_ch2_iq_{run_ts}.bin"
OUTPUT_FILE_TS  = SAVE_PATH / f"sdr_timestamps_{run_ts}.bin"


def run_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Request a 32 MB OS receive buffer (requires net.core.rmem_max >= 33554432 for full effect)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    sock.bind(("0.0.0.0", 5001))
    sock.settimeout(5.0)  # Timeout after 5s once stream starts

    print("[INFO] Listening for Dual-Channel IQ SDR data on UDP port 5001...")
    print(f"[INFO] Payload Endianness: {'Big-Endian (>f4)' if PAYLOAD_BIG_ENDIAN else 'Little-Endian (<f4)'}")
    print(f"[INFO] Saving Ch1 Complex64 to: {OUTPUT_FILE_CH1}")
    print(f"[INFO] Saving Ch2 Complex64 to: {OUTPUT_FILE_CH2}")

    prev_packet_id = None
    total_packets = 0
    total_dropped_packets = 0
    invalid_sync_cnt = 0
    skipped_events = []

    start_time = None
    last_report_time = None

    # Use 1 MB file write buffers to minimize disk I/O stalls during capture
    with open(OUTPUT_FILE_CH1, "wb", buffering=1024 * 1024) as f_ch1, \
         open(OUTPUT_FILE_CH2, "wb", buffering=1024 * 1024) as f_ch2, \
         open(OUTPUT_FILE_TS, "wb", buffering=64 * 1024) as f_ts:

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

                # 1. Unpack Header: signed int32 packet_id ('i'), uint32 sync_word ('I'), uint64 timestamp ('Q')
                packet_id, sync_word, hw_timestamp = struct.unpack(">iIQ", data[:HEADER_SZ])

                # Check for C client's FINISH packet (packet_id == -1)
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

                # 4. Unpack Payload into shape (89 frames, 2 channels, 2 IQ components)
                #    Axis 0: Frame index (0..88)
                #    Axis 1: Channel index (0 = Ch1, 1 = Ch2)
                #    Axis 2: Component (0 = I, 1 = Q)
                raw_payload = memoryview(data)[HEADER_SZ:EXPECTED_PACKET_LEN]
                frames = np.frombuffer(raw_payload, dtype=PAYLOAD_DTYPE).reshape(
                    NUM_FRAMES_PER_PKT, NUM_CHANNELS, 2
                )

                # Convert to native Little-Endian float32 (<f4) for GNU Radio complex64 compatibility
                ch1_iq = np.ascontiguousarray(frames[:, 0, :], dtype='<f4')
                ch2_iq = np.ascontiguousarray(frames[:, 1, :], dtype='<f4')

                # 5. Write raw interleaved [I0, Q0, I1, Q1, ...] float32 bytes to disk
                f_ch1.write(ch1_iq)
                f_ch2.write(ch2_iq)
                f_ts.write(struct.pack("<iQ", packet_id, hw_timestamp))

                # 6. Periodic status report once per second (avoids blocking UDP loop)
                if (now - last_report_time) >= 1.0:
                    elapsed = now - start_time
                    mb_received = (total_packets * PAYLOAD_SZ) / (1024 * 1024)
                    mbps = (total_packets * EXPECTED_PACKET_LEN * 8) / (elapsed * 1e6)
                    ch1_i0, ch1_q0 = ch1_iq[0, 0], ch1_iq[0, 1]
                    ch2_i0, ch2_q0 = ch2_iq[0, 0], ch2_iq[0, 1]
                    print(
                        f"\r[STATUS] Pkts: {total_packets:7d} | "
                        f"Data: {mb_received:6.2f} MB | "
                        f"Rate: {mbps:6.2f} Mbps | "
                        f"Dropped: {total_dropped_packets:4d} | "
                        f"Ch1[0]: {ch1_i0:+1.8f}{ch1_q0:+1.8f}j | "
                        f"Ch2[0]: {ch2_i0:+1.8f}{ch2_q0:+1.8f}j",
                        end="",
                        flush=True,
                    )
                    last_report_time = now

        except KeyboardInterrupt:
            print("\n[INFO] Capture stopped by user.")
        finally:
            sock.close()

    # --- Final Capture Summary ---
    total_samples_per_ch = total_packets * NUM_FRAMES_PER_PKT
    print("\n" + "=" * 60)
    print("CAPTURE SUMMARY")
    print("=" * 60)
    print(f"Total Packets Captured       : {total_packets}")
    print(f"Complex IQ Samples / Channel : {total_samples_per_ch}")
    print(f"Total Dropped Packets        : {total_dropped_packets} ({len(skipped_events)} gap events)")
    print(f"Invalid Sync Words           : {invalid_sync_cnt}")
    if skipped_events:
        print(f"First 10 Gap Events (prev_id -> curr_id, missed): {skipped_events[:10]}")
    print("=" * 60)


if __name__ == "__main__":
    run_receiver()