import sys
import socket
import struct
import time
from pathlib import Path
from datetime import datetime

import numpy as np
import sounddevice as sd

# --- Header Offsets (Matching udp_perf_client.c) ---
PACKET_ID_SZ = 4
SYNC_WORD_SZ = 4
TIMESTAMP_SZ = 8
HEADER_SZ = PACKET_ID_SZ + SYNC_WORD_SZ + TIMESTAMP_SZ  # 16 bytes

SYNC_WORD = 0xDEADBEEF

# --- Payload Specs (Dual-Channel Interleaved 32-bit Audio) ---
# Each 32-bit word = 1 real audio sample (4 bytes)
#   Word 0 (Even): Ch1 Audio Sample
#   Word 1 (Odd) : Ch2 Audio Sample
# Each 8-byte Frame = 2 words = [Ch1, Ch2]
BYTES_PER_SAMPLE = 4
NUM_CHANNELS = 2
BYTES_PER_FRAME = BYTES_PER_SAMPLE * NUM_CHANNELS       # 8 bytes per dual-ch frame

NUM_FRAMES_PER_PKT = 178                                # 178 audio samples/ch/pkt
PAYLOAD_SZ = NUM_FRAMES_PER_PKT * BYTES_PER_FRAME       # 1424 bytes
EXPECTED_PACKET_LEN = HEADER_SZ + PAYLOAD_SZ            # 1440 bytes

# Audio & Payload Format Configuration
AUDIO_SAMPLE_RATE = 48000
PAYLOAD_BIG_ENDIAN = False   # False for zero-copy SG DMA in udp_perf_client.c
PAYLOAD_IS_FLOAT = False     # Set True if Audio FIR outputs float32 instead of int32

_endian = '>' if PAYLOAD_BIG_ENDIAN else '<'
_kind = 'f4' if PAYLOAD_IS_FLOAT else 'i4'
PAYLOAD_DTYPE = f"{_endian}{_kind}"
SD_DTYPE = 'float32' if PAYLOAD_IS_FLOAT else 'int32'
NATIVE_DTYPE = np.float32 if PAYLOAD_IS_FLOAT else np.int32

# --- Output File Configuration ---
SAVE_FOLDER = "data_records"
this_dir = Path(__file__).parent.resolve()
run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

SAVE_PATH = this_dir / SAVE_FOLDER
SAVE_PATH.mkdir(parents=True, exist_ok=True)

fmt_tag = "f32" if PAYLOAD_IS_FLOAT else "s32"
OUTPUT_FILE_CH1 = SAVE_PATH / f"sdr_ch1_audio_{AUDIO_SAMPLE_RATE//1000}k_{fmt_tag}_{run_ts}.bin"
OUTPUT_FILE_CH2 = SAVE_PATH / f"sdr_ch2_audio_{AUDIO_SAMPLE_RATE//1000}k_{fmt_tag}_{run_ts}.bin"
OUTPUT_FILE_TS  = SAVE_PATH / f"sdr_audio_timestamps_{run_ts}.bin"


# --- Exact Q4.28 + FM Deviation Scaling (CORDIC in Radians) ---
FRAC_SCALE     = float(1 << 28)                          # 268,435,456.0 (28 fractional bits)
FS_DEMOD       = 384000.0                                # 122.88 MSPS / 160 / 2
MAX_DEV_HZ     = 75000.0                                 # Standard broadcast FM max deviation
MAX_PHASE_STEP = 2.0 * np.pi * MAX_DEV_HZ / FS_DEMOD     # 1.22718463... rad/sample
TOTAL_SCALE    = FRAC_SCALE * MAX_PHASE_STEP             # 329,419,866.9

class FixedFMNormalizer:
    def __init__(self, total_scale=TOTAL_SCALE, dc_alpha=0.999, volume=0.8):
        self.inv_scale = volume / total_scale
        self.dc_alpha = dc_alpha
        self.dc_est = 0.0

    def process(self, pcm_int32: np.ndarray) -> np.ndarray:
        x = pcm_int32.astype(np.float64)
        # Remove DC offset caused by any slight NCO-to-carrier frequency error
        self.dc_est = self.dc_alpha * self.dc_est + (1.0 - self.dc_alpha) * np.mean(x)
        x -= self.dc_est
        return np.clip(x * self.inv_scale, -1.0, 1.0).astype(np.float32)
    


def run_receiver(live_channel: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    sock.bind(("0.0.0.0", 5001))
    sock.settimeout(5.0)

    print(f"[INFO] Listening for interleaved dual-channel SDR audio on UDP port 5001...")
    print(f"[INFO] Payload Format: {PAYLOAD_DTYPE} | Live Playback: Ch{live_channel} @ {AUDIO_SAMPLE_RATE} Hz")
    print(f"[INFO] Saving Ch1 to: {OUTPUT_FILE_CH1}")
    print(f"[INFO] Saving Ch2 to: {OUTPUT_FILE_CH2}")

    # audio_stream = sd.OutputStream(
    #     samplerate=AUDIO_SAMPLE_RATE,
    #     channels=1,
    #     dtype=SD_DTYPE,
    # )
    # audio_stream.start()

    audio_stream = sd.OutputStream(
        samplerate=AUDIO_SAMPLE_RATE,
        channels=1,
        dtype='float32',
    )
    normalizer = FixedFMNormalizer(volume=0.8)
    audio_stream.start()

    norm_ch1 = FixedFMNormalizer(volume=0.8)
    norm_ch2 = FixedFMNormalizer(volume=0.8)

    prev_packet_id = None
    total_packets = 0
    total_dropped_packets = 0
    invalid_sync_cnt = 0
    skipped_events = []

    start_time = None
    last_report_time = None

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
                    print("[INFO] First audio packet received! Streaming...")

                # 1. Unpack Header: signed int32 packet_id, uint32 sync_word, uint64 timestamp
                packet_id, sync_word, hw_timestamp = struct.unpack(">iIQ", data[:HEADER_SZ])

                # Check for C client's FINISH marker (packet_id == -1)
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

                # 4. De-interleave Payload into shape (178 frames, 2 channels)
                #    Column 0 = Ch1, Column 1 = Ch2
                raw_payload = memoryview(data)[HEADER_SZ:EXPECTED_PACKET_LEN]
                frames = np.frombuffer(raw_payload, dtype=PAYLOAD_DTYPE).reshape(
                    NUM_FRAMES_PER_PKT, NUM_CHANNELS
                )

                ch1_pcm = np.ascontiguousarray(frames[:, 0], dtype=NATIVE_DTYPE)
                ch2_pcm = np.ascontiguousarray(frames[:, 1], dtype=NATIVE_DTYPE)

                # 5. Normalize both channels to float32 in [-1.0, +1.0]
                ch1_f32 = norm_ch1.process(ch1_pcm)
                ch2_f32 = norm_ch2.process(ch2_pcm)

                # 6. Play selected channel live & save normalized float32 streams to disk
                audio_stream.write(ch1_f32 if live_channel == 1 else ch2_f32)
                f_ch1.write(ch1_f32.tobytes())
                f_ch2.write(ch2_f32.tobytes())
                f_ts.write(struct.pack("<iQ", packet_id, hw_timestamp))
                
                # 7. Periodic status report once per second
                if (now - last_report_time) >= 1.0:
                    elapsed = now - start_time
                    mb_received = (total_packets * PAYLOAD_SZ) / (1024 * 1024)
                    kbps = (total_packets * EXPECTED_PACKET_LEN * 8) / (elapsed * 1e3)
                    print(
                        f"\r[STATUS] Pkts: {total_packets:7d} | "
                        f"Data: {mb_received:6.2f} MB | "
                        f"Rate: {kbps:7.1f} kbps | "
                        f"Dropped: {total_dropped_packets:4d} | "
                        f"Ch1[0]: {ch1_f32[0]} | Ch2[0]: {ch2_f32[0]}",
                        end="",
                        flush=True,
                    )
                    last_report_time = now

        except KeyboardInterrupt:
            print("\n[INFO] Dual-channel audio capture and live playback stopped by user.")
        finally:
            audio_stream.stop()
            audio_stream.close()
            sock.close()

    # --- Final Capture Summary ---
    total_samples_per_ch = total_packets * NUM_FRAMES_PER_PKT
    print("\n" + "=" * 60)
    print("AUDIO CAPTURE SUMMARY")
    print("=" * 60)
    print(f"Total Packets Captured     : {total_packets}")
    print(f"Audio Samples / Channel    : {total_samples_per_ch} ({total_samples_per_ch / AUDIO_SAMPLE_RATE:.2f} s)")
    print(f"Total Dropped Packets      : {total_dropped_packets} ({len(skipped_events)} gap events)")
    print(f"Invalid Sync Words         : {invalid_sync_cnt}")
    if skipped_events:
        print(f"First 10 Gap Events        : {skipped_events[:10]}")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("1", "2"):
        print("Usage: python sdr_udp_audio.py <live_audio_channel>")
        print("Example: python sdr_udp_audio.py 1")
        sys.exit(1)

    run_receiver(int(sys.argv[1]))