#!/usr/bin/env python3
"""
Complex Bandpass FIR .coe Generator (95-100 dB Attenuation, Shared Float Normalization)
Target: Vivado FIR Compiler (Maximize_Dynamic_Range, 18-bit Coeffs, Non_Symmetric)
"""

from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def design_high_atten_complex_bp(
    fs_in_hz: float = 122.88e6,
    fc_hz: float = 23.98e6,       # 98.9 MHz FM aliases to 23.98 MHz
    bw_hz: float = 175e3,         # 175 kHz complex passband width (+/- 87.5 kHz)
    num_taps: int = 512,          # 512 taps (16 taps/branch at M=32)
    stopband_atten_db: float = 96.0,
    coef_bits: int = 18,          # 18-bit matches Zynq DSP48E1 multiplier (95+ dB floor)
):
    n = np.arange(num_taps, dtype=np.float64)
    mid = (num_taps - 1) / 2.0
    t = n - mid

    # 1. Prototype Low-Pass with Kaiser window for 96 dB attenuation
    beta = signal.kaiser_beta(stopband_atten_db)
    fc_lp_norm = (bw_hz / 2.0) / fs_in_hz
    h_lp = 2.0 * fc_lp_norm * np.sinc(2.0 * fc_lp_norm * t)
    h_lp *= signal.windows.kaiser(num_taps, beta=beta)
    h_lp /= np.sum(h_lp)

    # 2. Modulate to target center frequency fc_hz
    phase = 2.0 * np.pi * (fc_hz / fs_in_hz) * t
    h_i_flt = h_lp * np.cos(phase)
    h_q_flt = -h_lp * np.sin(phase)

    # 3. Quantize I and Q using a SINGLE SHARED scale factor to preserve I/Q balance
    max_int = (1 << (coef_bits - 1)) - 1  # 131071 for 18-bit
    shared_peak = max(np.max(np.abs(h_i_flt)), np.max(np.abs(h_q_flt)))

    h_i_int = np.clip(np.round((h_i_flt / shared_peak) * max_int), -(max_int + 1), max_int)
    h_q_int = np.clip(np.round((h_q_flt / shared_peak) * max_int), -(max_int + 1), max_int)

    # 4. Normalize as floats for Vivado using a SHARED coherent passband gain denominator
    # (Prevents I/Q amplitude mismatch while setting passband gain at fc_hz to 0.00 dB)
    shared_coherent_gain = np.abs(np.sum((h_i_int + 1j * h_q_int) * np.exp(-1j * phase)))

    # n = np.arange(num_taps, dtype=np.float64)
    # w_c = 2.0 * np.pi * (fc_hz / fs_in_hz)
    # shared_passband_gain = np.abs(np.sum(h_i_int * np.cos(w_c * (n - (num_taps - 1) / 2.0))))

    h_i_norm = h_i_int / shared_coherent_gain
    h_q_norm = h_q_int / shared_coherent_gain

    return h_i_norm, h_q_norm, h_i_int, h_q_int


def write_float_coe(filepath: Path, taps_norm: np.ndarray, label: str):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"; Vivado Complex Bandpass FIR ({label}) - Normalized Float (0 dB Passband)\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")
        strs = [f"{c:0.24f}" for c in taps_norm]
        lines = [", ".join(strs[i : i + 4]) for i in range(0, len(strs), 4)]
        f.write(",\n".join(lines) + ";\n")
    print(f"[INFO] Wrote {filepath.name} ({len(taps_norm)} taps)")


if __name__ == "__main__":
    hi_norm, hq_norm, hi_int, hq_int = design_high_atten_complex_bp()
    write_float_coe(Path("complex_fir_I_175k_96dB.coe"), hi_norm, "I-Channel")
    write_float_coe(Path("complex_fir_Q_175k_96dB.coe"), hq_norm, "Q-Channel")





# #!/usr/bin/env python3
# """
# 1. Generates initial 256-tap Complex Bandpass FIR .coe files (I and Q) for Vivado
#    in Integer_Coefficients mode (16-bit signed, 15 fractional bits).
# 2. Parses Vivado's <fir_ip>_reload_order.txt into a C header array for Vitis FreeRTOS.
# """

# import argparse
# from pathlib import Path
# import numpy as np
# from scipy import signal


# def generate_complex_bp_taps(
#     fs_in_hz: float = 122.88e6,
#     fc_hz: float = 23.98e6,      # e.g. 98.9 MHz FM aliases to 122.88 - 98.9 = 23.98 MHz
#     bw_hz: float = 175e3,        # Total complex passband width (+/- 125 kHz)
#     num_taps: int = 512,         # 16 taps per polyphase branch at M=32 #  Multiple of M=32 -> 8 taps per polyphase branch
#     kaiser_beta: float = signal.kaiser_beta(96.0)  # beta = 9.61 for 96 dB attenuation
# ):
#     # 1. Design prototype low-pass filter with cutoff = bw_hz / 2
#     n = np.arange(num_taps, dtype=np.float64)
#     mid = (num_taps - 1) / 2.0
#     t = n - mid

#     # Normalized sinc low-pass prototype: f_c_lp = (bw_hz / 2) / fs_in_hz
#     fc_lp_norm = (bw_hz / 2.0) / fs_in_hz
#     h_lp = 2.0 * fc_lp_norm * np.sinc(2.0 * fc_lp_norm * t)
#     h_lp *= signal.windows.kaiser(num_taps, beta=kaiser_beta)

#     # Normalize prototype low-pass so DC sum == 1.0
#     h_lp /= np.sum(h_lp)

#     # 2. Complex modulation to shift +fc_hz down to 0 Hz (or equivalently pass +fc_hz)
#     # Using exp(+j * 2*pi*fc*t / fs_in) for complex bandpass filter taps:
#     phase = 2.0 * np.pi * (fc_hz / fs_in_hz) * t
#     h_i_float = h_lp * np.cos(phase)
#     h_q_float = -h_lp * np.sin(phase)

#     # 3. Quantize to Q1.15 signed 16-bit integers [-32768, 32767]
#     # Because sum(h_lp) = 1.0, scaling by 32767 * 2.0 gives 0 dB passband gain for real input
#     scale = 32767.0
#     h_i_int16 = np.clip(np.round(h_i_float * scale), -32768, 32767).astype(np.int16)
#     h_q_int16 = np.clip(np.round(h_q_float * scale), -32768, 32767).astype(np.int16)

#     # return h_i_int16, h_q_int16

#     ## normalize as floats for Vivado
#     # 1. Quantize to integer grid first using a shared scale factor
#     peak_val = max(np.max(np.abs(h_i_float)), np.max(np.abs(h_q_float)))
#     coef_bits = 16
#     max_int = (1 << (coef_bits - 1)) - 1  # 32767 for 16-bit, 131071 for 18-bit

#     h_i_int = np.round((h_i_float / peak_val) * max_int)
#     h_q_int = np.round((h_q_float / peak_val) * max_int)

#     # 2. Compute the true coherent passband gain at fc_hz (SHARED denominator!)
#     n = np.arange(num_taps, dtype=np.float64)
#     w_c = 2.0 * np.pi * (fc_hz / fs_in_hz)
#     shared_passband_gain = np.abs(np.sum(h_i_int * np.cos(w_c * (n - (num_taps - 1) / 2.0))))

#     # 3. Normalize both I and Q by the EXACT SAME scalar for 0 dB passband in Vivado
#     h_i_norm = h_i_int / shared_passband_gain
#     h_q_norm = h_q_int / shared_passband_gain

#     return h_i_norm, h_q_norm


# def write_int_coe(filepath: Path, taps: np.ndarray, label: str):
#     with open(filepath, "w", encoding="utf-8") as f:
#         f.write(f"; Vivado FIR Compiler - {label} Initial Complex Bandpass Taps (Q1.15 Integer)\n")
#         f.write("radix=10;\n")
#         f.write("coefdata=\n")
#         strs = [str(int(c)) for c in taps]
#         # strs = [f"{c:0.24f}" for c in taps]
#         lines = [", ".join(strs[i : i + 8]) for i in range(0, len(strs), 8)]
#         f.write(",\n".join(lines) + ";\n")
#     print(f"[INFO] Wrote {filepath.name} ({len(taps)} integer taps)")


# def export_reload_order_header(reload_txt_path: Path, header_out_path: Path, num_taps: int):
#     """
#     Converts Vivado's _reload_order.txt into a C lookup table `FIR_RELOAD_ORDER[]`.
#     If no _reload_order.txt is provided yet, generates the standard polyphase
#     M=32 decimation order table as a placeholder.
#     """
#     if reload_txt_path and reload_txt_path.exists():
#         indices = []
#         with open(reload_txt_path, "r", encoding="utf-8") as f:
#             for line in f:
#                 line = line.strip()
#                 if not line or line.startswith("#") or line.startswith(";"):
#                     continue
#                 # Extract integer coefficient index from lines like "Reload index 0 : Coefficient 31"
#                 parts = [int(s) for s in line.replace(":", " ").split() if s.isdigit()]
#                 if parts:
#                     indices.append(parts[-1])
#         print(f"[INFO] Parsed {len(indices)} reload indices from {reload_txt_path}")
#     else:
#         # Default sequential placeholder until Vivado generates <ip>_reload_order.txt
#         indices = list(range(num_taps))
#         print("[WARN] No _reload_order.txt supplied; wrote sequential placeholder table.")

#     with open(header_out_path, "w", encoding="utf-8") as f:
#         f.write("#ifndef FIR_RELOAD_ORDER_H\n#define FIR_RELOAD_ORDER_H\n\n")
#         f.write("#include <stdint.h>\n\n")
#         f.write(f"#define FIR_NUM_TAPS {len(indices)}\n\n")
#         f.write("static const uint16_t FIR_RELOAD_ORDER[FIR_NUM_TAPS] = {\n    ")
#         lines = [
#             ", ".join(f"{idx:3d}" for idx in indices[i : i + 8])
#             for i in range(0, len(indices), 8)
#         ]
#         f.write(",\n    ".join(lines) + "\n};\n\n#endif\n")
#     print(f"[INFO] Wrote C reload order header: {header_out_path}")


# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--taps", type=int, default=256)
#     parser.add_argument("--reload-txt", type=str, default=None, help="Path to Vivado _reload_order.txt")
#     args = parser.parse_args()

#     hi, hq = generate_complex_bp_taps(num_taps=args.taps)
#     write_int_coe(Path("complex_fir_I_init.coe"), hi, "In-Phase (I)")
#     write_int_coe(Path("complex_fir_Q_init.coe"), hq, "Quadrature (Q)")

#     reload_path = Path(args.reload_txt) if args.reload_txt else None
#     export_reload_order_header(reload_path, Path("fir_reload_order.h"), args.taps)










# #!/usr/bin/env python3
# """
# Bandpass Decimation (Mixerless Frequency-Translating FIR) .coe Generator
# Target: Eclypse Z7 + Zmod SDR (fs_in = 122.88 MSPS, M = 32 -> 3.84 MSPS)
# """

# import argparse
# from pathlib import Path
# import numpy as np
# from scipy import signal
# import matplotlib.pyplot as plt


# def design_bandpass_decimator(
#     fs_in_hz: float = 122.88e6,
#     m_decim: int = 32,
#     rf_center_hz: float = 98.9e6,
#     bandwidth_hz: float = 1.5e6,
#     num_taps: int = 255,
#     coef_bits: int = 16,
#     kaiser_atten_db: float = 75.0,
# ):
#     fs_out = fs_in_hz / m_decim
#     f_nyq_out = fs_out / 2.0  # 1.92 MHz zone width for M=32

#     # 1. Map RF frequency (e.g. 88-108 MHz in 2nd ADC Nyquist zone) to 1st ADC Nyquist zone [0, 61.44 MHz]
#     f_adc_nyq1 = abs(rf_center_hz - round(rf_center_hz / fs_in_hz) * fs_in_hz)
#     adc_zone = int(rf_center_hz // (fs_in_hz / 2.0)) + 1

#     # 2. Identify which decimation Nyquist zone k (each of width f_nyq_out = 1.92 MHz) it falls into
#     k_zone = int(f_adc_nyq1 // f_nyq_out)
#     zone_low = k_zone * f_nyq_out
#     zone_high = (k_zone + 1) * f_nyq_out
#     is_inverted = (k_zone % 2 == 1) ^ (adc_zone % 2 == 0)

#     # Calculate where rf_center_hz will land in the [0, 1.92 MHz] decimated output
#     if k_zone % 2 == 0:
#         f_baseband_out = f_adc_nyq1 - zone_low
#     else:
#         f_baseband_out = zone_high - f_adc_nyq1

#     print(f"=== Bandpass Decimation Frequency Plan (M={m_decim}) ===")
#     print(f"Target RF Center         : {rf_center_hz/1e6:.3f} MHz (ADC Nyquist Zone {adc_zone})")
#     print(f"1st-Nyquist Aliased Freq : {f_adc_nyq1/1e6:.3f} MHz")
#     print(f"Decimation Zone k={k_zone:<2d}     : [{zone_low/1e6:.3f} MHz .. {zone_high/1e6:.3f} MHz]")
#     print(f"Final Baseband Location  : {f_baseband_out/1e3:.1f} kHz ( inside [0 .. {f_nyq_out/1e3:.1f} kHz] )")
#     print(f"Spectral Orientation     : {'INVERTED (Flip FM sign / conjugate in GRC)' if is_inverted else 'UPRIGHT (Normal)'}")

#     # Check if requested passband crosses a 1.92 MHz zone boundary
#     f_low = f_adc_nyq1 - (bandwidth_hz / 2.0)
#     f_high = f_adc_nyq1 + (bandwidth_hz / 2.0)
#     if f_low <= zone_low or f_high >= zone_high:
#         print(
#             f"[WARNING] Passband [{f_low/1e6:.3f}, {f_high/1e6:.3f}] MHz crosses the "
#             f"zone boundary [{zone_low/1e6:.3f}, {zone_high/1e6:.3f}] MHz!\n"
#             f"          Centering filter on Zone {k_zone} midpoint ({(zone_low+zone_high)/2e6:.3f} MHz) to prevent self-aliasing."
#         )
#         f_bp_center = 0.5 * (zone_low + zone_high)
#     else:
#         f_bp_center = f_adc_nyq1

#     # 3. Design Prototype Low-Pass Filter with single-sided cutoff = bandwidth_hz / 2
#     beta = signal.kaiser_beta(kaiser_atten_db)
#     h_lp = signal.firwin(
#         num_taps,
#         cutoff=bandwidth_hz / 2.0,
#         window=("kaiser", beta),
#         fs=fs_in_hz,
#         scale=True,
#     )

#     # 4. Frequency-translate (heterodyne) the low-pass taps to f_bp_center
#     n_idx = np.arange(num_taps) - (num_taps - 1) / 2.0
#     # Real bandpass taps: 2 * h_lp[n] * cos(2*pi*f_c*n / fs_in)
#     h_bp_real = 2.0 * h_lp * np.cos(2.0 * np.pi * f_bp_center * n_idx / fs_in_hz)

#     # Complex bandpass taps (optional I/Q paths for boundary-free translation)
#     h_bp_i = h_lp * np.cos(2.0 * np.pi * f_adc_nyq1 * n_idx / fs_in_hz)
#     h_bp_q = -h_lp * np.sin(2.0 * np.pi * f_adc_nyq1 * n_idx / fs_in_hz)

#     # Normalize real bandpass taps so peak passband gain at f_bp_center == 1.0 (0 dB)
#     w_center = 2.0 * np.pi * f_bp_center / fs_in_hz
#     center_gain = np.abs(np.sum(h_bp_real * np.exp(-1j * w_center * np.arange(num_taps))))
#     h_bp_real /= center_gain

#     # Quantize to B-bit signed integer grid
#     max_int = (1 << (coef_bits - 1)) - 1
#     peak_abs = np.max(np.abs(h_bp_real))
#     h_q_int = np.round((h_bp_real / peak_abs) * max_int).astype(np.int64)
#     h_q_int = np.clip(h_q_int, -(1 << (coef_bits - 1)), max_int)

#     # Re-normalize quantized float values so passband gain at f_bp_center is exactly 1.0 (0 dB)
#     q_center_gain = np.abs(np.sum(h_q_int * np.exp(-1j * w_center * np.arange(num_taps))))
#     h_q_norm = h_q_int / q_center_gain

#     return h_bp_real, h_q_norm, (h_bp_i, h_bp_q), {
#         "fs_in_hz": fs_in_hz,
#         "fs_out": fs_out,
#         "m_decim": m_decim,
#         "rf_center_hz": rf_center_hz,
#         "f_adc_nyq1": f_adc_nyq1,
#         "f_bp_center": f_bp_center,
#         "f_baseband_out": f_baseband_out,
#         "k_zone": k_zone,
#         "zone_low": zone_low,
#         "zone_high": zone_high,
#         "is_inverted": is_inverted,
#         "num_taps": num_taps,
#         "coef_bits": coef_bits,
#     }


# def write_coe(filepath: Path, coeffs: np.ndarray, meta: dict):
#     with open(filepath, "w", encoding="utf-8") as f:
#         f.write("; ===========================================================================\n")
#         f.write("; Vivado FIR Compiler - Bandpass Decimator (Mixerless DDC) (.coe)\n")
#         f.write(f"; Target RF Frequency : {meta['rf_center_hz']/1e6:.3f} MHz\n")
#         f.write(f"; 1st-Nyquist Center  : {meta['f_bp_center']/1e6:.3f} MHz (Zone k={meta['k_zone']})\n")
#         f.write(f"; Baseband Output Freq: {meta['f_baseband_out']/1e3:.1f} kHz (fs_out = {meta['fs_out']/1e6:.2f} MSPS)\n")
#         f.write("; ===========================================================================\n")
#         f.write("radix=10;\n")
#         f.write("coefdata=\n")
#         coeff_strs = [f"{float(c):.18f}" for c in coeffs]
#         lines = [", ".join(coeff_strs[i : i + 4]) for i in range(0, len(coeff_strs), 4)]
#         f.write(",\n".join(lines) + ";\n")
#     print(f"[INFO] Wrote Bandpass Decimator .coe file: {filepath.resolve()}")


# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--rf", type=float, default=98.9e6, help="Target RF center frequency in Hz (default: 98.9 MHz)")
#     parser.add_argument("--bw", type=float, default=1.4e6, help="Usable passband width in Hz (default: 1.4 MHz)")
#     parser.add_argument("--taps", type=int, default=255, help="Number of FIR taps (default: 255)")
#     args = parser.parse_args()

#     h_flt, h_q_norm, _, info = design_bandpass_decimator(
#         rf_center_hz=args.rf, bandwidth_hz=args.bw, num_taps=args.taps
#     )
#     write_coe(Path(f"bp_decim_{args.rf/1e6:.1f}MHz_M32.coe"), h_q_norm, info)