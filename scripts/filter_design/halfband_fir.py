#!/usr/bin/env python3
"""
Vivado FIR Compiler (.coe) Generator for Half-Band Decimation (M = 2)
Target: Eclypse Z7 + Zmod SDR
"""

from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def design_halfband_fir(
    fs_hz: float,
    num_taps: int,
    coef_bits: int,
    stopband_atten_db: float,
):
    if (num_taps + 1) % 4 != 0:
        num_taps = int(np.ceil((num_taps + 1) / 4) * 4 - 1)
        print(f"[INFO] Adjusted num_taps to {num_taps} to satisfy Half-Band constraint (4k + 3).")

    fs_out = fs_hz / 2.0
    f_nyq_in = fs_hz / 2.0
    f_nyq_out = fs_out / 2.0

    beta = signal.kaiser_beta(stopband_atten_db)
    h_float = signal.firwin(num_taps, cutoff=0.5, window=("kaiser", beta), scale=True)

    center = (num_taps - 1) // 2
    zero_mask = np.zeros(num_taps, dtype=bool)
    for i in range(num_taps):
        if i != center and ((i - center) % 2 == 0):
            zero_mask[i] = True
    h_float[zero_mask] = 0.0
    h_float = h_float / np.sum(h_float)

    max_int = (1 << (coef_bits - 1)) - 1
    peak_val = np.max(np.abs(h_float))
    h_quant_int = np.round((h_float / peak_val) * max_int).astype(np.int64)
    h_quant_int[zero_mask] = 0
    h_quant_int = np.clip(h_quant_int, -(1 << (coef_bits - 1)), max_int)

    dc_gain_int = int(np.sum(h_quant_int))
    h_quant_norm = h_quant_int / float(dc_gain_int)
    h_quant_norm[zero_mask] = 0.0

    w_est, H_est = signal.freqz(h_quant_norm, worN=16384, fs=fs_hz)
    mag_db = 20.0 * np.log10(np.maximum(np.abs(H_est), 1e-12))
    pb_idx = np.where(np.abs(mag_db) > 0.05)[0]
    f_pass = w_est[pb_idx[0]] if len(pb_idx) > 0 else f_nyq_out * 0.7
    f_stop = f_nyq_in - f_pass

    meta = {
        "fs_hz": fs_hz,
        "fs_out": fs_out,
        "f_nyq_in": f_nyq_in,
        "f_nyq_out": f_nyq_out,
        "f_pass": f_pass,
        "f_stop": f_stop,
        "num_taps": num_taps,
        "nonzero_taps": int(np.count_nonzero(h_quant_int)),
        "coef_bits": coef_bits,
        "dc_gain_int": dc_gain_int,
    }
    return h_float, h_quant_int, h_quant_norm, meta


def write_vivado_coe(filepath: Path, coeffs_norm: np.ndarray, meta: dict):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("; ===========================================================================\n")
        f.write("; Vivado FIR Compiler Half-Band Decimation Coefficient File (.coe)\n")
        f.write(f"; Input Sample Rate  : {meta['fs_hz'] / 1e3:.2f} kSPS\n")
        f.write(f"; Output Sample Rate : {meta['fs_out'] / 1e3:.2f} kSPS (M = 2)\n")
        f.write(f"; Total Taps         : {meta['num_taps']} ({meta['nonzero_taps']} non-zero taps)\n")
        f.write(f"; Coefficient Width  : {meta['coef_bits']}-bit Quantized Float (DC Sum = 1.0)\n")
        f.write("; ===========================================================================\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")

        coeff_strs = ["0.0" if c == 0.0 else f"{float(c):.16f}" for c in coeffs_norm]
        lines = [", ".join(coeff_strs[i : i + 6]) for i in range(0, len(coeff_strs), 6)]
        f.write(",\n".join(lines) + ";\n")

    print(f"[INFO] Saved Half-Band .coe to: {filepath.resolve()}")


def plot_halfband_response(h_float: np.ndarray, h_quant_norm: np.ndarray, meta: dict):
    n_fft = 16384
    freqs_hz, H_flt = signal.freqz(h_float, worN=n_fft, fs=meta["fs_hz"])
    _, H_qnt = signal.freqz(h_quant_norm, worN=n_fft, fs=meta["fs_hz"])

    to_db = lambda x: 20.0 * np.log10(np.maximum(np.abs(x), 1e-12))
    mag_flt_db, mag_qnt_db = to_db(H_flt), to_db(H_qnt)

    freqs_khz = freqs_hz / 1e3
    f_pass_khz = meta["f_pass"] / 1e3
    f_stop_khz = meta["f_stop"] / 1e3
    f_nyq_out_khz = meta["f_nyq_out"] / 1e3

    half = n_fft // 2
    folded_alias_db = mag_qnt_db[half:][::-1]

    fig, axes = plt.subplots(2, 1, figsize=(11, 8.5), constrained_layout=True)
    fig.suptitle(
        f"Half-Band FIR Decimator (M = 2, {meta['num_taps']} Taps / {meta['nonzero_taps']} Non-Zero, {meta['coef_bits']}-bit)\n"
        f"{meta['fs_hz']/1e3:.1f} kSPS -> {meta['fs_out']/1e3:.1f} kSPS",
        fontsize=12.5,
        fontweight="bold",
    )

    ax1 = axes[0]
    ax1.plot(freqs_khz, mag_flt_db, label="Ideal (Float64)", color="#1f77b4", linewidth=1.4)
    ax1.plot(freqs_khz, mag_qnt_db, label=f"Quantized ({meta['coef_bits']}-bit)", color="#ff7f0e", linestyle="--", linewidth=1.4)
    ax1.plot(freqs_khz[:half], folded_alias_db, label="Folded Alias", color="crimson", linestyle="-.", linewidth=1.4)
    ax1.axvspan(f_pass_khz, f_stop_khz, color="orange", alpha=0.15, label="Transition Band")
    ax1.axvline(f_nyq_out_khz, color="black", linestyle=":", label=f"Output Nyquist ({f_nyq_out_khz:.1f} kHz)")
    ax1.set_xlim(0, meta["f_nyq_in"] / 1e3)
    ax1.set_ylim(-120, 5)
    ax1.set_xlabel("Frequency (kHz)")
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, which="both", alpha=0.4)
    ax1.legend(loc="upper right", fontsize=9)

    ax2 = axes[1]
    pb_mask = freqs_khz <= f_pass_khz
    ax2.plot(freqs_khz[pb_mask], mag_flt_db[pb_mask], label="Ideal (Float64)", color="#1f77b4", linewidth=1.5)
    ax2.plot(freqs_khz[pb_mask], mag_qnt_db[pb_mask], label=f"Quantized ({meta['coef_bits']}-bit)", color="#ff7f0e", linestyle="--", linewidth=1.5)
    max_rip = np.max(np.abs(mag_qnt_db[pb_mask]))
    ax2.set_title(f"In-Band Passband Ripple (Peak Deviation: +/- {max_rip:.5f} dB)")
    ax2.set_xlim(0, f_pass_khz)
    ax2.set_ylim(-max(0.01, max_rip * 2.0), max(0.01, max_rip * 2.0))
    ax2.set_xlabel("Frequency (kHz)")
    ax2.set_ylabel("Magnitude (dB)")
    ax2.grid(True, which="both", alpha=0.4)
    ax2.legend(loc="upper right", fontsize=9)

    plt.show()


if __name__ == "__main__":
    # =========================================================================
    # HARDCODED CONFIGURATION PARAMETERS
    # =========================================================================
    FS_HZ             = 384e3   # Input sample rate in Hz (e.g., 384e3 or 122.88e6)
    NUM_TAPS          = 31      # Total taps; must be 4k + 3 (e.g., 15, 23, 31, 47)
    COEF_BITS         = 18      # Coefficient quantization bit width (18-bit for DSP48E1)
    STOPBAND_ATTEN_DB = 90.0    # Target Kaiser window stopband attenuation in dB
    OUTPUT_FILE       = Path(f"fir_hb_{int(FS_HZ/1e3)}k_{NUM_TAPS}taps_{COEF_BITS}b.coe")
    # =========================================================================

    h_flt, h_q_int, h_q_norm, info = design_halfband_fir(
        fs_hz=FS_HZ,
        num_taps=NUM_TAPS,
        coef_bits=COEF_BITS,
        stopband_atten_db=STOPBAND_ATTEN_DB,
    )

    write_vivado_coe(OUTPUT_FILE, h_q_norm, info)
    plot_halfband_response(h_flt, h_q_norm, info)