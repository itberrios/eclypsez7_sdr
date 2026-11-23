#!/usr/bin/env python3
"""
Vivado FIR Compiler (.coe) Generator for Polyphase Decimation
Target: Eclypse Z7 + Zmod SDR
"""

from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def design_decimation_fir(
    fs_hz: float,
    decimation_factor: int,
    taps_per_phase: int,
    coef_bits: int,
    passband_ratio: float,
    stopband_atten_db: float,
):
    num_taps = decimation_factor * taps_per_phase - 1
    fs_out = fs_hz / decimation_factor
    f_nyq_in = fs_hz / 2.0
    f_nyq_out = fs_out / 2.0

    f_pass = f_nyq_out * passband_ratio
    f_stop = f_nyq_out
    f_cutoff = (f_pass + f_stop) / 2.0

    beta = signal.kaiser_beta(stopband_atten_db)

    h_float = signal.firwin(
        num_taps,
        cutoff=f_cutoff,
        window=("kaiser", beta),
        fs=fs_hz,
        scale=True,
    )

    max_int = (1 << (coef_bits - 1)) - 1
    peak_val = np.max(np.abs(h_float))
    scale_factor = max_int / peak_val

    h_quant_int = np.round(h_float * scale_factor).astype(np.int64)
    h_quant_int = np.clip(h_quant_int, -(1 << (coef_bits - 1)), max_int)

    dc_gain_int = np.sum(h_quant_int)
    h_quant_norm = h_quant_int / dc_gain_int

    meta = {
        "fs_hz": fs_hz,
        "fs_out": fs_out,
        "decimation_factor": decimation_factor,
        "num_taps": num_taps,
        "taps_per_phase": taps_per_phase,
        "coef_bits": coef_bits,
        "f_pass": f_pass,
        "f_stop": f_stop,
        "f_cutoff": f_cutoff,
        "f_nyq_in": f_nyq_in,
        "f_nyq_out": f_nyq_out,
        "dc_gain_int": dc_gain_int,
    }
    return h_float, h_quant_int, h_quant_norm, meta


def write_vivado_coe(filepath: Path, coeffs_int: np.ndarray, meta: dict):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("; ===========================================================================\n")
        f.write("; Vivado FIR Compiler Polyphase Decimation Coefficient File (.coe)\n")
        f.write(f"; Input Sample Rate  : {meta['fs_hz'] / 1e3:.2f} kSPS\n")
        f.write(f"; Decimation Rate (M): {meta['decimation_factor']}\n")
        f.write(f"; Output Sample Rate : {meta['fs_out'] / 1e3:.2f} kSPS\n")
        f.write(f"; Passband Edge      : {meta['f_pass'] / 1e3:.2f} kHz\n")
        f.write(f"; Total Taps         : {meta['num_taps']} ({meta['taps_per_phase']} taps/phase)\n")
        f.write(f"; Coefficient Width  : {meta['coef_bits']}-bit Quantized Float (DC Sum = 1.0)\n")
        f.write("; ===========================================================================\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")

        max_c = np.sum(coeffs_int)
        coeff_strs = [f"{(float(c)/max_c):0.16f}" for c in coeffs_int]
        lines = [", ".join(coeff_strs[i : i + 8]) for i in range(0, len(coeff_strs), 8)]
        f.write(",\n".join(lines) + ";\n")

    print(f"[INFO] Saved Vivado .coe file to: {filepath.resolve()}")


def plot_filter_response(h_float: np.ndarray, h_quant_norm: np.ndarray, meta: dict):
    n_fft = 65536
    freqs_hz, H_float = signal.freqz(h_float, worN=n_fft, fs=meta["fs_hz"])
    _, H_quant = signal.freqz(h_quant_norm, worN=n_fft, fs=meta["fs_hz"])

    mag_float_db = 20.0 * np.log10(np.maximum(np.abs(H_float), 1e-12))
    mag_quant_db = 20.0 * np.log10(np.maximum(np.abs(H_quant), 1e-12))
    freqs_khz = freqs_hz / 1e3

    f_pass_khz = meta["f_pass"] / 1e3
    f_nyq_out_khz = meta["f_nyq_out"] / 1e3

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True)
    fig.suptitle(
        f"Polyphase FIR Decimator (M = {meta['decimation_factor']}, "
        f"{meta['num_taps']} Taps, {meta['coef_bits']}-bit)\n"
        f"{meta['fs_hz']/1e3:.1f} kSPS -> {meta['fs_out']/1e3:.1f} kSPS "
        f"(Passband = {f_pass_khz:.1f} kHz, Nyquist = {f_nyq_out_khz:.1f} kHz)",
        fontsize=12.5,
        fontweight="bold",
    )

    ax1 = axes[0]
    ax1.plot(freqs_khz, mag_float_db, label="Float64", linewidth=1.5, color="#1f77b4")
    ax1.plot(freqs_khz, mag_quant_db, label=f"Quantized ({meta['coef_bits']}-bit)", linestyle="--", color="#ff7f0e")
    ax1.axvline(f_nyq_out_khz, color="red", linestyle=":", label=f"Output Nyquist ({f_nyq_out_khz:.1f} kHz)")
    ax1.set_xlim(0, meta["f_nyq_in"] / 1e3)
    ax1.set_ylim(-130, 5)
    ax1.set_xlabel("Frequency (kHz)")
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, which="both", alpha=0.4)
    ax1.legend(loc="upper right")

    ax2 = axes[1]
    ax2.plot(freqs_khz, mag_float_db, label="Float64", linewidth=1.5, color="#1f77b4")
    ax2.plot(freqs_khz, mag_quant_db, label=f"Quantized ({meta['coef_bits']}-bit)", linestyle="--", color="#ff7f0e")
    ax2.axvline(f_pass_khz, color="green", linestyle="--", label=f"Passband ({f_pass_khz:.1f} kHz)")
    ax2.axvline(f_nyq_out_khz, color="red", linestyle=":", label=f"Output Nyquist ({f_nyq_out_khz:.1f} kHz)")
    ax2.axvspan(f_nyq_out_khz, 2 * f_nyq_out_khz - f_pass_khz, color="red", alpha=0.1, label="First Alias Zone")
    ax2.set_xlim(0, min(4.0 * f_nyq_out_khz, meta["f_nyq_in"] / 1e3))
    ax2.set_ylim(-120, 5)
    ax2.set_xlabel("Frequency (kHz)")
    ax2.set_ylabel("Magnitude (dB)")
    ax2.grid(True, which="both", alpha=0.4)
    ax2.legend(loc="upper right")

    ax3 = axes[2]
    pass_mask = freqs_khz <= f_pass_khz
    ax3.plot(freqs_khz[pass_mask], mag_float_db[pass_mask], label="Float64", linewidth=1.5, color="#1f77b4")
    ax3.plot(freqs_khz[pass_mask], mag_quant_db[pass_mask], label=f"Quantized ({meta['coef_bits']}-bit)", linestyle="--", color="#ff7f0e")
    max_ripple = np.max(np.abs(mag_quant_db[pass_mask]))
    ax3.set_title(f"In-Band Passband Ripple (Peak Deviation: +/- {max_ripple:.4f} dB)")
    ax3.set_xlim(0, f_pass_khz)
    ax3.set_ylim(-max(0.05, max_ripple * 2.0), max(0.05, max_ripple * 2.0))
    ax3.set_xlabel("Frequency (kHz)")
    ax3.set_ylabel("Magnitude (dB)")
    ax3.grid(True, which="both", alpha=0.4)
    ax3.legend(loc="upper right")

    plt.show()


if __name__ == "__main__":
    # =========================================================================
    # HARDCODED CONFIGURATION PARAMETERS (Single-Stage 384k -> 48k Audio LPF)
    # =========================================================================
    FS_HZ             = 384e3   # Input sample rate in Hz (from phase differentiator)
    DECIMATION_FACTOR = 8       # Decimation factor M (384 kSPS / 8 = 48 kSPS)
    TAPS_PER_PHASE    = 24      # Taps per polyphase branch (Total taps = M * 24 - 1 = 191)
    COEF_BITS         = 18      # Coefficient quantization bit width (18-bit for DSP48E1)
    PASSBAND_RATIO    = 0.625   # Usable passband fraction of output Nyquist (e.g. 0.625 * 24 kHz = 15 kHz)
    STOPBAND_ATTEN_DB = 90.0    # Target Kaiser window stopband attenuation in dB
    OUTPUT_FILE       = Path(f"fir_audio_decim_{DECIMATION_FACTOR}x_{COEF_BITS}b.coe")
    # =========================================================================

    h_flt, h_q_int, h_q_norm, info = design_decimation_fir(
        fs_hz=FS_HZ,
        decimation_factor=DECIMATION_FACTOR,
        taps_per_phase=TAPS_PER_PHASE,
        coef_bits=COEF_BITS,
        passband_ratio=PASSBAND_RATIO,
        stopband_atten_db=STOPBAND_ATTEN_DB,
    )

    write_vivado_coe(OUTPUT_FILE, h_q_int, info)
    plot_filter_response(h_flt, h_q_norm, info)