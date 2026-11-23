#!/usr/bin/env python3
"""
Two-Stage SDR Filter Designer & .coe Generator for Vivado FIR Compiler
  - Stage 1 (FIR1_I & FIR1_Q): 122.88 MSPS -> 3.84 MSPS (M1 = 32)
    Complex Bandpass FIR (I and Q .coe files)
  - Stage 2 (FIR2 Shared I/Q): 3.84 MSPS -> 360 kSPS (Rational L2 = 3, M2 = 32)
    Shared Real Low-Pass Polyphase Resampler (.coe file)
"""

import argparse
from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def design_fir1_complex_bandpass(
    fs_in_hz: float = 122.88e6,
    m1_decim: int = 32,
    rf_freq_hz: float = 99.84e6,    # 99.84 MHz aliases to 23.04 MHz (lands at 0 Hz after /32!)
    bw1_hz: float = 360e3,          # Two-sided passband width (+/- 180 kHz) feeding FIR2
    num_taps: int = 512,            # 16 taps per polyphase branch at M1=32
    atten_db: float = 96.0,         # 96 dB stopband attenuation
    coef_bits: int = 18,            # 18-bit matches Zynq DSP48E1 multiplier
):
    """
    Designs Stage 1 Complex Bandpass FIR taps (FIR1_I and FIR1_Q).
    Normalized so each individual core (FIR1_I and FIR1_Q) shows 0.00 dB passband
    gain in Vivado while preserving exact I/Q amplitude balance.
    """
    fs_out1_hz = fs_in_hz / m1_decim

    # Map target RF frequency into 1st ADC Nyquist Zone [0, 61.44 MHz]
    nearest_mult = round(rf_freq_hz / fs_in_hz)
    f_nyq1_hz = abs(rf_freq_hz - nearest_mult * fs_in_hz)
    adc_zone = int(rf_freq_hz // (0.5 * fs_in_hz)) + 1

    # Flip Q sign for even ADC Nyquist zones (88-108 MHz FM is Zone 2) so spectrum stays upright
    q_sign = 1.0 if (adc_zone % 2 == 0) else -1.0

    # Calculate residual baseband offset after decimating by M1 = 32
    f_res_hz = ((f_nyq1_hz + 0.5 * fs_out1_hz) % fs_out1_hz) - 0.5 * fs_out1_hz

    n = np.arange(num_taps, dtype=np.float64)
    mid = (num_taps - 1) / 2.0
    t = n - mid

    # 1. Prototype Low-Pass Filter with single-sided cutoff = bw1_hz / 2
    beta = signal.kaiser_beta(atten_db)
    fc_lp_norm = (bw1_hz / 2.0) / fs_in_hz
    h_lp = 2.0 * fc_lp_norm * np.sinc(2.0 * fc_lp_norm * t)
    h_lp *= signal.windows.kaiser(num_taps, beta=beta)
    h_lp /= np.sum(h_lp)

    # 2. Modulate prototype low-pass to f_nyq1_hz
    phase = 2.0 * np.pi * (f_nyq1_hz / fs_in_hz) * t
    h_i_flt = h_lp * np.cos(phase)
    h_q_flt = q_sign * h_lp * np.sin(phase)

    # 3. Quantize to B-bit signed integer grid using a SINGLE SHARED peak scalar
    max_int = (1 << (coef_bits - 1)) - 1  # 131071 for 18-bit
    shared_peak = max(np.max(np.abs(h_i_flt)), np.max(np.abs(h_q_flt)))
    h_i_int = np.clip(np.round((h_i_flt / shared_peak) * max_int), -(max_int + 1), max_int)
    h_q_int = np.clip(np.round((h_q_flt / shared_peak) * max_int), -(max_int + 1), max_int)

    # 4. Shared coherent gain normalization (0.5 * complex coherent sum = 0 dB per branch)
    complex_taps = h_i_int + 1j * (q_sign * h_q_int)
    shared_branch_gain = 0.5 * np.abs(np.sum(complex_taps * np.exp(-1j * phase)))

    h_i_norm = h_i_int / shared_branch_gain
    h_q_norm = h_q_int / shared_branch_gain

    meta1 = {
        "fs_in_hz": fs_in_hz,
        "fs_out1_hz": fs_out1_hz,
        "m1_decim": m1_decim,
        "rf_freq_hz": rf_freq_hz,
        "f_nyq1_hz": f_nyq1_hz,
        "f_res_hz": f_res_hz,
        "bw1_hz": bw1_hz,
        "num_taps": num_taps,
        "atten_db": atten_db,
        "coef_bits": coef_bits,
        "q_sign": q_sign,
    }
    return h_i_norm, h_q_norm, h_i_int, h_q_int, meta1


def design_fir2_rational_lowpass(
    fs_in2_hz: float = 3.84e6,
    l2_interp: int = 3,
    m2_decim: int = 32,
    passband_bw_hz: float = 175e3,  # 175 kHz WBFM two-sided channel (+/- 87.5 kHz)
    num_taps: int = 384,            # Multiple of M2=32 and L2=3 (12 taps per phase)
    atten_db: float = 96.0,
    coef_bits: int = 18,
):
    """
    Designs Stage 2 Shared Real Low-Pass FIR for 3/32 Rational Resampling
    (3.84 MSPS -> 360 kSPS, Nyquist = +/- 180 kHz).
    In a rational L/M filter, the prototype filter runs at the virtual upsampled
    rate fs_virt = fs_in2 * L2 = 11.52 MSPS.
    """
    fs_virt_hz = fs_in2_hz * l2_interp
    fs_out2_hz = fs_virt_hz / m2_decim       # 360 kSPS
    f_nyq_out2_hz = fs_out2_hz / 2.0         # 180 kHz single-sided (+/- 180 kHz complex)

    # Single-sided low-pass cutoff (e.g., 175 kHz / 2 = 87.5 kHz, or up to ~150 kHz)
    f_cutoff_hz = passband_bw_hz / 2.0

    n = np.arange(num_taps, dtype=np.float64)
    mid = (num_taps - 1) / 2.0
    t = n - mid

    beta = signal.kaiser_beta(atten_db)
    fc_norm = f_cutoff_hz / fs_virt_hz
    h2_flt = 2.0 * fc_norm * np.sinc(2.0 * fc_norm * t)
    h2_flt *= signal.windows.kaiser(num_taps, beta=beta)

    # Quantize to B-bit signed integer grid
    max_int = (1 << (coef_bits - 1)) - 1
    peak_val = np.max(np.abs(h2_flt))
    h2_int = np.clip(np.round((h2_flt / peak_val) * max_int), -(max_int + 1), max_int)

    # Normalize for 0 dB DC passband gain:
    # Note: Sum of taps = 1.0 gives 0 dB in standard freqz; for an L-interpolator
    # in hardware, multiplying by L2 compensates for the (L2-1) zero-stuffing loss.
    h2_norm = h2_int / np.sum(h2_int)

    meta2 = {
        "fs_in2_hz": fs_in2_hz,
        "fs_virt_hz": fs_virt_hz,
        "fs_out2_hz": fs_out2_hz,
        "f_nyq_out2_hz": f_nyq_out2_hz,
        "l2_interp": l2_interp,
        "m2_decim": m2_decim,
        "f_cutoff_hz": f_cutoff_hz,
        "passband_bw_hz": passband_bw_hz,
        "num_taps": num_taps,
        "atten_db": atten_db,
        "coef_bits": coef_bits,
    }
    return h2_norm, h2_int, meta2


def write_float_coe(filepath: Path, taps_norm: np.ndarray, header_comment: str):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"; {header_comment}\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")
        strs = [f"{c:0.24f}" for c in taps_norm]
        lines = [", ".join(strs[i : i + 4]) for i in range(0, len(strs), 4)]
        f.write(",\n".join(lines) + ";\n")
    print(f"[INFO] Wrote {filepath.name} ({len(taps_norm)} taps)")


def plot_all_filter_responses(
    hi_norm: np.ndarray,
    hq_norm: np.ndarray,
    meta1: dict,
    h2_norm: np.ndarray,
    meta2: dict,
):
    to_db = lambda x: 20.0 * np.log10(np.maximum(np.abs(x), 1e-12))

    fig, axes = plt.subplots(3, 1, figsize=(12, 11), constrained_layout=True)
    fig.suptitle(
        f"Two-Stage SDR Filter Cascade:\n"
        f"Stage 1 Complex Bandpass (122.88 MSPS -> 3.84 MSPS, RF={meta1['rf_freq_hz']/1e6:.2f} MHz, {meta1['num_taps']} Taps) + "
        f"Stage 2 Rational {meta2['l2_interp']}/{meta2['m2_decim']} FIR (3.84 MSPS -> {meta2['fs_out2_hz']/1e3:.0f} kSPS, {meta2['num_taps']} Taps)",
        fontsize=12,
        fontweight="bold",
    )

    # --- Plot 1: FIR1_I and FIR1_Q Individual Real Frequency Responses [0, 61.44 MHz] ---
    n_fft1 = 65536
    f1_hz, H1_i = signal.freqz(hi_norm, worN=n_fft1, fs=meta1["fs_in_hz"])
    _, H1_q = signal.freqz(hq_norm, worN=n_fft1, fs=meta1["fs_in_hz"])
    f1_mhz = f1_hz / 1e6

    ax1 = axes[0]
    ax1.plot(f1_mhz, to_db(H1_i), label="FIR1_I (In-Phase Real Response)", color="#1f77b4", linewidth=1.3, alpha=0.85)
    ax1.plot(f1_mhz, to_db(H1_q), label="FIR1_Q (Quadrature Real Response)", color="#ff7f0e", linestyle="--", linewidth=1.3, alpha=0.85)
    ax1.axvline(meta1["f_nyq1_hz"] / 1e6, color="black", linestyle=":", alpha=0.7, label=f"Aliased RF Center ({meta1['f_nyq1_hz']/1e6:.2f} MHz)")
    ax1.set_title("1. Stage 1 Individual Real Filters (FIR1_I & FIR1_Q) Across 1st ADC Nyquist Zone [0 to 61.44 MHz]")
    ax1.set_xlabel("Frequency (MHz)")
    ax1.set_ylabel("Magnitude (dB)")
    ax1.set_xlim(0, meta1["fs_in_hz"] / 2e6)
    ax1.set_ylim(-115, 5)
    ax1.grid(True, which="both", alpha=0.4)
    ax1.legend(loc="upper right")

    # --- Plot 2: Analytic Complex Bandpass Response H1_complex(f) = H(FIR1_I + j*FIR1_Q) [-61.44 to +61.44 MHz] ---
    # Shows one-sided passband (+f_c) and -96 dB image rejection at (-f_c)
    h1_complex = hi_norm + 1j * (meta1["q_sign"] * hq_norm)
    f1_twosided_hz, H1_cplx = signal.freqz(h1_complex, worN=n_fft1, whole=True, fs=meta1["fs_in_hz"])
    # Shift FFT so 0 Hz is in the center [-fs/2, +fs/2)
    f1_twosided_mhz = np.fft.fftshift(f1_twosided_hz - (meta1["fs_in_hz"] * (f1_twosided_hz >= meta1["fs_in_hz"] / 2.0))) / 1e6
    H1_cplx_shifted_db = to_db(np.fft.fftshift(H1_cplx)) - 6.0206  # -6 dB aligns complex sum (2.0) to 0 dB

    ax2 = axes[1]
    ax2.plot(f1_twosided_mhz, H1_cplx_shifted_db, label="Combined Complex Response (I + jQ)", color="#2ca02c", linewidth=1.5)
    ax2.axvline(meta1["f_nyq1_hz"] / 1e6, color="black", linestyle="--", alpha=0.6, label=f"Selected Passband (+{meta1['f_nyq1_hz']/1e6:.2f} MHz)")
    ax2.axvline(-meta1["f_nyq1_hz"] / 1e6, color="#d62728", linestyle=":", alpha=0.7, label=f"Rejected Image (-{meta1['f_nyq1_hz']/1e6:.2f} MHz)")
    ax2.set_title("2. Stage 1 Two-Sided Complex Bandpass Response (Showing >95 dB Negative-Frequency Image Rejection)")
    ax2.set_xlabel("Frequency (MHz)")
    ax2.set_ylabel("Magnitude (dB)")
    ax2.set_xlim(-meta1["fs_in_hz"] / 2e6, meta1["fs_in_hz"] / 2e6)
    ax2.set_ylim(-115, 5)
    ax2.grid(True, which="both", alpha=0.4)
    ax2.legend(loc="upper right")

    # --- Plot 3: Stage 2 (FIR2) Shared Real Low-Pass Response (3/32 Rational Resampler -> 360 kSPS) ---
    n_fft2 = 65536
    f2_hz, H2 = signal.freqz(h2_norm, worN=n_fft2, fs=meta2["fs_virt_hz"])
    f2_khz = f2_hz / 1e3
    H2_db = to_db(H2)

    ax3 = axes[2]
    ax3.plot(f2_khz, H2_db, label=f"FIR2 Shared Low-Pass ({meta2['num_taps']} Taps, L={meta2['l2_interp']}/M={meta2['m2_decim']})", color="#9467bd", linewidth=1.8)
    ax3.axvline(meta2["f_cutoff_hz"] / 1e3, color="black", linestyle="--", alpha=0.7, label=f"WBFM Passband Edge ({meta2['f_cutoff_hz']/1e3:.1f} kHz = +/-87.5 kHz)")
    ax3.axvline(meta2["f_nyq_out2_hz"] / 1e3, color="#d62728", linestyle="-.", alpha=0.8, label=f"Final Output Nyquist ({meta2['f_nyq_out2_hz']/1e3:.1f} kHz @ 360 kSPS)")
    ax3.set_title(
        f"3. Stage 2 (FIR2) Shared Real Low-Pass Filter Response "
        f"(Zoomed 0 to 600 kHz | Output Rate = {meta2['fs_out2_hz']/1e3:.0f} kSPS, Nyquist = {meta2['f_nyq_out2_hz']/1e3:.0f} kHz)"
    )
    ax3.set_xlabel("Baseband Frequency (kHz)")
    ax3.set_ylabel("Magnitude (dB)")
    ax3.set_xlim(0, 600)
    ax3.set_ylim(-115, 5)
    ax3.grid(True, which="both", alpha=0.4)
    ax3.legend(loc="upper right")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate FIR1 (I/Q) & FIR2 (3/32 Rational) .coe files")
    parser.add_argument("--rf", type=float, default=99.84e6, help="Target RF station in Hz (default: 99.84e6)")
    parser.add_argument("--taps1", type=int, default=512, help="FIR1 tap count (default: 512)")
    parser.add_argument("--taps2", type=int, default=384, help="FIR2 tap count (default: 384)")
    parser.add_argument("--bits", type=int, default=18, help="Coefficient quantization bits (default: 18)")
    args = parser.parse_args()

    # 1. Design FIR1 (I and Q Complex Bandpass, M1 = 32)
    hi_norm, hq_norm, _, _, meta1 = design_fir1_complex_bandpass(
        rf_freq_hz=args.rf, num_taps=args.taps1, coef_bits=args.bits
    )

    # 2. Design FIR2 (Shared Real Low-Pass, Rational 3/32 -> 360 kSPS)
    h2_norm, _, meta2 = design_fir2_rational_lowpass(
        num_taps=args.taps2, coef_bits=args.bits
    )

    print(f"[SUMMARY] FIR1: 122.88 MSPS -> 3.84 MSPS | RF={meta1['rf_freq_hz']/1e6:.3f} MHz -> Baseband Offset={meta1['f_res_hz']/1e3:+.1f} kHz")
    print(f"[SUMMARY] FIR2: 3.84 MSPS *(3/32) -> {meta2['fs_out2_hz']/1e3:.1f} kSPS | Nyquist = +/-{meta2['f_nyq_out2_hz']/1e3:.1f} kHz")

    # 3. Export Vivado .coe files
    write_float_coe(
        Path("fir1_I_complex_bp.coe"),
        hi_norm,
        f"FIR1_I Complex Bandpass (RF={meta1['rf_freq_hz']/1e6:.3f} MHz, M=32, 18-bit Normalized Float)",
    )
    write_float_coe(
        Path("fir1_Q_complex_bp.coe"),
        hq_norm,
        f"FIR1_Q Complex Bandpass (RF={meta1['rf_freq_hz']/1e6:.3f} MHz, M=32, 18-bit Normalized Float)",
    )
    write_float_coe(
        Path("fir2_shared_lp_3_32.coe"),
        h2_norm,
        f"FIR2 Shared Real Low-Pass (Rational L=3, M=32 -> 360 kSPS, Cutoff=87.5 kHz, 18-bit Float)",
    )

    # 4. Plot frequency responses
    plot_all_filter_responses(hi_norm, hq_norm, meta1, h2_norm, meta2)