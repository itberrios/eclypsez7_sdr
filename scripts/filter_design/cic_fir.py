#!/usr/bin/env python3
"""
CIC Decimator Simulator & FIR Droop Compensation Designer for Vivado
Target: Eclypse Z7 + Zmod SDR (fs_in = 122.88 MSPS)
"""

from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def cic_magnitude_response(
    freqs_hz: np.ndarray,
    fs_in_hz: float,
    r_cic: int,
    n_stages: int = 5,
    diff_delay: int = 1,
) -> np.ndarray:
    f_norm = freqs_hz / fs_in_hz
    mag = np.ones_like(f_norm, dtype=np.float64)
    nz = np.abs(np.sin(np.pi * f_norm)) > 1e-12
    num = np.sin(np.pi * r_cic * diff_delay * f_norm[nz])
    den = r_cic * diff_delay * np.sin(np.pi * f_norm[nz])
    mag[nz] = np.abs(num / den) ** n_stages
    return mag


def design_cic_comp_fir(
    fs_in_hz: float,
    r_cic: int,
    n_stages: int,
    diff_delay: int,
    m_fir: int,
    num_taps: int,
    passband_ratio: float,
    coef_bits: int,
    kaiser_atten_db: float = 80.0,
):
    if num_taps % 2 == 0:
        num_taps += 1

    fs_cic_out = fs_in_hz / r_cic
    fs_final = fs_cic_out / m_fir
    f_nyq_fir_in = fs_cic_out / 2.0
    f_nyq_final = fs_final / 2.0

    f_pass = f_nyq_final * passband_ratio
    f_stop = f_nyq_final

    n_grid = 16384
    f_grid = np.linspace(0.0, f_nyq_fir_in, n_grid)
    desired_mag = np.zeros_like(f_grid)

    pass_mask = f_grid <= f_pass
    cic_pb = cic_magnitude_response(
        f_grid[pass_mask], fs_in_hz, r_cic, n_stages, diff_delay
    )
    desired_mag[pass_mask] = 1.0 / np.maximum(cic_pb, 1e-6)

    trans_mask = (f_grid > f_pass) & (f_grid < f_stop)
    if np.any(trans_mask):
        edge_gain = desired_mag[pass_mask][-1]
        phase = (f_grid[trans_mask] - f_pass) / (f_stop - f_pass) * np.pi
        desired_mag[trans_mask] = edge_gain * 0.5 * (1.0 + np.cos(phase))

    beta = signal.kaiser_beta(kaiser_atten_db)
    h_float = signal.firwin2(
        num_taps,
        f_grid,
        desired_mag,
        nfreqs=32769,
        window=("kaiser", beta),
        fs=fs_cic_out,
    )
    h_float = h_float / np.sum(h_float)

    max_int = (1 << (coef_bits - 1)) - 1
    peak_abs = np.max(np.abs(h_float))
    h_quant_int = np.round((h_float / peak_abs) * max_int).astype(np.int64)
    h_quant_int = np.clip(h_quant_int, -(1 << (coef_bits - 1)), max_int)

    h_quant_dc_norm = h_quant_int / np.sum(h_quant_int)
    h_quant_peak_norm = h_quant_int / float(max_int)

    meta = {
        "fs_in_hz": fs_in_hz,
        "fs_cic_out": fs_cic_out,
        "fs_final": fs_final,
        "r_cic": r_cic,
        "n_stages": n_stages,
        "diff_delay": diff_delay,
        "m_fir": m_fir,
        "total_decim": r_cic * m_fir,
        "num_taps": num_taps,
        "coef_bits": coef_bits,
        "f_pass": f_pass,
        "f_stop": f_stop,
        "f_nyq_fir_in": f_nyq_fir_in,
        "f_nyq_final": f_nyq_final,
        "cic_bit_growth": int(np.ceil(n_stages * np.log2(r_cic * diff_delay))),
        "dc_sum_int": int(np.sum(h_quant_int)),
    }
    return h_float, h_quant_dc_norm, h_quant_peak_norm, h_quant_int, meta


def write_normalized_float_coe(filepath: Path, coeffs_norm: np.ndarray, meta: dict):
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("; ===========================================================================\n")
        f.write("; Vivado FIR Compiler - CIC Droop Compensation Decimator (.coe)\n")
        f.write(f"; ADC Input Rate (fs_in)    : {meta['fs_in_hz']/1e6:.4f} MSPS\n")
        f.write(f"; CIC Decimation (R_CIC)    : {meta['r_cic']} ({meta['n_stages']} stages, D={meta['diff_delay']})\n")
        f.write(f"; FIR Input Rate (fs_cic)   : {meta['fs_cic_out']/1e6:.4f} MSPS\n")
        f.write(f"; FIR Decimation (M_FIR)    : {meta['m_fir']} (Total Decimation = {meta['total_decim']}x)\n")
        f.write(f"; Final Output Sample Rate  : {meta['fs_final']/1e3:.2f} kSPS\n")
        f.write(f"; Compensated Passband Edge : {meta['f_pass']/1e3:.2f} kHz\n")
        f.write(f"; Total Taps                : {meta['num_taps']} ({meta['coef_bits']}-bit Quantized Float)\n")
        f.write("; ===========================================================================\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")

        coeff_strs = [f"{float(c):.18f}" for c in coeffs_norm]
        lines = [", ".join(coeff_strs[i : i + 4]) for i in range(0, len(coeff_strs), 4)]
        f.write(",\n".join(lines) + ";\n")

    print(f"[INFO] Wrote normalized float .coe file: {filepath.resolve()}")


def plot_cic_and_fir_responses(h_float: np.ndarray, h_quant_dc_norm: np.ndarray, meta: dict):
    m_fir = meta["m_fir"]
    r_cic = meta["r_cic"]
    total_decim = meta["total_decim"]

    pts_per_final_nyq = 16384
    n_fft = pts_per_final_nyq * m_fir

    freqs_hz, H_fir_flt = signal.freqz(h_float, worN=n_fft, fs=meta["fs_cic_out"])
    _, H_fir_qnt = signal.freqz(h_quant_dc_norm, worN=n_fft, fs=meta["fs_cic_out"])

    mag_cic = cic_magnitude_response(
        freqs_hz, meta["fs_in_hz"], r_cic, meta["n_stages"], meta["diff_delay"]
    )
    mag_fir_flt = np.abs(H_fir_flt)
    mag_fir_qnt = np.abs(H_fir_qnt)
    mag_comb_flt = mag_cic * mag_fir_flt
    mag_comb_qnt = mag_cic * mag_fir_qnt

    to_db = lambda x: 20.0 * np.log10(np.maximum(x, 1e-12))
    cic_db, fir_flt_db, fir_qnt_db = to_db(mag_cic), to_db(mag_fir_flt), to_db(mag_fir_qnt)
    comb_flt_db, comb_qnt_db = to_db(mag_comb_flt), to_db(mag_comb_qnt)

    f_base_khz = freqs_hz[:pts_per_final_nyq] / 1e3
    fir_folded_zones_db = []
    fir_alias_power_sum = np.zeros(pts_per_final_nyq, dtype=np.float64)

    for k in range(1, m_fir):
        zone_slice = mag_comb_qnt[k * pts_per_final_nyq : (k + 1) * pts_per_final_nyq]
        folded_mag = zone_slice[::-1] if (k % 2 == 1) else zone_slice
        fir_folded_zones_db.append((k, to_db(folded_mag)))
        fir_alias_power_sum += folded_mag ** 2

    total_alias_power_sum = np.copy(fir_alias_power_sum)
    worst_cic_alias_mag = np.zeros(pts_per_final_nyq, dtype=np.float64)

    for k in range(m_fir, total_decim):
        f_zone_hz = (k * meta["f_nyq_final"]) + freqs_hz[:pts_per_final_nyq]
        cic_zone_mag = cic_magnitude_response(
            f_zone_hz, meta["fs_in_hz"], r_cic, meta["n_stages"], meta["diff_delay"]
        )
        sub_k = k % m_fir
        if (k // m_fir) % 2 == 1:
            sub_k = (m_fir - 1) - sub_k
            fir_zone_mag = mag_fir_qnt[sub_k * pts_per_final_nyq : (sub_k + 1) * pts_per_final_nyq][::-1]
        else:
            fir_zone_mag = mag_fir_qnt[sub_k * pts_per_final_nyq : (sub_k + 1) * pts_per_final_nyq]

        casc_zone_mag = cic_zone_mag * fir_zone_mag
        folded_zone = casc_zone_mag[::-1] if (k % 2 == 1) else casc_zone_mag
        total_alias_power_sum += folded_zone ** 2
        worst_cic_alias_mag = np.maximum(worst_cic_alias_mag, folded_zone)

    total_rms_alias_db = to_db(np.sqrt(total_alias_power_sum))
    worst_cic_alias_db = to_db(worst_cic_alias_mag)

    freqs_khz = freqs_hz / 1e3
    f_pass_khz = meta["f_pass"] / 1e3
    f_nyq_final_khz = meta["f_nyq_final"] / 1e3
    f_nyq_fir_in_khz = meta["f_nyq_fir_in"] / 1e3

    fig, axes = plt.subplots(3, 1, figsize=(11.5, 11), constrained_layout=True)
    fig.suptitle(
        f"CIC (R={r_cic}, N={meta['n_stages']}) + FIR Droop Compensator (M={m_fir}, {meta['num_taps']} Taps, {meta['coef_bits']}-bit)\n"
        f"{meta['fs_in_hz']/1e6:.2f} MSPS -> {meta['fs_cic_out']/1e3:.1f} kSPS -> {meta['fs_final']/1e3:.1f} kSPS",
        fontsize=12.5,
        fontweight="bold",
    )

    ax1 = axes[0]
    ax1.plot(freqs_khz, cic_db, label="CIC Only", color="#d62728", linestyle=":", linewidth=1.5)
    ax1.plot(freqs_khz, fir_qnt_db, label=f"FIR Compensator ({meta['coef_bits']}-bit)", color="#ff7f0e", linestyle="--", linewidth=1.3)
    ax1.plot(freqs_khz, comb_qnt_db, label="Combined Cascade", color="#2ca02c", linewidth=1.9)
    for k, f_db in fir_folded_zones_db:
        ax1.plot(f_base_khz, f_db, label=f"Folded FIR Alias (Zone {k+1})", color="#e377c2", linestyle="-.", linewidth=1.5)
    ax1.plot(f_base_khz, worst_cic_alias_db, label="Worst High-Zone CIC+FIR Alias", color="#7f7f7f", linestyle=":", linewidth=1.4)
    ax1.plot(f_base_khz, total_rms_alias_db, label="Total RMS Folded Aliasing", color="#9467bd", linewidth=1.4, alpha=0.85)
    ax1.axvline(f_pass_khz, color="black", linestyle="--", alpha=0.6, label=f"Passband ({f_pass_khz:.1f} kHz)")
    ax1.axvline(f_nyq_final_khz, color="purple", linestyle="-.", alpha=0.7, label=f"Final Nyquist ({f_nyq_final_khz:.1f} kHz)")
    ax1.set_xlim(0, f_nyq_fir_in_khz)
    ax1.set_ylim(-110, 10)
    ax1.set_xlabel("Frequency (kHz)")
    ax1.set_ylabel("Magnitude (dB)")
    ax1.grid(True, which="both", alpha=0.4)
    ax1.legend(loc="upper right", fontsize=8.5, ncol=2)

    ax2 = axes[1]
    zoom_mask = freqs_khz <= f_nyq_final_khz * 1.15
    ax2.plot(freqs_khz[zoom_mask], cic_db[zoom_mask], label="Uncorrected CIC Droop", color="#d62728", linestyle=":", linewidth=2.0)
    ax2.plot(freqs_khz[zoom_mask], fir_qnt_db[zoom_mask], label="FIR Boost", color="#ff7f0e", linestyle="--", linewidth=1.8)
    ax2.plot(freqs_khz[zoom_mask], comb_qnt_db[zoom_mask], label="Combined (Quantized)", color="#2ca02c", linestyle="-.", linewidth=2.0)
    ax2.axvline(f_pass_khz, color="black", linestyle="--", alpha=0.6)
    ax2.axvline(f_nyq_final_khz, color="purple", linestyle="-.", alpha=0.7)
    ax2.set_xlim(0, f_nyq_final_khz * 1.15)
    ax2.set_ylim(-6.0, 4.5)
    ax2.set_xlabel("Frequency (kHz)")
    ax2.set_ylabel("Magnitude (dB)")
    ax2.grid(True, which="both", alpha=0.4)
    ax2.legend(loc="lower left", fontsize=8.5)

    ax3 = axes[2]
    pb_mask = freqs_khz <= f_pass_khz
    ax3.plot(freqs_khz[pb_mask], comb_flt_db[pb_mask], label="Combined (Float)", color="#1f77b4", linewidth=1.6)
    ax3.plot(freqs_khz[pb_mask], comb_qnt_db[pb_mask], label=f"Combined ({meta['coef_bits']}-bit)", color="#2ca02c", linestyle="--", linewidth=1.6)
    max_ripple_db = np.max(np.abs(comb_qnt_db[pb_mask]))
    ax3.set_title(f"Combined Passband Flatness Ripple (Peak: +/- {max_ripple_db:.4f} dB)")
    ax3.set_xlim(0, f_pass_khz)
    ax3.set_ylim(-max(0.02, max_ripple_db * 1.8), max(0.02, max_ripple_db * 1.8))
    ax3.set_xlabel("Frequency (kHz)")
    ax3.set_ylabel("Error (dB)")
    ax3.grid(True, which="both", alpha=0.4)
    ax3.legend(loc="upper right", fontsize=9)

    plt.show()


if __name__ == "__main__":
    # =========================================================================
    # HARDCODED CONFIGURATION PARAMETERS
    # =========================================================================
    FS_IN_HZ        = 122.88e6  # ADC input sample rate in Hz
    R_CIC           = 160       # Programmable CIC decimation factor (122.88M / 160 = 768 kSPS)
    N_STAGES        = 5         # Number of CIC stages (N)
    DIFF_DELAY      = 1         # CIC differential delay (D = 1 or 2)
    M_FIR           = 2         # CIC-comp FIR decimation factor (768k / 2 = 384 kSPS)
    NUM_TAPS        = 63        # Number of FIR taps (odd Type-I symmetric)
    COEF_BITS       = 18        # Coefficient quantization bit width (18-bit for DSP48E1)
    PASSBAND_RATIO  = 0.8       # Flat passband fraction of final Nyquist (0.65 * 192 kHz ≈ 125 kHz)
    KAISER_ATTEN_DB = 90.0      # Target Kaiser window stopband attenuation in dB
    NORM_MODE       = "dc"      # "dc" (sum = 1.0, 0 dB gain) or "peak" (max = 1.0)
    OUTPUT_FILE     = Path(f"cic_comp_R{R_CIC}_M{M_FIR}_{NUM_TAPS}taps_{COEF_BITS}b.coe")
    # =========================================================================

    h_flt, h_q_dc, h_q_peak, h_q_int, info = design_cic_comp_fir(
        fs_in_hz=FS_IN_HZ,
        r_cic=R_CIC,
        n_stages=N_STAGES,
        diff_delay=DIFF_DELAY,
        m_fir=M_FIR,
        num_taps=NUM_TAPS,
        passband_ratio=PASSBAND_RATIO,
        coef_bits=COEF_BITS,
        kaiser_atten_db=KAISER_ATTEN_DB,
    )

    coe_coeffs = h_q_dc if NORM_MODE == "dc" else h_q_peak
    write_normalized_float_coe(OUTPUT_FILE, coe_coeffs, info)
    plot_cic_and_fir_responses(h_flt, h_q_dc, info)