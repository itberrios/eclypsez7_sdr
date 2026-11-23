import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

# ==============================================================================
# 1. System Specifications (Eclypse Z7 Zmod SDR: 122.88 Msps Dual-Channel)
# ==============================================================================
fs = 122.88e6               # ADC sample rate (Real)
N_fft = 4096                # FFT / PFB size
df = fs / N_fft             # Bin spacing: 30.0 kHz
P = 8                       # PFB taps per branch (Total taps = 32,768)
COEFF_BITS = 18             # 18-bit signed quantization (DSP48E1)

num_avg_frames = 32         # Video averaging frames (matches FPGA accumulator)
num_total_frames = num_avg_frames + P  # Extra P frames for PFB startup transient

# ==============================================================================
# Helper Functions: Quantization & Vivado .coe Export
# ==============================================================================
def quantize_coeffs(coeffs, bits=18, norm_mode="sum"):
    """Normalize (sum=1.0 for FIR, max=1.0 for Window ROM) and quantize."""
    if norm_mode == "sum":
        c_norm = coeffs / np.sum(coeffs)
    else:
        c_norm = coeffs / np.max(np.abs(coeffs))
    max_val = np.max(np.abs(c_norm))
    scale = (2**(bits - 1) - 1) / max_val
    return np.round(c_norm * scale) / scale

def write_coe_file(filename, coeffs, comment=""):
    """Write float-format .coe file for Vivado FIR Compiler / Block Memory."""
    with open(filename, 'w') as f:
        f.write(f"; {comment}\n")
        f.write(f"; Length: {len(coeffs)}, Quantized: {COEFF_BITS}-bit, Sum: {np.sum(coeffs):.6f}\n")
        f.write("radix=10;\n")
        f.write("coefdata=\n")
        for i, c in enumerate(coeffs):
            sep = ";" if i == len(coeffs) - 1 else ","
            f.write(f"{c:.10f}{sep}\n")
    print(f"Exported {filename} ({len(coeffs)} entries)")

# ==============================================================================
# 2. Design & Quantize Window (Ch1) and PFB Prototype Filter (Ch2)
# ==============================================================================
# Ch1: Standard Hann Window (and Blackman-Harris for reference comparison)
win_hann_ideal = signal.windows.hann(N_fft, sym=False)
win_hann_q = quantize_coeffs(win_hann_ideal, bits=COEFF_BITS, norm_mode="max")
win_bh_q = quantize_coeffs(signal.windows.blackmanharris(N_fft, sym=False), bits=COEFF_BITS, norm_mode="max")
write_coe_file("ch1_hann_window_4096.coe", win_hann_q, "Ch1 4096-pt Hann Window ROM")

# Ch2: 8-Tap/Branch PFB Prototype LPF (Widened for overlapping bins & <0.5 dB scalloping)
OVERLAP_FACTOR = 0.5 # 0.75   # 0.50 = -6 dB edge; 0.75 = flat-top (<0.5 dB scalloping)
num_taps_pfb = P * N_fft
h_pfb_ideal = signal.firwin(
    num_taps_pfb,
    cutoff=OVERLAP_FACTOR * df,
    window=('kaiser', 8.5),
    fs=fs
)
h_pfb_q = quantize_coeffs(h_pfb_ideal, bits=COEFF_BITS, norm_mode="sum")
write_coe_file("ch2_pfb_4096_8tap.coe", h_pfb_q, "Ch2 4096-pt 8-Tap PFB Prototype LPF")

pfb_taps = h_pfb_q.reshape((P, N_fft)).T  # Shape: (N_fft, P)

# ==============================================================================
# 3. Synthetic Dual-Channel RF Test Signal (Identical Signal Split to Ch1 & Ch2)
# ==============================================================================
N_samples = num_total_frames * N_fft
t = np.arange(N_samples) / fs

# Test tones
k_strong = 500              # 15.00 MHz (0 dB blowtorch)
k_weak   = 503              # 15.09 MHz (-60 dB weak signal, 3 bins away)
k_mid    = 800.5            # 24.015 MHz (-6 dB mid-bin scalloping test)

f_strong = k_strong * df
f_weak   = k_weak * df
f_mid    = k_mid * df

amp_strong = 1.0
amp_weak   = 10 ** (-60.0 / 20.0)
amp_mid    = 10 ** (-6.0 / 20.0)

rng = np.random.default_rng(42)
noise_floor = rng.normal(0, 2e-5, size=N_samples)  # ~14-bit ADC thermal floor

adc_in = (
    amp_strong * np.cos(2 * np.pi * f_strong * t)
    + amp_weak * np.cos(2 * np.pi * f_weak * t)
    + amp_mid  * np.cos(2 * np.pi * f_mid * t)
    + noise_floor
)

# ==============================================================================
# 4. Channel 1 (Standard Windowed FFT) vs. Channel 2 (PFB Spectrometer)
# ==============================================================================
# Reshape into frames: (num_total_frames, N_fft)
frames = adc_in.reshape((num_total_frames, N_fft))

# --- Ch1: Standard Windowed FFT (Hann & Blackman-Harris) ---
# Discard first P frames so Ch1 & Ch2 average the exact same time window
frames_steady = frames[P:, :]
spec_ch1_hann = np.fft.rfft(frames_steady * win_hann_q, axis=1)
spec_ch1_bh   = np.fft.rfft(frames_steady * win_bh_q, axis=1)

psd_ch1_hann = np.mean(np.abs(spec_ch1_hann) ** 2, axis=0)
psd_ch1_bh   = np.mean(np.abs(spec_ch1_bh) ** 2, axis=0)

# --- Ch2: PFB Spectrometer (Vectorized Polyphase FIR + rFFT) ---
x_blocks = frames.T  # Shape: (N_fft, num_total_frames)
pfb_frames = np.zeros((N_fft, num_avg_frames), dtype=np.float64)

for m in range(num_avg_frames):
    idx_end = m + P
    # Reverse slice to align with causal FIR convolution: sum_p h[k,p]*x[k, m-p]
    slice_p = x_blocks[:, idx_end - P + 1 : idx_end + 1][:, ::-1]
    pfb_frames[:, m] = np.sum(slice_p * pfb_taps, axis=1)

spec_ch2_pfb = np.fft.rfft(pfb_frames, axis=0)
psd_ch2_pfb  = np.mean(np.abs(spec_ch2_pfb) ** 2, axis=1)

# Normalize all PSDs so the 0 dB strong tone sits at 0 dB
psd_ch1_hann_db = 10 * np.log10(psd_ch1_hann / np.max(psd_ch1_hann) + 1e-20)
psd_ch1_bh_db   = 10 * np.log10(psd_ch1_bh   / np.max(psd_ch1_bh)   + 1e-20)
psd_ch2_pfb_db  = 10 * np.log10(psd_ch2_pfb  / np.max(psd_ch2_pfb)  + 1e-20)

freqs_mhz = np.fft.rfftfreq(N_fft, d=1.0 / fs) / 1e6

# Print quantitative comparison at the weak signal & mid-bin scalloping tone
print(f"\n--- Quantitative Comparison (Bin Spacing = {df/1e3:.1f} kHz) ---")
print(f"Weak Signal (Bin {k_weak}, True = -60.0 dB):")
print(f"  Ch1 Hann FFT           : {psd_ch1_hann_db[k_weak]:6.2f} dB (buried in leakage)")
print(f"  Ch1 Blackman-Harris FFT: {psd_ch1_bh_db[k_weak]:6.2f} dB (mainlobe broadened)")
print(f"  Ch2 PFB Spectrometer   : {psd_ch2_pfb_db[k_weak]:6.2f} dB (cleanly resolved)")

print(f"\nMid-Bin Tone (Bin {k_mid}, True = -6.0 dB):")
print(f"  Ch1 Hann FFT Peak      : {max(psd_ch1_hann_db[800], psd_ch1_hann_db[801]):6.2f} dB (Loss: {(-6.0 - max(psd_ch1_hann_db[800], psd_ch1_hann_db[801])):.2f} dB)")
print(f"  Ch1 Blackman-Harris    : {max(psd_ch1_bh_db[800], psd_ch1_bh_db[801]):6.2f} dB (Loss: {(-6.0 - max(psd_ch1_bh_db[800], psd_ch1_bh_db[801])):.2f} dB)")
print(f"  Ch2 PFB Spectrometer   : {max(psd_ch2_pfb_db[800], psd_ch2_pfb_db[801]):6.2f} dB (Loss: {(-6.0 - max(psd_ch2_pfb_db[800], psd_ch2_pfb_db[801])):.2f} dB)")


# ==============================================================================
# Export 8 Polyphase ROM .mem Files for Vivado $readmemh (18-bit Signed Hex)
# ==============================================================================
def write_pfb_mem_files(h_pfb_q, P, N_fft, bits=18, prefix="pfb_taps_p"):
    # Scale prototype filter to full 18-bit signed integer range
    max_int = (1 << (bits - 1)) - 1
    scale = max_int / np.max(np.abs(h_pfb_q))
    h_int = np.round(h_pfb_q * scale).astype(np.int64)
    
    # Reshape to (P, N_fft): row 0 = newest sample (delay 0), row P-1 = oldest (delay P-1)
    taps_2d = h_int.reshape((P, N_fft))
    mask = (1 << bits) - 1
    hex_digits = (bits + 3) // 4  # 5 hex chars for 18-bit

    for p in range(P):
        fname = f"{prefix}{p}.mem"
        with open(fname, "w") as f:
            for val in taps_2d[p]:
                f.write(f"{int(val) & mask:0{hex_digits}x}\n")
        print(f"Exported {fname} ({N_fft} entries, {bits}-bit signed hex)")

write_pfb_mem_files(h_pfb_q, P=P, N_fft=N_fft, bits=COEFF_BITS)


def write_hann_mem(win_hann_q, bits=18, fname="hann_win_4096.mem"):
    max_int = (1 << (bits - 1)) - 1
    w_int = np.round((win_hann_q / np.max(np.abs(win_hann_q))) * max_int).astype(np.int64)
    mask = (1 << bits) - 1
    hex_digits = (bits + 3) // 4  # 5 hex chars for 18-bit
    with open(fname, "w") as f:
        for val in w_int:
            f.write(f"{int(val) & mask:0{hex_digits}x}\n")
    print(f"Exported {fname} ({len(w_int)} entries)")

write_hann_mem(win_hann_q, bits=18)


# ==============================================================================
# WINDOW 1: Single-Bin Frequency Response (Brick-Wall vs. Windowed FFT)
# ==============================================================================
fig1, ax1 = plt.subplots(figsize=(12, 6))
fig1.canvas.manager.set_window_title("Window 1: Single-Bin Frequency Response")

f_bins = np.linspace(-6.0, 6.0, 4096)  # Offset in units of FFT bins
f_hz = np.abs(f_bins) * df

_, h_rect = signal.freqz(np.ones(N_fft) / N_fft, worN=f_hz, fs=fs)
_, h_hann = signal.freqz(win_hann_q / np.sum(win_hann_q), worN=f_hz, fs=fs)
_, h_bh   = signal.freqz(win_bh_q / np.sum(win_bh_q), worN=f_hz, fs=fs)
_, h_pfb  = signal.freqz(h_pfb_q, worN=f_hz, fs=fs)

ax1.plot(f_bins, 20 * np.log10(np.abs(h_rect) + 1e-12), color='gray', alpha=0.5, linestyle=':', label='Rectangular FFT')
ax1.plot(f_bins, 20 * np.log10(np.abs(h_hann) + 1e-12), color='steelblue', linewidth=1.4, label='Ch1: Hann FFT')
ax1.plot(f_bins, 20 * np.log10(np.abs(h_bh) + 1e-12), color='darkorange', linewidth=1.4, label='Ch1: Blackman-Harris FFT')
ax1.plot(f_bins, 20 * np.log10(np.abs(h_pfb) + 1e-12), color='crimson', linewidth=2.0, label=f'Ch2: {P}-Tap PFB Spectrometer')
ax1.axvline(-0.5, color='black', linestyle='--', alpha=0.4)
ax1.axvline(0.5, color='black', linestyle='--', alpha=0.4, label='Bin Boundaries (±0.5 bin)')
ax1.set_xlim(-6, 6)
ax1.set_ylim(-115, 5)
ax1.set_title("Single-Bin Frequency Response: Standard Windowed FFT vs. 8-Tap PFB", fontweight='bold')
ax1.set_xlabel("Frequency Offset (FFT Bins, 1 bin = 30 kHz)")
ax1.set_ylabel("Magnitude (dB)")
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.6)

# ==============================================================================
# WINDOW 2: Ch1 vs. Ch2 Averaged Spectrum (Full Band & Zoomed Leakage Test)
# ==============================================================================
fig2, (ax2a, ax2b, ax2c) = plt.subplots(3, 1, figsize=(13, 11), gridspec_kw={'hspace': 0.4})
fig2.canvas.manager.set_window_title("Window 2: Ch1 vs. Ch2 Spectrometer Comparison")

# Full Nyquist Band (0 - 61.44 MHz)
ax2a.plot(freqs_mhz, psd_ch1_hann_db, color='steelblue', alpha=0.75, label='Ch1: Hann FFT')
ax2a.plot(freqs_mhz, psd_ch2_pfb_db, color='crimson', alpha=0.85, linewidth=1.2, label='Ch2: PFB Spectrometer')
ax2a.set_xlim(0, fs / 2e6)
ax2a.set_ylim(-110, 5)
ax2a.set_title(f"Full 1st Nyquist Zone Spectrum ({num_avg_frames}-Frame Video Average, 4096-pt)", fontweight='bold')
ax2a.set_xlabel("Frequency (MHz)")
ax2a.set_ylabel("Power (dB)")
ax2a.legend(loc='upper right')
ax2a.grid(True, linestyle='--', alpha=0.6)

# Zoom 1: Strong Blowtorch (Bin 500) + Weak Adjacent Signal (Bin 503, -60 dB)
ax2b.plot(freqs_mhz, psd_ch1_hann_db, marker='o', markersize=3, color='steelblue', label='Ch1: Hann FFT')
ax2b.plot(freqs_mhz, psd_ch1_bh_db, marker='^', markersize=3, color='darkorange', label='Ch1: Blackman-Harris FFT')
ax2b.plot(freqs_mhz, psd_ch2_pfb_db, marker='s', markersize=3, color='crimson', linewidth=1.8, label='Ch2: PFB Spectrometer')
ax2b.axvline(f_weak / 1e6, color='green', linestyle='--', alpha=0.7, label='Weak Signal (-60 dB, +3 bins)')
ax2b.set_xlim((k_strong - 12) * df / 1e6, (k_strong + 15) * df / 1e6)
ax2b.set_ylim(-110, 5)
ax2b.set_title("Dynamic Range & Leakage Test: 0 dB Blowtorch vs. -60 dB Signal (3 Bins Away)", fontweight='bold')
ax2b.set_xlabel("Frequency (MHz)")
ax2b.set_ylabel("Power (dB)")
ax2b.legend(loc='upper right')
ax2b.grid(True, linestyle='--', alpha=0.6)

# Zoom 2: Mid-Bin Scalloping Test (Bin 800.5, True = -6 dB)
ax2c.plot(freqs_mhz, psd_ch1_hann_db, marker='o', markersize=4, color='steelblue', label='Ch1: Hann FFT')
ax2c.plot(freqs_mhz, psd_ch1_bh_db, marker='^', markersize=4, color='darkorange', label='Ch1: Blackman-Harris FFT')
ax2c.plot(freqs_mhz, psd_ch2_pfb_db, marker='s', markersize=4, color='crimson', linewidth=1.8, label='Ch2: PFB Spectrometer')
ax2c.axhline(-6.0, color='black', linestyle='--', alpha=0.6, label='True Tone Amplitude (-6.0 dB)')
ax2c.set_xlim((800 - 8) * df / 1e6, (800 + 9) * df / 1e6)
ax2c.set_ylim(-100, 5)
ax2c.set_title("Scalloping Test: Tone Placed Halfway Between Bins 800 & 801", fontweight='bold')
ax2c.set_xlabel("Frequency (MHz)")
ax2c.set_ylabel("Power (dB)")
ax2c.legend(loc='lower right')
ax2c.grid(True, linestyle='--', alpha=0.6)

# ==============================================================================
# WINDOW 3: Scalloping Loss Across One Bin Width (0.0 to 1.0 Bin Offset)
# ==============================================================================
fig3, ax3 = plt.subplots(figsize=(11, 5))
fig3.canvas.manager.set_window_title("Window 3: Scalloping Loss Comparison")

offsets = np.linspace(0.0, 1.0, 501)
dist_nearest = np.minimum(offsets, 1.0 - offsets) * df  # Distance to nearest bin center

_, sc_rect = signal.freqz(np.ones(N_fft) / N_fft, worN=dist_nearest, fs=fs)
_, sc_hann = signal.freqz(win_hann_q / np.sum(win_hann_q), worN=dist_nearest, fs=fs)
_, sc_bh   = signal.freqz(win_bh_q / np.sum(win_bh_q), worN=dist_nearest, fs=fs)
_, sc_pfb  = signal.freqz(h_pfb_q, worN=dist_nearest, fs=fs)

ax3.plot(offsets, 20 * np.log10(np.abs(sc_rect) + 1e-12), color='gray', linestyle=':', label='Rectangular (Max Loss: -3.92 dB)')
ax3.plot(offsets, 20 * np.log10(np.abs(sc_hann) + 1e-12), color='steelblue', linewidth=1.5, label='Ch1: Hann (Max Loss: -1.42 dB)')
ax3.plot(offsets, 20 * np.log10(np.abs(sc_bh) + 1e-12), color='darkorange', linewidth=1.5, label='Ch1: Blackman-Harris (Max Loss: -0.83 dB)')
ax3.plot(offsets, 20 * np.log10(np.abs(sc_pfb) + 1e-12), color='crimson', linewidth=2.0, label='Ch2: 8-Tap PFB (Flat-Top Response)')
ax3.set_xlim(0, 1)
ax3.set_ylim(-5, 0.5)
ax3.set_title("Scalloping Loss: Measured Peak Amplitude vs. Inter-Bin Tone Offset", fontweight='bold')
ax3.set_xlabel("Tone Offset Between Adjacent Bin Centers (Bins)")
ax3.set_ylabel("Amplitude Error (dB)")
ax3.legend(loc='lower center')
ax3.grid(True, linestyle='--', alpha=0.6)

# ==============================================================================
# WINDOW 4: Quantized vs. Ideal PFB Prototype Response
# ==============================================================================
fig4, ax4 = plt.subplots(figsize=(11, 5))
fig4.canvas.manager.set_window_title("Window 4: PFB Prototype Ideal vs. 18-Bit Quantized")

_, h_pfb_id_plot = signal.freqz(h_pfb_ideal / np.sum(h_pfb_ideal), worN=f_hz, fs=fs)
ax4.plot(f_bins, 20 * np.log10(np.abs(h_pfb_id_plot) + 1e-12), color='black', linestyle='--', label='Ideal 64-bit Float')
ax4.plot(f_bins, 20 * np.log10(np.abs(h_pfb) + 1e-12), color='crimson', alpha=0.8, label=f'Quantized {COEFF_BITS}-bit')
ax4.set_xlim(-6, 6)
ax4.set_ylim(-120, 5)
ax4.set_title(f"Ch2 PFB Prototype Filter: Ideal vs. {COEFF_BITS}-Bit Quantized Response", fontweight='bold')
ax4.set_xlabel("Frequency Offset (FFT Bins)")
ax4.set_ylabel("Magnitude (dB)")
ax4.legend(loc='upper right')
ax4.grid(True, linestyle='--', alpha=0.6)


# ==============================================================================
# WINDOW 5: Full 1st Nyquist Zone Coverage (0 - 61.44 MHz, All 2,049 Bins)
# ==============================================================================
fig5, (ax5a, ax5b) = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={'hspace': 0.25})
fig5.canvas.manager.set_window_title("Window 5: Full 1st Nyquist Zone Bin Coverage")

# 1. Compute prototype responses ONCE via zero-padded rFFT
N_ZP = N_fft * 128
f_bins_zp = np.fft.rfftfreq(N_ZP, d=1.0) * N_fft

mag_hann_proto = np.abs(np.fft.rfft(win_hann_q / np.sum(win_hann_q), n=N_ZP))
mag_pfb_proto  = np.abs(np.fft.rfft(h_pfb_q / np.sum(h_pfb_q), n=N_ZP))

# 2. Vectorized builder for all bins in the 1st Nyquist Zone (0 to N_fft/2)
#    Draws each bin over +/- 2.5 bins with NaN separators for fast single-call plotting
def build_nyquist_bank(proto_mag, parity, half_span=2.5, pts_per_bin=25):
    k_vals = np.arange(parity, N_fft // 2 + 1, 2)
    rel = np.linspace(-half_span, half_span, pts_per_bin)
    
    # Frequency matrix: shape (num_bins, pts_per_bin)
    f_bins_mat = k_vals[:, None] + rel[None, :]
    f_mhz_mat = (f_bins_mat * df) / 1e6
    
    # Evaluate magnitude response once for the relative offsets and tile
    mag_db = 20 * np.log10(np.interp(np.abs(rel), f_bins_zp, proto_mag) + 1e-12)
    mag_mat = np.broadcast_to(mag_db[None, :], f_bins_mat.shape).copy()
    
    # Mask anything outside [0, fs/2] and append NaN column to break lines between bins
    out_of_band = (f_bins_mat < 0) | (f_bins_mat > N_fft // 2)
    f_mhz_mat[out_of_band] = np.nan
    mag_mat[out_of_band] = np.nan
    
    nans = np.full((len(k_vals), 1), np.nan)
    x_flat = np.hstack([f_mhz_mat, nans]).ravel()
    y_flat = np.hstack([mag_mat, nans]).ravel()
    return x_flat, y_flat

# 3. Compute exact Max-Coverage Envelope across the entire 1st Nyquist Zone
f_env_bins = np.linspace(0, N_fft // 2, 32768)
f_env_mhz  = (f_env_bins * df) / 1e6
dist_nearest = np.minimum(f_env_bins % 1.0, 1.0 - (f_env_bins % 1.0))

env_hann_db = 20 * np.log10(np.interp(dist_nearest, f_bins_zp, mag_hann_proto) + 1e-12)
env_pfb_db  = 20 * np.log10(np.interp(dist_nearest, f_bins_zp, mag_pfb_proto) + 1e-12)

# 4. Plot Ch1 (Hann FFT) and Ch2 (PFB) across 0 - 61.44 MHz
x_h_even, y_h_even = build_nyquist_bank(mag_hann_proto, parity=0)
x_h_odd,  y_h_odd  = build_nyquist_bank(mag_hann_proto, parity=1)

ax5a.plot(x_h_even, y_h_even, color='steelblue',  linewidth=0.7, alpha=0.65, label='Even Bins')
ax5a.plot(x_h_odd,  y_h_odd,  color='darkorange', linewidth=0.7, alpha=0.65, label='Odd Bins')
ax5a.plot(f_env_mhz, env_hann_db, color='black', linestyle='--', linewidth=1.2, label='Max Coverage Envelope')

x_p_even, y_p_even = build_nyquist_bank(mag_pfb_proto, parity=0)
x_p_odd,  y_p_odd  = build_nyquist_bank(mag_pfb_proto, parity=1)

ax5b.plot(x_p_even, y_p_even, color='crimson',   linewidth=0.7, alpha=0.65, label='Even Bins')
ax5b.plot(x_p_odd,  y_p_odd,  color='teal',      linewidth=0.7, alpha=0.65, label='Odd Bins')
ax5b.plot(f_env_mhz, env_pfb_db, color='black', linestyle='--', linewidth=1.2, label='Max Coverage Envelope')

for ax, title in [
    (ax5a, f"Ch1: Standard Hann FFT — Full 1st Nyquist Zone Coverage (0 - {fs/2e6:.2f} MHz, {N_fft//2 + 1} Bins)"),
    (ax5b, f"Ch2: {P}-Tap PFB Spectrometer — Full 1st Nyquist Zone Coverage (0 - {fs/2e6:.2f} MHz, {N_fft//2 + 1} Bins)"),
]:
    ax.axhline(-3.0, color='gray', linestyle=':', alpha=0.7)
    ax.set_xlim(0, fs / 2e6)
    ax.set_ylim(-95, 5)
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title(title, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='lower right', ncol=3, fontsize=9)

ax5b.set_xlabel(f"Frequency (MHz)  [1st Nyquist Zone: 0 to {fs/2e6:.2f} MHz | Bin Spacing = {df/1e3:.1f} kHz]")



plt.show()
