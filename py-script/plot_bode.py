#!/usr/bin/env python3
# bode_plot.py — current loop plant Bode plot with curve fit
#
# Usage:
#   python3 bode_plot.py drive_data/sysid_log.csv

import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence
from scipy.optimize import curve_fit

# Nominal motor parameters (line-to-line)
R_NOM = 3.3       # ohm  DMM line-to-line
L_NOM = 0.00204   # H    datasheet

# -----------------------------------------------------------------------------
# Load and filter
# -----------------------------------------------------------------------------
df = pd.read_csv(sys.argv[1])
df = df[df['flags'] == '0x0001']
df = df[(df['host_time_s'] > 0.5) & (df['host_time_s'] < 20.0)]
df = df[df['sysid_f'] > 0]
df = df[df['dt'] > 0]
df = df[df['vd_mV'].abs() < 1500]

t   = df['host_time_s'].values - df['host_time_s'].min()
vd  = df['vd_mV'].values / 1000.0
id_ = df['id_mA'].values / 1000.0
fs  = 1.0 / np.mean(np.diff(t))

print(f"samples : {len(df)}  fs: {fs:.1f} Hz")
print(f"vd      : {vd.min()*1000:.1f} to {vd.max()*1000:.1f} mV")
print(f"id      : {id_.min()*1000:.1f} to {id_.max()*1000:.1f} mA")
print(f"freq    : {df['sysid_f'].min():.1f} to {df['sysid_f'].max():.1f} Hz")
print(f"corr    : {np.corrcoef(vd, id_)[0,1]:.4f}")

# -----------------------------------------------------------------------------
# Welch CSD
# -----------------------------------------------------------------------------
nperseg = int(fs * 2)

f_csd, Piv = csd(vd, id_, fs=fs, nperseg=nperseg)
f_csd, Pvv = welch(vd,      fs=fs, nperseg=nperseg)
f_coh, Cxy = coherence(vd, id_, fs=fs, nperseg=nperseg)

H   = Piv / (Pvv + 1e-10)
mag = 20 * np.log10(np.abs(H) + 1e-12)
phi = np.degrees(np.angle(H))

band     = (f_csd >= 1.0) & (f_csd <= 1000.0)
coh_good = band & (Cxy >= 0.5)

# -----------------------------------------------------------------------------
# -3dB extraction
# -----------------------------------------------------------------------------
mag_dc  = np.mean(mag[band][5:20])
mag_3db = mag_dc - 3.0
idx_3db = np.argmin(np.abs(mag[band] - mag_3db))
fc_meas = f_csd[band][idx_3db]
R_meas  = 1.0 / (10 ** (mag_dc / 20))
L_meas  = R_meas / (2 * np.pi * fc_meas)

print(f"\n-3dB extraction:")
print(f"  R_meas  = {R_meas:.3f} ohm")
print(f"  fc_meas = {fc_meas:.1f} Hz")
print(f"  L_meas  = {L_meas*1000:.3f} mH")

# -----------------------------------------------------------------------------
# Curve fit to RL magnitude model
# -----------------------------------------------------------------------------
def rl_magnitude(f, R, L):
    w = 2 * np.pi * f
    return 20 * np.log10(1.0 / np.sqrt(R**2 + (w * L)**2))

f_fit   = f_csd[coh_good]
mag_fit = mag[coh_good]

try:
    popt, pcov = curve_fit(rl_magnitude, f_fit, mag_fit, p0=[R_NOM, L_NOM])
    R_fit, L_fit = popt
    perr = np.sqrt(np.diag(pcov))
    fc_fit = R_fit / (2 * np.pi * L_fit)
    print(f"\nCurve fit (coherence >= 0.5):")
    print(f"  R_fit   = {R_fit:.3f} ± {perr[0]:.3f} ohm")
    print(f"  L_fit   = {L_fit*1000:.3f} ± {perr[1]*1000:.3f} mH")
    print(f"  fc_fit  = {fc_fit:.1f} Hz")
    fit_ok = True
except Exception as e:
    print(f"\nCurve fit failed: {e}")
    fit_ok = False

# -----------------------------------------------------------------------------
# Theory and fit overlays
# -----------------------------------------------------------------------------
f_th = np.logspace(np.log10(1.0), np.log10(1000.0), 500)
w_th = 2 * np.pi * f_th

H_nom = 1.0 / (R_NOM + 1j * w_th * L_NOM)
mag_nom = 20 * np.log10(np.abs(H_nom))
phi_nom = np.degrees(np.angle(H_nom))

if fit_ok:
    H_fit_th  = 1.0 / (R_fit + 1j * w_th * L_fit)
    mag_fit_th = 20 * np.log10(np.abs(H_fit_th))
    phi_fit_th = np.degrees(np.angle(H_fit_th))

# -----------------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------------
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

# Magnitude
ax1.semilogx(f_csd[band], mag[band], 'b.-', label='Measured', alpha=0.7)
ax1.semilogx(f_th, mag_nom, 'r--',
             label=f'Nominal R={R_NOM}Ω L={L_NOM*1000:.2f}mH')
if fit_ok:
    ax1.semilogx(f_th, mag_fit_th, 'g-',
                 label=f'Fit R={R_fit:.2f}Ω L={L_fit*1000:.2f}mH fc={fc_fit:.0f}Hz')
ax1.axhline(mag_3db, color='gray', linestyle=':', alpha=0.6,
            label=f'-3dB = {mag_3db:.1f} dB')
ax1.axvline(fc_meas, color='purple', linestyle=':', alpha=0.6,
            label=f'fc = {fc_meas:.0f} Hz')
ax1.plot(fc_meas, mag_3db, 'go', markersize=8)
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title('Current Plant Bode — id/Vd  (theta=0, d-axis aligned)')
ax1.legend(fontsize=8)
ax1.grid(True, which='both')

# Phase
ax2.semilogx(f_csd[coh_good], phi[coh_good], 'b.-',
             label='Measured (coh≥0.5)', alpha=0.7)
ax2.semilogx(f_th, phi_nom, 'r--', label='Nominal theory')
if fit_ok:
    ax2.semilogx(f_th, phi_fit_th, 'g-', label='Fit')
ax2.set_ylabel('Phase (deg)')
ax2.set_ylim([-100, 10])
ax2.legend(fontsize=8)
ax2.grid(True, which='both')

# Coherence
ax3.semilogx(f_coh[band], Cxy[band], 'g.-', label='Coherence')
ax3.axhline(0.5, color='r',      linestyle='--', label='0.5 threshold')
ax3.axhline(0.8, color='orange', linestyle='--', label='0.8 threshold')
ax3.set_ylabel('Coherence')
ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylim([0, 1.1])
ax3.legend(fontsize=8)
ax3.grid(True, which='both')

plt.tight_layout()
plt.savefig('bode_plot.png', dpi=160)
print("\nwrote: bode_plot.png")