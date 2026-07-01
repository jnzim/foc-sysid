#!/usr/bin/env python3
# bode_plot.py — AKM11E current loop plant Bode plot
#
# Usage:
#   python3 bode_plot.py drive_data/sysid_log.csv
#
# Output:
#   bode_plot.png saved next to the input CSV
#
# Plant parameters confirmed:
#   R = 3.55 ohm  (from DC magnitude -11dB)
#   fc = 229 Hz   (read from Bode plot)
#   L = R / (2*pi*fc) = 2.47 mH
#   H(s) = 1 / (Ls + R)

import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence

# Nominal motor parameters (line-to-line)
R_NOM = 3.3       # ohm  DMM line-to-line
L_NOM = 0.00204   # H    datasheet

# Confirmed from Bode measurement
R_MEAS  = 3.55    # ohm  from DC magnitude
FC_MEAS = 229.0   # Hz   read from plot
L_MEAS  = R_MEAS / (2 * np.pi * FC_MEAS)   # 2.47 mH

# -----------------------------------------------------------------------------
# Load and filter
# -----------------------------------------------------------------------------
csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / 'bode_plot.png'

df = pd.read_csv(csv_path)
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
print(f"\nConfirmed plant parameters:")
print(f"  H(s)  = 1 / (Ls + R)")
print(f"  R     = {R_MEAS:.2f} ohm")
print(f"  L     = {L_MEAS*1000:.2f} mH")
print(f"  fc    = {FC_MEAS:.0f} Hz")

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
# Theory overlays
# -----------------------------------------------------------------------------
f_th = np.logspace(np.log10(1.0), np.log10(1000.0), 500)
w_th = 2 * np.pi * f_th

H_nom    = 1.0 / (R_NOM  + 1j * w_th * L_NOM)
H_conf   = 1.0 / (R_MEAS + 1j * w_th * L_MEAS)
mag_nom  = 20 * np.log10(np.abs(H_nom))
mag_conf = 20 * np.log10(np.abs(H_conf))
phi_nom  = np.degrees(np.angle(H_nom))
phi_conf = np.degrees(np.angle(H_conf))

mag_dc  = 20 * np.log10(1.0 / R_MEAS)
mag_3db = mag_dc - 3.0

# -----------------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------------
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

# Magnitude
ax1.semilogx(f_csd[band], mag[band], 'b.-', label='Measured', alpha=0.7)
ax1.semilogx(f_th, mag_nom, 'r--',
             label=f'Nominal R={R_NOM}Ω L={L_NOM*1000:.2f}mH')
ax1.semilogx(f_th, mag_conf, 'g-',
             label=f'Confirmed R={R_MEAS:.2f}Ω L={L_MEAS*1000:.2f}mH fc={FC_MEAS:.0f}Hz')
ax1.axhline(mag_3db, color='gray', linestyle=':', alpha=0.7,
            label=f'-3dB = {mag_3db:.1f} dB')
ax1.axvline(FC_MEAS, color='purple', linestyle=':', alpha=0.7,
            label=f'fc = {FC_MEAS:.0f} Hz')
ax1.plot(FC_MEAS, mag_3db, 'go', markersize=8)

# Transfer function text box
tf_text = (
    f'$H(s) = \\dfrac{{1}}{{Ls + R}}$\n\n'
    f'$R = {R_MEAS:.2f}\\,\\Omega$\n'
    f'$L = {L_MEAS*1000:.2f}\\,mH$\n'
    f'$f_c = {FC_MEAS:.0f}\\,Hz$'
)
ax1.text(0.98, 0.97, tf_text,
         transform=ax1.transAxes,
         fontsize=11,
         verticalalignment='top',
         horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

ax1.set_ylabel('Magnitude (dB)')
ax1.set_title(
    'AKM11E Current Plant Bode — id/Vd  (theta=0, d-axis aligned)\n'
    'STM32F411 + DRV8353RS-EVM'
)
ax1.legend(fontsize=8, loc='lower left')
ax1.grid(True, which='both')

# Phase
ax2.semilogx(f_csd[coh_good], phi[coh_good], 'b.-',
             label='Measured (coh≥0.5)', alpha=0.7)
ax2.semilogx(f_th, phi_nom,  'r--', label='Nominal theory')
ax2.semilogx(f_th, phi_conf, 'g-',  label='Confirmed')
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
plt.savefig(out_path, dpi=160)
print(f"\nwrote: {out_path}")