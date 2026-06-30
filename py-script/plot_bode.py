import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import csd, welch
import sys

df = pd.read_csv(sys.argv[1])
df = df[df['flags'] == '0x0001']
df = df[df['host_time_s'] > 1.5]

t_start = df['host_time_s'].min()
t = df['host_time_s'].values - t_start
vd = df['vd_mV'].values / 1000.0
id_ = df['ia_mA'].values / 1000.0
freq = df['ic_mA'].values

print(f"ia range: {df['ia_mA'].min()} to {df['ia_mA'].max()} mA")
print(f"vd range: {df['vd_mV'].min()} to {df['vd_mV'].max()} mV")
print(f"freq range: {freq.min()} to {freq.max()} Hz")
print(f"samples: {len(df)}")

fs = 1.0 / np.mean(np.diff(t))
print(f"fs: {fs:.1f} Hz")

# Full time-series Welch CSD
nperseg = int(fs * 2)  # 2-second windows
f_csd, Piv = csd(vd, id_, fs=fs, nperseg=nperseg)
f_csd, Pvv = welch(vd, fs=fs, nperseg=nperseg)

H = Piv / (Pvv + 1e-10)
mag = 20 * np.log10(np.abs(H))
phase = np.degrees(np.angle(H))

# Restrict to 1-250 Hz
mask = (f_csd >= 1.0) & (f_csd <= 250.0)
freq_out = f_csd[mask]
mag_out = mag[mask]
phase_out = phase[mask]

R = 1.55
L = 0.00204
f_theory = np.logspace(np.log10(1.0), np.log10(300.0), 500)
omega_theory = 2 * np.pi * f_theory
H_theory = 1.0 / (R + 1j * omega_theory * L)
mag_theory = 20 * np.log10(np.abs(H_theory))
phase_theory = np.degrees(np.angle(H_theory))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
ax1.semilogx(freq_out, mag_out, 'b.-', label='Measured')
ax1.semilogx(f_theory, mag_theory, 'r--', label=f'Theory R={R}Ω L={L*1000:.2f}mH')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title('Current Loop Plant - Bode Plot (id/Vd)')
ax1.legend()
ax1.grid(True, which='both')

ax2.semilogx(freq_out, phase_out, 'b.-', label='Measured')
ax2.semilogx(f_theory, phase_theory, 'r--', label='Theory')
ax2.set_ylabel('Phase (deg)')
ax2.set_xlabel('Frequency (Hz)')
ax2.set_ylim([-100, 10])
ax2.legend()
ax2.grid(True, which='both')

plt.tight_layout()
plt.savefig('bode_plot.png')
print("wrote: bode_plot.png")