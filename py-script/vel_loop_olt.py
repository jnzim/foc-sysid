#!/usr/bin/env python3
"""Velocity loop OPEN-LOOP TF, backed out exactly from measured CLOSED-loop
H(s) = vel_meas/vel_cmd via L = H/(1-H) -- no assumption about C(s) needed,
unlike the P(s)=vel/iq backout in closed_vel_bode_plot.py (which assumes a
known linear C(s) and breaks under real saturation). This is the actual
open-loop TF of the velocity loop as currently deployed (VEL_KP/VEL_KI),
straight from the same measured, high-coherence H(s) data.
"""
import sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import csd, welch, coherence, detrend

ENCODER_CPR = 8192.0
VEL_TELEM_DIV = 8.0  # firmware sends vel_meas_counts / VEL_TELEM_DIV as int16
FLAG_RUN = 1
SETTLE_TIME_S = 4.0

csv_path = Path(sys.argv[1]).resolve()
df = pd.read_csv(csv_path)

def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))

flags_numeric = parse_flags(df["flags"])
df = df[(flags_numeric == FLAG_RUN) & (df["dt"] > 0)].copy()
df = df[df["sysid_f"] != 0].copy()

run_start_time = df["host_time_s"].to_numpy(dtype=np.float64)[0]
df = df[df["host_time_s"] >= run_start_time + SETTLE_TIME_S].copy()

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]
fs = 1.0 / np.median(np.diff(t))

vel_cmd  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0
vel_meas = df["iq_cmd_mA"].to_numpy(dtype=np.float64) * VEL_TELEM_DIV * (2.0 * np.pi / ENCODER_CPR)

cmd_ac  = detrend(vel_cmd,  type="linear")
meas_ac = detrend(vel_meas, type="linear")

nperseg = min(len(df), max(256, int(fs * 2.0)))
f_csd, S_cm = csd(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)
_,     S_cc = welch(cmd_ac,        fs=fs, nperseg=nperseg)
f_coh, Cxy  = coherence(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)

H = S_cm / (S_cc + 1e-30)

band = (f_csd >= 0.2) & (f_csd <= fs * 0.45)
good = band & (Cxy >= 0.5)
strong = band & (Cxy >= 0.8)

f_good = f_csd[good]
H_good = H[good]
coh_good = Cxy[good]

# L(s) = H/(1-H) -- exact identity, no C(s) assumption
L = H_good / (1.0 - H_good + 1e-30)
mag_db = 20.0 * np.log10(np.abs(L) + 1e-30)
phase_deg = np.degrees(np.unwrap(np.angle(L)))

print(f"samples: {len(df)}  fs: {fs:.1f} Hz  freq bins (coh>=0.5): {len(f_good)}")
print(f"freq range with coh>=0.5: {f_good.min():.2f} - {f_good.max():.2f} Hz")
print(f"freq range with coh>=0.8: {f_csd[strong].min():.2f} - {f_csd[strong].max():.2f} Hz" if np.any(strong) else "no bins with coh>=0.8")
print()
print("Velocity loop OPEN-LOOP TF  L(jw) = H(jw)/(1-H(jw))")
print("f(Hz)    |L|(dB)   phase(deg)  coh")
for i in range(0, len(f_good), max(1, len(f_good)//40)):
    print(f"{f_good[i]:7.2f}  {mag_db[i]:8.2f}  {phase_deg[i]:9.2f}  {coh_good[i]:.2f}")

# gain crossover: first place |L| crosses 0dB going down
sign = np.sign(mag_db - 0.0)
cross_idx = np.where(np.diff(sign) != 0)[0]
print()
if len(cross_idx):
    i = cross_idx[0]
    f1, f2 = f_good[i], f_good[i+1]
    m1, m2 = mag_db[i], mag_db[i+1]
    p1, p2 = phase_deg[i], phase_deg[i+1]
    frac = -m1 / (m2 - m1) if (m2 - m1) != 0 else 0
    gc = f1 + frac * (f2 - f1)
    pm_phase = p1 + frac * (p2 - p1)
    pm = 180.0 + pm_phase
    print(f"Gain crossover (0dB): {gc:.2f} Hz")
    print(f"Phase there          : {pm_phase:.2f} deg")
    print(f"Phase margin         : {pm:.2f} deg")
else:
    print("No 0dB crossing found in coherent range -- |L| stays " +
          ("above 0dB (loop gain never drops enough)" if mag_db[0] > 0 else "below 0dB throughout"))

# phase crossover: first place phase crosses -180
sign2 = np.sign(phase_deg + 180.0)
cross_idx2 = np.where(np.diff(sign2) != 0)[0]
if len(cross_idx2):
    i = cross_idx2[0]
    f1, f2 = f_good[i], f_good[i+1]
    p1, p2 = phase_deg[i], phase_deg[i+1]
    m1, m2 = mag_db[i], mag_db[i+1]
    frac = (-180.0 - p1) / (p2 - p1) if (p2 - p1) != 0 else 0
    pc = f1 + frac * (f2 - f1)
    gm_mag = m1 + frac * (m2 - m1)
    print(f"Phase crossover (-180deg): {pc:.2f} Hz")
    print(f"Gain margin              : {-gm_mag:.2f} dB")
else:
    print("Phase never reaches -180 deg in coherent range -- gain margin effectively infinite here")
