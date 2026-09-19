#!/usr/bin/env python3
"""Emit the page's `cl_measured` and `cl_loop` JSON blocks from a CL current chirp.

Mirrors closed_current_bode_plot.py: H = iq_meas/iq_cmd by Welch/CSD, coherence
split at 0.8/0.5, single-pole fit for the bandwidth, and L = H/(1-H) backed out
for the open-loop crossover and phase margin.
"""
import csv, json, sys
import numpy as np
from scipy.signal import csd, welch, coherence
from scipy.optimize import curve_fit

path, out = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else None)
rows = [r for r in csv.DictReader(open(path)) if r['flags'] == '0x0001']
t    = np.array([float(r['host_time_s']) for r in rows])
cmd  = np.array([float(r['sysid_f'])    for r in rows]) / 1000.0   # iq_cmd  [A]
meas = np.array([float(r['iq_mA'])      for r in rows]) / 1000.0   # iq_meas [A]
fs   = 1.0 / np.median(np.diff(t))

x, y = cmd - cmd.mean(), meas - meas.mean()
nperseg = 4096
f, Pxy = csd(x, y, fs=fs, nperseg=nperseg)
f, Pxx = welch(x,    fs=fs, nperseg=nperseg)
f, Cxy = coherence(x, y, fs=fs, nperseg=nperseg)

H    = Pxy / Pxx
band = (f >= 0.5) & (f <= min(950.0, fs/2*0.95)) & np.isfinite(H)
good, strong = band & (Cxy >= 0.5), band & (Cxy >= 0.8)
mag  = 20*np.log10(np.abs(H))
pha  = np.degrees(np.unwrap(np.angle(H)))

def cross(fx, m, target):
    d = m - target
    i = np.where(np.diff(np.sign(d)) != 0)[0]
    if not len(i): return None
    i = i[0]
    lx = np.log10(fx[i]) + (target-m[i])*(np.log10(fx[i+1])-np.log10(fx[i]))/(m[i+1]-m[i])
    return float(10**lx)

fg, mg = f[good], mag[good]
dc     = float(np.mean(mg[fg < 5]))
bw_raw = cross(fg, mg, dc - 3.0)

def one_pole(ff, K, tau):
    Hh = K/(1j*2*np.pi*ff*tau + 1.0)
    return np.concatenate([Hh.real, Hh.imag])
Hg   = H[good]
p, _ = curve_fit(one_pole, fg, np.concatenate([Hg.real, Hg.imag]),
                 p0=[float(np.abs(Hg[np.argmin(fg)])), 1/(2*np.pi*250)], maxfev=20000)
K_fit, tau_fit = p[0], abs(p[1])
bw_fit = 1/(2*np.pi*tau_fit)

# open loop backed out of the closed loop
L      = Hg/(1.0 - Hg)
Lmag   = 20*np.log10(np.abs(L))
Lpha   = np.degrees(np.unwrap(np.angle(L)))
gc     = cross(fg, Lmag, 0.0)
pm     = None
if gc:
    pm = 180.0 + float(np.interp(np.log10(gc), np.log10(fg), Lpha))

def thin(mask, n):
    idx = np.where(mask)[0]
    return idx[np.linspace(0, len(idx)-1, n).astype(int)] if len(idx) > n else idx
si, wi = thin(strong, 400), thin(good & ~strong, 400)
li     = np.linspace(0, len(fg)-1, min(400, len(fg))).astype(int)
f_fit  = np.logspace(np.log10(0.3), np.log10(950), 200)
Hf     = K_fit/(1j*2*np.pi*f_fit*tau_fit + 1.0)
rnd    = lambda a, n=2: [round(float(v), n) for v in a]

res = {
 "cl_measured": {
   "dc_gain_db": round(dc,2), "bw_raw_hz": round(bw_raw,1), "bw_fit_hz": round(bw_fit,1),
   "f_strong": rnd(f[si]), "mag_strong": rnd(mag[si]), "phase_strong": rnd(pha[si]),
   "f_weak": rnd(f[wi]), "mag_weak": rnd(mag[wi]), "phase_weak": rnd(pha[wi]),
   "f_fit": rnd(f_fit), "mag_fit": rnd(20*np.log10(np.abs(Hf))),
   "phase_fit": rnd(np.degrees(np.angle(Hf))),
 },
 "cl_loop": {
   "gc": round(gc,1), "pm": round(pm,1), "gm": None,
   "f": rnd(fg[li]), "mag": rnd(Lmag[li]), "phase": rnd(Lpha[li]),
 },
}
print(json.dumps(res), file=open(out,'w') if out else sys.stdout)
print(f"fs={fs:.1f}  dc={dc:.2f} dB  bw_raw={bw_raw:.1f}  bw_fit={bw_fit:.1f}  gc={gc:.1f}  pm={pm:.1f}", file=sys.stderr)
