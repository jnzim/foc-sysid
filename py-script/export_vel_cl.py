#!/usr/bin/env python3
"""Emit a `vel_cl_*` JSON block (H = vel_meas/vel_cmd) for the website.

Same estimate as closed_vel_bode_plot.py: Welch/CSD on detrended cmd/meas,
coherence split at 0.8 / 0.5.
"""
import csv, json, sys
import numpy as np
from scipy.signal import csd, welch, coherence, detrend

ENCODER_CPR, VEL_TELEM_DIV = 8192.0, 8.0
path, out = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else None)
rows = [r for r in csv.DictReader(open(path)) if r['flags'] == '0x0001']
t    = np.array([float(r['host_time_s']) for r in rows]); t -= t[0]
cmd  = np.array([float(r['sysid_f'])     for r in rows]) / 1000.0
meas = np.array([float(r['iq_cmd_mA'])   for r in rows]) * VEL_TELEM_DIV * (2*np.pi/ENCODER_CPR)
keep = t >= 2.0                      # drop the settle transient
t, cmd, meas = t[keep], cmd[keep], meas[keep]
fs = 1.0/np.median(np.diff(t))

nperseg = min(len(t), max(256, int(fs*2.0)))
c, m = detrend(cmd), detrend(meas)
f, Scm = csd(c, m, fs=fs, nperseg=nperseg)
f, Scc = welch(c,   fs=fs, nperseg=nperseg)
f, Cxy = coherence(c, m, fs=fs, nperseg=nperseg)
H   = Scm/(Scc + 1e-30)
mag = 20*np.log10(np.abs(H))
pha = np.degrees(np.unwrap(np.angle(H)))
band   = (f >= 0.5) & (f <= 200.0) & np.isfinite(H)
good, strong = band & (Cxy >= 0.5), band & (Cxy >= 0.8)

def thin(mask, n):
    i = np.where(mask)[0]
    return i[np.linspace(0, len(i)-1, n).astype(int)] if len(i) > n else i
si, wi = thin(strong, 400), thin(good & ~strong, 400)
rnd = lambda a, n=2: [round(float(v), n) for v in a]
blk = {"f_strong": rnd(f[si]), "mag_strong": rnd(mag[si]), "phase_strong": rnd(pha[si]),
       "f_weak":   rnd(f[wi]), "mag_weak":   rnd(mag[wi]), "phase_weak":   rnd(pha[wi])}
print(json.dumps(blk), file=open(out,'w') if out else sys.stdout)
print(f"fs={fs:.1f}  strong {strong.sum()} bins to {f[strong].max():.1f} Hz  "
      f"weak {(good&~strong).sum()}  DC |H| {np.mean(mag[band & (f<3)]):.2f} dB", file=sys.stderr)
