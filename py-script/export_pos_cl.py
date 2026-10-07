#!/usr/bin/env python3
"""Emit the page's `pos_cl` JSON block (H_pos = pos_meas / pos_cmd)."""
import csv, json, sys
import numpy as np
from scipy.signal import csd, welch, coherence, detrend

# TIME BASE -- SysIdSample.t is the STM's 20kHz control tick, 50us per count,
# so it is the instant the sample was actually taken. host_time_s is the Pi's
# read() return and jitters 99-598us against a 200us nominal; the Pi also
# samples non-uniformly by construction. Welch/CSD assume a uniform grid and do
# not fail on a non-uniform one -- they return a plausible spectrum of a
# time-warped signal, with the error growing with frequency. Requires firmware
# at drive@01ae2ce or later.
TICK_S = 50e-6


def uniform_grid(rows, cols):
    """rows -> (t, fs, {col: series}) on a true uniform grid."""
    tt = np.array([float(r['t']) for r in rows]) * TICK_S
    tt = tt - tt[0]
    keep = np.concatenate([[True], np.diff(tt) > 0])   # drop duplicate ticks
    tt = tt[keep]
    fs = 1.0 / np.median(np.diff(tt))
    grid = np.arange(tt[0], tt[-1], 1.0 / fs)
    out = {}
    for c in cols:
        v = np.array([float(r[c]) for r in rows])[keep]
        out[c] = np.interp(grid, tt, v)
    return grid, fs, out


path, out = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else None)
rows = [r for r in csv.DictReader(open(path)) if r['flags'] == '0x0001']
t, fs, _S = uniform_grid(rows, ['sysid_f', 'iq_cmd_mA'])
cmd  = _S['sysid_f']   / 1000.0
meas = _S['iq_cmd_mA'] / 1000.0
keep = t >= 2.0
t, cmd, meas = t[keep], cmd[keep], meas[keep]
nperseg = min(len(t), max(256, int(fs * 2.0)))
c, m = detrend(cmd), detrend(meas)
f, Scm = csd(c, m, fs=fs, nperseg=nperseg)
f, Scc = welch(c,   fs=fs, nperseg=nperseg)
f, Cxy = coherence(c, m, fs=fs, nperseg=nperseg)
H   = Scm / (Scc + 1e-30)
mag = 20 * np.log10(np.abs(H))
pha = np.degrees(np.unwrap(np.angle(H)))
band   = (f >= 0.3) & (f <= 150.0) & np.isfinite(H)
good, strong = band & (Cxy >= 0.5), band & (Cxy >= 0.8)

def thin(mask, n):
    i = np.where(mask)[0]
    return i[np.linspace(0, len(i) - 1, n).astype(int)] if len(i) > n else i
si, wi = thin(strong, 400), thin(good & ~strong, 400)
rnd = lambda a, n=2: [round(float(v), n) for v in a]
blk = {"f_strong": rnd(f[si]), "mag_strong": rnd(mag[si]), "phase_strong": rnd(pha[si]),
       "f_weak":   rnd(f[wi]), "mag_weak":   rnd(mag[wi]), "phase_weak":   rnd(pha[wi])}
print(json.dumps(blk), file=open(out, 'w') if out else sys.stdout)
print(f"fs={fs:.1f}  strong {strong.sum()} to {f[strong].max():.1f} Hz  DC {np.mean(mag[band & (f<2)]):.2f} dB",
      file=sys.stderr)
