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

# TIME BASE -- SysIdSample.t is the STM's 20kHz control tick, 50us per count,
# so it is the instant the sample was actually taken. host_time_s is when the
# Pi's read() returned and jitters 99-598us against a 200us nominal. The Pi
# also samples non-uniformly by construction (gaps alternate 2/3 ticks), and
# Welch/CSD assume a uniform grid -- given a non-uniform one they do not fail,
# they return a plausible spectrum of a time-warped signal, with the error
# growing with frequency. So: real timestamps, then resample onto a uniform
# grid. Requires firmware at drive@01ae2ce or later, which stamps the control
# tick into SysIdSample.t; before that the field counted publishes.
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
t, fs, _S = uniform_grid(rows, ['sysid_f', 'iq_mA'])
cmd  = _S['sysid_f'] / 1000.0   # iq_cmd  [A]
meas = _S['iq_mA']   / 1000.0   # iq_meas [A]

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
Hg     = H[good]
dc     = float(np.mean(mg[fg < 5]))
bw_raw = cross(fg, mg, dc - 3.0)

# One pole plus a transport delay, DC gain FIXED at 1.
#
# K is not a free parameter: the controller has an integrator, so the loop has
# infinite DC gain and H(0) = 1 exactly. Letting the optimizer move it does not
# measure anything, it gives model error somewhere to hide -- and the error it
# was hiding is the delay. A delay adds phase without touching magnitude, which
# a single pole cannot reproduce, so the old fit dragged the pole down to
# manufacture phase and then raised K to put the magnitude back. That is why
# the published curve sat ~1.4dB above the measured data at DC.
#
#   K free, 1 pole        K=1.151  fc=445 Hz              rms 0.1118
#   K=1,    1 pole        K=1.000  fc=497 Hz              rms 0.1307
#   K=1,    1 pole+delay  K=1.000  fc=783 Hz  td=113us    rms 0.0066
#
# 113us is 2.3 control ticks -- ADC conversion, computation, PWM update -- and
# it is what caps this loop, so it is reported rather than absorbed into a gain
# the loop cannot physically have.
def pole_delay(ff, fc, td):
    Hh = np.exp(-2j*np.pi*ff*td)/(1.0 + 1j*ff/fc)
    return np.concatenate([Hh.real, Hh.imag])

p, _ = curve_fit(pole_delay, fg, np.concatenate([Hg.real, Hg.imag]),
                 p0=[250.0, 100e-6], maxfev=20000)
fc_fit, td_fit = abs(p[0]), abs(p[1])
tau_fit = 1.0/(2*np.pi*fc_fit)
K_fit   = 1.0
bw_fit  = fc_fit

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
Hf     = np.exp(-2j*np.pi*f_fit*td_fit)/(1.0 + 1j*f_fit/fc_fit)
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
print(f"fit pole={fc_fit:.1f} Hz  delay={td_fit*1e6:.0f} us\nfs={fs:.1f}  dc={dc:.2f} dB  bw_raw={bw_raw:.1f}  bw_fit={bw_fit:.1f}  gc={gc:.1f}  pm={pm:.1f}", file=sys.stderr)
