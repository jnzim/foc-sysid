#!/usr/bin/env python3
"""Emit the closing_three_loops.html `ol_plant` JSON block from a capture.

Mirrors bode_plot.py: same Welch/CSD estimate of iq/vq, same coherence split
(>=0.8 "strong", >=0.5 "weak"), same first-order fit.
"""
import csv, json, sys
import numpy as np
from scipy.signal import csd, welch, coherence

path = sys.argv[1]
R_NOM, L_NOM_MH = 1.55, 1.02

rows = [r for r in csv.DictReader(open(path)) if r['flags'] == '0x0001']
t  = np.array([float(r['host_time_s']) for r in rows])
vq = np.array([float(r['vq_mV']) for r in rows]) / 1000.0
iq = np.array([float(r['iq_mA']) for r in rows]) / 1000.0
fs = 1.0 / np.median(np.diff(t))

vq_ac = vq - vq.mean()
iq_ac = iq - iq.mean()
nperseg = 4096
f, Piv = csd(vq_ac, iq_ac, fs=fs, nperseg=nperseg)
f, Pvv = welch(vq_ac, fs=fs, nperseg=nperseg)
f, Cxy = coherence(vq_ac, iq_ac, fs=fs, nperseg=nperseg)

H = Piv / Pvv
band = (f >= 1.0) & (f <= min(1000.0, fs / 2 * 0.95)) & np.isfinite(H)
good, strong = band & (Cxy >= 0.5), band & (Cxy >= 0.8)

mag = 20 * np.log10(np.abs(H))
pha = np.degrees(np.unwrap(np.angle(H)))

# first-order fit, identical to bode_plot.py: H = K/(1 + jw*tau), R = 1/K, L = tau*R
from scipy.optimize import curve_fit
def first_order_complex(ff, K, tau):
    Hh = K / (1j * 2.0 * np.pi * ff * tau + 1.0)
    return np.concatenate([Hh.real, Hh.imag])
f_c, H_c = f[good], H[good]
y_c = np.concatenate([H_c.real, H_c.imag])
K0 = float(np.abs(H_c[np.argmin(f_c)]))
popt, _ = curve_fit(first_order_complex, f_c, y_c, p0=[K0, 1.0/(2*np.pi*200.0)], maxfev=20000)
K_fit, tau_fit = popt[0], abs(popt[1])
R = 1.0 / K_fit
L = tau_fit * R
fc = 1.0 / (2.0 * np.pi * tau_fit)

def thin(mask, n):
    idx = np.where(mask)[0]
    if len(idx) > n:
        idx = idx[np.linspace(0, len(idx) - 1, n).astype(int)]
    return idx

si, wi = thin(strong, 400), thin(good & ~strong, 400)
f_fit = np.logspace(0, np.log10(1000), 200)
Hf = 1.0 / (R + 1j * 2 * np.pi * f_fit * L)
Hn = 1.0 / (R_NOM + 1j * 2 * np.pi * f_fit * L_NOM_MH * 1e-3)
rnd = lambda a, n=2: [round(float(x), n) for x in a]

out = {
    "R": round(R, 3), "L_mH": round(L * 1e3, 3), "fc_hz": round(fc, 1),
    "R_nom": R_NOM, "L_nom_mH": L_NOM_MH,
    "f_strong": rnd(f[si]), "mag_strong": rnd(mag[si]), "phase_strong": rnd(pha[si]),
    "f_weak": rnd(f[wi]), "mag_weak": rnd(mag[wi]), "phase_weak": rnd(pha[wi]),
    "f_fit": rnd(f_fit), "mag_fit": rnd(20 * np.log10(np.abs(Hf))),
    "phase_fit": rnd(np.degrees(np.angle(Hf))),
    "mag_nom": rnd(20 * np.log10(np.abs(Hn))),
    "phase_nom": rnd(np.degrees(np.angle(Hn))),
}
print(json.dumps(out), file=open(sys.argv[2], 'w') if len(sys.argv) > 2 else sys.stdout)
print(f"fs={fs:.1f} Hz  bins used: strong {strong.sum()}, good {good.sum()}\n"
      f"R={R:.3f} ohm  L={L*1e3:.3f} mH  fc={fc:.1f} Hz", file=sys.stderr)
