#!/usr/bin/env python3
# bode_plot.py — AKM11E current-loop plant Bode + PI loop margin estimate
#
# Usage:
#   python3 bode_plot.py drive_data/sysid_log.csv
#
# Output:
#   bode_plot.png saved next to the input CSV
#
# Plant:
#   P(s) = iq / vq = 1 / (Ls + R)
#
# Loop gain for current controller:
#   Loop(s) = C(s) P(s)
#   C(s) = Kp + Ki/s

import sys
import os
import datetime
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scipy.signal import csd, welch, coherence
from scipy.optimize import curve_fit


# -----------------------------------------------------------------------------
# Motor / plant parameters
# -----------------------------------------------------------------------------

# Nominal motor parameters (datasheet/reference, plotted for comparison only —
# R_MEAS/L_MEAS/FC_MEAS below are fit from this run's own data, not hardcoded)
R_LL = 3.10
L_LL = 0.00204

R_NOM = R_LL / 2.0       # 1.55 ohm
L_NOM = L_LL / 2.0       # 1.02 mH

# Optional plant measurement delay overlay
TD_PLANT = 25e-6   # seconds; try 0, 10e-6, 25e-6, 50e-6


# -----------------------------------------------------------------------------
# Current-loop controller design target
# -----------------------------------------------------------------------------
# PI zero is placed at the fitted plant pole (cancellation), Kp is then solved
# so the resulting pure-integrator loop crosses 0 dB at BW_TARGET_HZ — same
# zero-cancellation + BW-target approach as bode_vel_plot.py.
#
# 500 Hz matches the CURRENT_LOOP_BW_HZ that bode_vel_plot.py *assumes* when
# it derives the velocity loop's own BW target (current/10) — running this
# fit tells you whether that assumption actually holds for this hardware.

BW_TARGET_HZ = 500.0  # V/A and V/(A*s) gains (KP_I, KI_I) are derived below

# Optional digital/control/PWM/ADC delay in loop margin estimate
TD_LOOP = 25e-6  # seconds

# -----------------------------------------------------------------------------
# ACTUAL deployed current-loop gains -- must be kept in sync by hand with
# CURRENT_LOOP_KP/CURRENT_LOOP_KI in Include/config.h (drive repo). The
# margin estimate below (gc, PM, GM) is computed from THESE, not from
# KP_I/KI_I -- KP_I/KI_I are a fresh 500Hz-target design solved from
# whatever plant this run happens to fit, so a margin estimate built from
# them would trivially always show ~BW_TARGET_HZ regardless of what's
# actually flashed. This is the number that answers "does the deployed
# loop actually hit its target," not "would a fresh design hit it."
# -----------------------------------------------------------------------------

DEPLOYED_KP = 2.07     # V/A   -- config.h CURRENT_LOOP_KP
DEPLOYED_KI = 2450.0   # V/(A*s) -- config.h CURRENT_LOOP_KI


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def first_order_complex(f, K, tau):
    w = 2.0 * np.pi * f
    H = K / (1j * w * tau + 1.0)
    return np.concatenate([H.real, H.imag])


def interp_log_x_for_y(x_hz, y, target):
    """
    Find first log-frequency interpolation where y crosses target.
    Returns interpolated frequency or None.
    """
    y_shift = y - target
    idxs = np.where(np.diff(np.sign(y_shift)) != 0)[0]

    if len(idxs) == 0:
        return None, None

    i = idxs[0]

    f1, f2 = x_hz[i], x_hz[i + 1]
    y1, y2 = y[i], y[i + 1]

    x1 = np.log10(f1)
    x2 = np.log10(f2)

    if abs(y2 - y1) < 1e-20:
        x_cross = x1
    else:
        x_cross = x1 + (target - y1) * (x2 - x1) / (y2 - y1)

    f_cross = 10.0 ** x_cross

    return f_cross, i


def interp_log_y_at_f(x_hz, y, f_query):
    """
    Interpolate y at f_query using log-frequency interpolation.
    """
    x = np.log10(x_hz)
    xq = np.log10(f_query)
    return np.interp(xq, x, y)


# -----------------------------------------------------------------------------
# Load and filter
# -----------------------------------------------------------------------------

if len(sys.argv) < 2:
    print("Usage: python3 bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "bode_plot.png"

csv_mtime = datetime.datetime.fromtimestamp(os.path.getmtime(csv_path))
plot_gen_time = datetime.datetime.now()
print(f"data file  : {csv_path.name}  (written {csv_mtime:%Y-%m-%d %H:%M:%S})")
print(f"plot gen'd : {plot_gen_time:%Y-%m-%d %H:%M:%S}")

df = pd.read_csv(csv_path)

df = df[df["flags"] == "0x0001"]
df = df[(df["host_time_s"] > 0.5) & (df["host_time_s"] < 20.0)]
df = df[df["sysid_f"] > 0]
df = df[df["dt"] > 0]
df = df[df["vq_mV"].abs() < 1500]

t  = df["host_time_s"].values - df["host_time_s"].min()
vq = df["vq_mV"].values / 1000.0


iq_ = df["iq_mA"].values / 1000.0

fs = 1.0 / np.mean(np.diff(t))

print(f"samples : {len(df)}  fs: {fs:.1f} Hz")
print(f"vq      : {vq.min()*1000:.1f} to {vq.max()*1000:.1f} mV")
print(f"iq      : {iq_.min()*1000:.1f} to {iq_.max()*1000:.1f} mA")
print(f"freq    : {df['sysid_f'].min():.1f} to {df['sysid_f'].max():.1f} Hz")
print(f"corr    : {np.corrcoef(vq, iq_)[0,1]:.4f}")

# Remove DC bias before spectral estimate
vq_ac = vq - np.mean(vq)
iq_ac = iq_ - np.mean(iq_)


# -----------------------------------------------------------------------------
# Welch / CSD transfer estimate
# -----------------------------------------------------------------------------

nperseg = int(fs * 2.0)

f_csd, Piv = csd(vq_ac, iq_ac, fs=fs, nperseg=nperseg)
f_csd, Pvv = welch(vq_ac,     fs=fs, nperseg=nperseg)
f_coh, Cxy = coherence(vq_ac, iq_ac, fs=fs, nperseg=nperseg)

# Transfer estimate: P = iq / vq
P_meas = Piv / (Pvv + 1e-20)

mag = 20.0 * np.log10(np.abs(P_meas) + 1e-20)
phi = np.degrees(np.unwrap(np.angle(P_meas)))

band = (
    (f_csd >= 1.0) &
    (f_csd <= 1000.0) &
    np.isfinite(mag) &
    np.isfinite(phi)
)

coh_good   = band & (Cxy >= 0.5)
coh_strong = band & (Cxy >= 0.8)


# -----------------------------------------------------------------------------
# Plant identification — first-order fit P(s) = iq/vq = 1/(Ls + R)
#                       = K/(tau*s + 1), K = 1/R, tau = L/R
# -----------------------------------------------------------------------------

f_fit = f_csd[coh_good]
H_fit = P_meas[coh_good]

if len(f_fit) < 5:
    raise RuntimeError("Too few coherent (coh>=0.5) frequency bins to fit the plant")

y_fit = np.concatenate([H_fit.real, H_fit.imag])

K0   = float(np.abs(H_fit[np.argmin(f_fit)]))
tau0 = 1.0 / (2.0 * np.pi * 200.0)

popt, _ = curve_fit(first_order_complex, f_fit, y_fit, p0=[K0, tau0], maxfev=20_000)
K_fit, tau_fit = popt
tau_fit = abs(tau_fit)

R_MEAS  = 1.0 / K_fit
L_MEAS  = tau_fit * R_MEAS
FC_MEAS = 1.0 / (2.0 * np.pi * tau_fit)

print("\nFitted plant parameters (this run's data, coh>=0.5 bins):")
print("  P(s)  = 1 / (Ls + R)")
print(f"  R     = {R_MEAS:.2f} ohm")
print(f"  L     = {L_MEAS*1000:.2f} mH")
print(f"  fc    = {FC_MEAS:.0f} Hz")

# -----------------------------------------------------------------------------
# Current-loop PI design — zero at fitted plant pole, gain set for BW_TARGET_HZ
# -----------------------------------------------------------------------------

wc_target = 2.0 * np.pi * BW_TARGET_HZ
TI_I      = tau_fit
KP_I      = wc_target * TI_I / K_fit
KI_I      = KP_I / TI_I

print("\nDesigned current-loop PI (zero-cancellation @ fitted pole):")
print(f"  BW target = {BW_TARGET_HZ:.0f} Hz")
print(f"  Ti        = tau_fit = {TI_I*1000:.3f} ms")
print(f"  Kp        = {KP_I:.4g} V/A")
print(f"  Ki        = {KI_I:.4g} V/(A*s)")


# -----------------------------------------------------------------------------
# Plant theory overlays
# -----------------------------------------------------------------------------

f_th = np.logspace(np.log10(1.0), np.log10(2000.0), 1000)
w_th = 2.0 * np.pi * f_th
s_th = 1j * w_th

P_nom  = 1.0 / (R_NOM  + s_th * L_NOM)
P_conf = 1.0 / (R_MEAS + s_th * L_MEAS)

mag_nom  = 20.0 * np.log10(np.abs(P_nom))
mag_conf = 20.0 * np.log10(np.abs(P_conf))

phi_nom  = np.degrees(np.angle(P_nom))
phi_conf = np.degrees(np.angle(P_conf))

# Theory plus plant delay
phi_conf_delay = phi_conf - 360.0 * f_th * TD_PLANT

mag_dc  = 20.0 * np.log10(1.0 / R_MEAS)
mag_3db = mag_dc - 3.0


# -----------------------------------------------------------------------------
# Current-loop gain and margins -- uses the ACTUAL deployed gains
# (DEPLOYED_KP/KI) against the TRUE per-phase plant, not P_conf (which is
# R_MEAS/L_MEAS as fit, i.e. line-to-line-equivalent -- halve both for the
# per-phase R/L the firmware's Kp/L relationship actually assumes; see the
# CURRENT_LOOP_KP/KI derivation comment in config.h).
# -----------------------------------------------------------------------------

R_TRUE = R_MEAS / 2.0
L_TRUE = L_MEAS / 2.0
P_true = 1.0 / (R_TRUE + s_th * L_TRUE)

print(f"\nTrue per-phase plant (R_MEAS/L_MEAS halved): R={R_TRUE:.3f} ohm  L={L_TRUE*1000:.3f} mH")

C_i = DEPLOYED_KP + DEPLOYED_KI / s_th

Delay_loop = np.exp(-s_th * TD_LOOP)

Loop = C_i * P_true * Delay_loop

loop_mag_db = 20.0 * np.log10(np.abs(Loop) + 1e-20)
loop_phase_deg = np.degrees(np.unwrap(np.angle(Loop)))

# Gain crossover: |Loop| = 1, or 0 dB
fgc, _ = interp_log_x_for_y(f_th, loop_mag_db, 0.0)

pm = None
phase_gc = None

if fgc is not None:
    phase_gc = interp_log_y_at_f(f_th, loop_phase_deg, fgc)
    pm = 180.0 + phase_gc

# Phase crossover: phase = -180 deg
fpc, _ = interp_log_x_for_y(f_th, loop_phase_deg, -180.0)

gm_db = None
mag_pc = None

if fpc is not None:
    mag_pc = interp_log_y_at_f(f_th, loop_mag_db, fpc)
    gm_db = -mag_pc

print("\nMeasured plant phase debug:")
print("  Expected plant theory is negative phase for current lagging voltage.\n")

for ff in [10, 50, 100, 229, 500, 1000]:
    idx = np.argmin(np.abs(f_csd - ff))

    h_theory = 1.0 / (R_MEAS + 1j * 2.0 * np.pi * f_csd[idx] * L_MEAS)
    phi_theory = np.degrees(np.angle(h_theory))
    phi_theory_delay = phi_theory - 360.0 * f_csd[idx] * TD_PLANT

    print(
        f"  {f_csd[idx]:7.1f} Hz: "
        f"mag={mag[idx]:7.2f} dB, "
        f"phi={phi[idx]:8.2f} deg, "
        f"theory={phi_theory:8.2f} deg, "
        f"theory+delay={phi_theory_delay:8.2f} deg, "
        f"coh={Cxy[idx]:.3f}"
    )

print("\nCurrent-loop margin estimate (DEPLOYED gains, not a fresh design):")
print("  Loop(s) = C(s)P(s), P(s) built from the true per-phase plant")
print("  C(s)    = Kp + Ki/s")
print(f"  Kp      = {DEPLOYED_KP:.6g} V/A   (config.h CURRENT_LOOP_KP)")
print(f"  Ki      = {DEPLOYED_KI:.6g} V/(A*s)   (config.h CURRENT_LOOP_KI)")
print(f"  delay   = {TD_LOOP*1e6:.1f} us")

if fgc is not None:
    print(f"  gain crossover  = {fgc:.1f} Hz")
    print(f"  phase at gc     = {phase_gc:.1f} deg")
    print(f"  phase margin    = {pm:.1f} deg")
else:
    print("  gain crossover  = none in plotted range")
    print("  phase margin    = N/A")

if fpc is not None:
    print(f"  phase crossover = {fpc:.1f} Hz")
    print(f"  gain at pc      = {mag_pc:.1f} dB")
    print(f"  gain margin     = {gm_db:.1f} dB")
else:
    print("  phase crossover = none in plotted range")
    print("  gain margin     = infinite / N/A in plotted range")


# -----------------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------------

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 16))

fig.text(
    0.995, 0.005,
    f"data: {csv_path.name} @ {csv_mtime:%Y-%m-%d %H:%M:%S}   "
    f"|   plot generated: {plot_gen_time:%Y-%m-%d %H:%M:%S}",
    ha="right", va="bottom", fontsize=8, color="dimgray",
)


# -----------------------------------------------------------------------------
# 1. Plant magnitude
# -----------------------------------------------------------------------------

ax1.semilogx(
    f_csd[band],
    mag[band],
    "b.-",
    label="Measured plant iq/Vq",
    alpha=0.7
)

ax1.semilogx(
    f_th,
    mag_nom,
    "r--",
    label=f"Nominal R={R_NOM}Ω L={L_NOM*1000:.2f}mH"
)

ax1.semilogx(
    f_th,
    mag_conf,
    "g-",
    label=f"Fitted R={R_MEAS:.2f}Ω L={L_MEAS*1000:.2f}mH fc={FC_MEAS:.0f}Hz"
)

ax1.axhline(
    mag_3db,
    color="gray",
    linestyle=":",
    alpha=0.7,
    label=f"-3dB = {mag_3db:.1f} dB"
)

ax1.axvline(
    FC_MEAS,
    color="purple",
    linestyle=":",
    alpha=0.7,
    label=f"fc = {FC_MEAS:.0f} Hz"
)

ax1.plot(FC_MEAS, mag_3db, "go", markersize=8)

tf_text = (
    f"$P(s) = \\dfrac{{1}}{{Ls + R}}$\n\n"
    f"$R = {R_MEAS:.2f}\\,\\Omega$\n"
    f"$L = {L_MEAS*1000:.2f}\\,mH$\n"
    f"$f_c = {FC_MEAS:.0f}\\,Hz$"
)

ax1.text(
    0.98,
    0.97,
    tf_text,
    transform=ax1.transAxes,
    fontsize=11,
    verticalalignment="top",
    horizontalalignment="right",
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.9)
)

ax1.set_ylabel("Plant Mag (dB)")
ax1.set_title(
    "AKM11E Current Plant Bode + Current-Loop Margin Estimate\n"
    "STM32F411 + DRV8353RS-EVM"
)
ax1.legend(fontsize=8, loc="lower left")
ax1.grid(True, which="both")


# -----------------------------------------------------------------------------
# 2. Plant phase
# -----------------------------------------------------------------------------

ax2.semilogx(
    f_csd[coh_strong],
    phi[coh_strong],
    "b.",
    label="Measured coh≥0.8",
    alpha=0.85,
    markersize=4
)

ax2.semilogx(
    f_csd[coh_good],
    phi[coh_good],
    "c.",
    label="Measured coh≥0.5",
    alpha=0.35,
    markersize=3
)

ax2.semilogx(
    f_th,
    phi_nom,
    "r--",
    label="Nominal theory"
)

ax2.semilogx(
    f_th,
    phi_conf,
    "g-",
    label="Fitted"
)

if TD_PLANT > 0:
    ax2.semilogx(
        f_th,
        phi_conf_delay,
        "k--",
        label=f"Fitted + {TD_PLANT*1e6:.0f} us delay"
    )

ax2.axvline(
    FC_MEAS,
    color="purple",
    linestyle=":",
    alpha=0.7
)

ax2.set_ylabel("Plant Phase (deg)")
ax2.set_ylim([-140, 40])
ax2.legend(fontsize=8)
ax2.grid(True, which="both")


# -----------------------------------------------------------------------------
# 3. Coherence
# -----------------------------------------------------------------------------

ax3.semilogx(
    f_coh[band],
    Cxy[band],
    "g.-",
    label="Coherence"
)

ax3.axhline(
    0.5,
    color="r",
    linestyle="--",
    label="0.5 threshold"
)

ax3.axhline(
    0.8,
    color="orange",
    linestyle="--",
    label="0.8 threshold"
)

ax3.set_ylabel("Coherence")
ax3.set_ylim([0, 1.1])
ax3.legend(fontsize=8)
ax3.grid(True, which="both")


# -----------------------------------------------------------------------------
# 4. Loop gain magnitude
# -----------------------------------------------------------------------------

ax4.semilogx(
    f_th,
    loop_mag_db,
    "b-",
    label="$|C(s)P(s)|$"
)

ax4.axhline(
    0.0,
    color="gray",
    linestyle=":",
    alpha=0.8,
    label="0 dB"
)

if fgc is not None:
    ax4.axvline(
        fgc,
        color="purple",
        linestyle=":",
        alpha=0.8,
        label=f"gc={fgc:.1f} Hz, PM={pm:.1f}°"
    )

if fpc is not None:
    ax4.axvline(
        fpc,
        color="red",
        linestyle=":",
        alpha=0.8,
        label=f"pc={fpc:.1f} Hz, GM={gm_db:.1f} dB"
    )

margin_text = ""

if fgc is not None:
    margin_text += f"PM = {pm:.1f}° @ {fgc:.1f} Hz\n"
else:
    margin_text += "PM = N/A\n"

if fpc is not None:
    margin_text += f"GM = {gm_db:.1f} dB @ {fpc:.1f} Hz"
else:
    margin_text += "GM = ∞ / N/A"

ax4.text(
    0.98,
    0.95,
    margin_text,
    transform=ax4.transAxes,
    fontsize=10,
    verticalalignment="top",
    horizontalalignment="right",
    bbox=dict(boxstyle="round", facecolor="white", alpha=0.9)
)

ax4.set_ylabel("Loop Mag (dB)")
ax4.legend(fontsize=8)
ax4.grid(True, which="both")


# -----------------------------------------------------------------------------
# 5. Loop phase
# -----------------------------------------------------------------------------

ax5.semilogx(
    f_th,
    loop_phase_deg,
    "b-",
    label="Loop phase"
)

ax5.axhline(
    -180.0,
    color="gray",
    linestyle=":",
    alpha=0.8,
    label="-180°"
)

if fgc is not None:
    ax5.axvline(
        fgc,
        color="purple",
        linestyle=":",
        alpha=0.8,
        label=f"gc={fgc:.1f} Hz"
    )

if fpc is not None:
    ax5.axvline(
        fpc,
        color="red",
        linestyle=":",
        alpha=0.8,
        label=f"pc={fpc:.1f} Hz"
    )

ax5.set_ylabel("Loop Phase (deg)")
ax5.set_xlabel("Frequency (Hz)")
ax5.legend(fontsize=8)
ax5.grid(True, which="both")


plt.tight_layout()
plt.savefig(out_path, dpi=160)

print(f"\nwrote: {out_path}")