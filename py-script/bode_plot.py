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
#   P(s) = id / vd = 1 / (Ls + R)
#
# Loop gain for current controller:
#   Loop(s) = C(s) P(s)
#   C(s) = Kp + Ki/s

import sys
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scipy.signal import csd, welch, coherence


# -----------------------------------------------------------------------------
# Motor / plant parameters
# -----------------------------------------------------------------------------

# Nominal motor parameters
R_LL = 3.10
L_LL = 0.00204

R_NOM = R_LL / 2.0       # 1.55 ohm
L_NOM = L_LL / 2.0       # 1.02 mH


# Confirmed line-line values
R_MEAS_LL = 3.55
FC_MEAS = 229.0

# dq/phase plant values
R_MEAS = R_MEAS_LL / 2.0
L_MEAS = R_MEAS / (2.0 * np.pi * FC_MEAS)

# Optional plant measurement delay overlay
TD_PLANT = 25e-6   # seconds; try 0, 10e-6, 25e-6, 50e-6


# -----------------------------------------------------------------------------
# Current-loop controller for margin analysis
# -----------------------------------------------------------------------------
# Replace these with your actual firmware current-loop gains.
#
# Units:
#   KP_I = V/A
#   KI_I = V/(A*s)
#
# Loop gain:
#   L(s) = (Kp + Ki/s) * 1/(Ls + R)

KP_I = 7.78       # V/A      <-- replace with real current-loop Kp
KI_I = 11153.0     # V/(A*s)  <-- replace with real current-loop Ki

# Optional digital/control/PWM/ADC delay in loop margin estimate
TD_LOOP = 25e-6  # seconds


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

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

df = pd.read_csv(csv_path)

df = df[df["flags"] == "0x0001"]
df = df[(df["host_time_s"] > 0.5) & (df["host_time_s"] < 20.0)]
df = df[df["sysid_f"] > 0]
df = df[df["dt"] > 0]
df = df[df["vd_mV"].abs() < 1500]

t  = df["host_time_s"].values - df["host_time_s"].min()
vd = df["vd_mV"].values / 1000.0


id_ = df["id_mA"].values / 1000.0

fs = 1.0 / np.mean(np.diff(t))

print(f"samples : {len(df)}  fs: {fs:.1f} Hz")
print(f"vd      : {vd.min()*1000:.1f} to {vd.max()*1000:.1f} mV")
print(f"id      : {id_.min()*1000:.1f} to {id_.max()*1000:.1f} mA")
print(f"freq    : {df['sysid_f'].min():.1f} to {df['sysid_f'].max():.1f} Hz")
print(f"corr    : {np.corrcoef(vd, id_)[0,1]:.4f}")

print("\nConfirmed plant parameters:")
print("  P(s)  = 1 / (Ls + R)")
print(f"  R     = {R_MEAS:.2f} ohm")
print(f"  L     = {L_MEAS*1000:.2f} mH")
print(f"  fc    = {FC_MEAS:.0f} Hz")

# Remove DC bias before spectral estimate
vd_ac = vd - np.mean(vd)
id_ac = id_ - np.mean(id_)


# -----------------------------------------------------------------------------
# Welch / CSD transfer estimate
# -----------------------------------------------------------------------------

nperseg = int(fs * 2.0)

f_csd, Piv = csd(vd_ac, id_ac, fs=fs, nperseg=nperseg)
f_csd, Pvv = welch(vd_ac,     fs=fs, nperseg=nperseg)
f_coh, Cxy = coherence(vd_ac, id_ac, fs=fs, nperseg=nperseg)

# Transfer estimate: P = id / vd
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
# Current-loop gain and margins
# -----------------------------------------------------------------------------

C_i = KP_I + KI_I / s_th

Delay_loop = np.exp(-s_th * TD_LOOP)

Loop = C_i * P_conf * Delay_loop

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

print("\nCurrent-loop margin estimate:")
print("  Loop(s) = C(s)P(s)")
print("  C(s)    = Kp + Ki/s")
print(f"  Kp      = {KP_I:.6g} V/A")
print(f"  Ki      = {KI_I:.6g} V/(A*s)")
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


# -----------------------------------------------------------------------------
# 1. Plant magnitude
# -----------------------------------------------------------------------------

ax1.semilogx(
    f_csd[band],
    mag[band],
    "b.-",
    label="Measured plant id/Vd",
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
    label=f"Confirmed R={R_MEAS:.2f}Ω L={L_MEAS*1000:.2f}mH fc={FC_MEAS:.0f}Hz"
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
    label="Confirmed"
)

if TD_PLANT > 0:
    ax2.semilogx(
        f_th,
        phi_conf_delay,
        "k--",
        label=f"Confirmed + {TD_PLANT*1e6:.0f} us delay"
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