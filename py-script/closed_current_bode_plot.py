#!/usr/bin/env python3
"""Analyze SYSID_TEST_CL_CURRENT_CHIRP -- CLOSED current loop chirp.

Usage:
    python3 closed_current_bode_plot.py drive_data/sysid_log.csv

Unlike bode_plot.py (which chirps Vq open-loop and combines the measured
electrical plant with the *assumed* linear controller C(s)=Kp+Ki/s to
predict PM/GM), this test chirps iq_cmd with the current PI already closed
around it -- so this script measures the CLOSED-loop response
H(s) = iq_meas / iq_cmd directly, then backs out the loop's real open-loop
transfer function algebraically: L(s) = H(s) / (1 - H(s)). No linearity
assumption about the controller survives that step -- whatever nonlinear
behavior (saturation, etc.) is actually present is baked into the measured
H and carried through.

This is the same method bode_plot.py's open-loop route was shown to
disagree with on the velocity loop (37% off on crossover, most likely from
saturation) -- this script exists to get the current loop's real measured
margin instead of trusting that design-by-construction number again.

Telemetry mapping (see foc_sysid.c run_cl_current_chirp):
    sysid_f   -> iq_cmd  [mA]
    iq_cmd_mA -> iq_meas [mA]  (mirrors the native iq_mA column)
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy.signal import coherence, csd, welch, detrend

FLAG_RUN = 1
SETTLE_TIME_S = 2.0   # discard settle transient at the start of RUN

# Design-by-construction numbers this test exists to check (see config.h
# CURRENT_LOOP_KP/KI comment) -- not used in any calculation below, printed
# only as a reference to compare the measured result against.
DESIGN_BW_HZ = 500.0
DESIGN_PM_DEG_LO = 85.0
DESIGN_PM_DEG_HI = 88.0

# Gains actually deployed on the STM32 right now (config.h CURRENT_LOOP_KP/KI).
# Used below to compute what THESE gains predict against the plant fit,
# independent of whatever BW label they were designed under.
DEPLOYED_KP = 2.07       # V/A
DEPLOYED_KI = 2450.0     # V/(A*s)

# Plant fit config.h's CURRENT_LOOP_KP/KI comment cites as "the real" current
# best estimate ("...against the real (now 1.67/1.41mH) plant fit"). Not
# re-fit in this script -- this test is closed-loop (iq_cmd->iq_meas), it
# has no open-loop Vd/id data of its own to fit R/L from. See bode_plot.py
# for the open-loop plant identification these numbers came from.
PLANT_R = 1.67           # ohm, per-phase
PLANT_L = 1.41e-3        # H, per-phase


def zero_cancel_pi(bw_hz, R, L):
    """Kp/Ki for a zero-cancellation PI design against P(s)=1/(Ls+R)
    crossing 0dB at bw_hz (same formula as bode_plot.py's PI design)."""
    wc = 2.0 * np.pi * bw_hz
    kp = wc * L
    ki = kp * (R / L)
    return kp, ki


def closed_loop_theory(f_hz, kp, ki, R, L):
    """H(s) = C(s)P(s) / (1 + C(s)P(s)), C=Kp+Ki/s, P=1/(Ls+R)."""
    s = 1j * 2.0 * np.pi * f_hz
    C = kp + ki / s
    P = 1.0 / (L * s + R)
    Lp = C * P
    return Lp / (1.0 + Lp)


def first_order_complex(f, K, tau):
    """Generic single-pole model, fit directly to measured H(f) -- same
    functional form and curve_fit pattern bode_plot.py uses for its own
    (open-loop) plant fit."""
    w = 2.0 * np.pi * f
    H = K / (1j * w * tau + 1.0)
    return np.concatenate([H.real, H.imag])


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def interp_log_x_for_y(x_hz, y, target):
    """First log-frequency crossing of y through target. None if none found."""
    y_shift = y - target
    idxs = np.where(np.diff(np.sign(y_shift)) != 0)[0]
    if len(idxs) == 0:
        return None
    i = idxs[0]
    f1, f2 = x_hz[i], x_hz[i + 1]
    y1, y2 = y_shift[i], y_shift[i + 1]
    x1, x2 = np.log10(f1), np.log10(f2)
    if abs(y2 - y1) < 1e-20:
        return f1
    x_cross = x1 - y1 * (x2 - x1) / (y2 - y1)
    return 10.0 ** x_cross


if len(sys.argv) != 2:
    print("Usage: python3 closed_current_bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "closed_current_bode_plot.png"

df = pd.read_csv(csv_path)
required = ["host_time_s", "flags", "sysid_f", "iq_cmd_mA", "dt"]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing CSV columns: {', '.join(missing)}")

flags_numeric = parse_flags(df["flags"])
df = df[(flags_numeric == FLAG_RUN) & (df["dt"] > 0)].copy()
df = df[df["sysid_f"] != 0].copy()
if len(df) < 100:
    raise RuntimeError("Too few RUN-stage samples in the CSV")

run_start_time = df["host_time_s"].to_numpy(dtype=np.float64)[0]
df = df[df["host_time_s"] >= run_start_time + SETTLE_TIME_S].copy()
if len(df) < 100:
    raise RuntimeError(
        f"Too few samples remain after discarding the first {SETTLE_TIME_S}s "
        "settling transient"
    )

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]
dt_arr = np.diff(t)
if np.any(dt_arr <= 0):
    raise RuntimeError("host_time_s must be strictly increasing after filtering")
fs = 1.0 / np.median(dt_arr)

iq_cmd  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0        # A
iq_meas = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0      # A

print(f"samples     : {len(df)}")
print(f"sample rate : {fs:.1f} Hz")
print(f"iq_cmd      : {iq_cmd.min():+.3f} to {iq_cmd.max():+.3f} A")
print(f"iq_meas     : {iq_meas.min():+.3f} to {iq_meas.max():+.3f} A")

cmd_ac  = detrend(iq_cmd,  type="linear")
meas_ac = detrend(iq_meas, type="linear")

nperseg = min(len(df), max(256, int(fs * 2.0)))

f_csd, S_cm = csd(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)   # cmd->meas cross spectrum
_,     S_cc = welch(cmd_ac,        fs=fs, nperseg=nperseg)   # cmd auto spectrum
f_coh, Cxy  = coherence(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)

H = S_cm / (S_cc + 1e-30)   # closed-loop H(f) = iq_meas / iq_cmd

mag_db    = 20.0 * np.log10(np.abs(H) + 1e-30)
phase_deg = np.degrees(np.unwrap(np.angle(H)))

band = (f_csd >= 0.2) & (f_csd <= fs * 0.45) & np.isfinite(mag_db) & np.isfinite(phase_deg)
good   = band & (Cxy >= 0.5)
strong = band & (Cxy >= 0.8)

if not np.any(good):
    raise RuntimeError("No frequency bins have coherence >= 0.5")

# DC/low-frequency gain reference for the -3dB point -- median of the lowest
# decade of good bins, more robust than just the single lowest bin.
f_good = f_csd[good]
mag_good = mag_db[good]
phase_good = phase_deg[good]
low_ref_mask = f_good <= max(f_good.min() * 3.0, f_good.min() + 1.0)
mag_dc = float(np.median(mag_good[low_ref_mask]))
mag_3db = mag_dc - 3.0

f_bw = interp_log_x_for_y(f_csd[good], mag_good, mag_3db)

print(f"\nClosed current loop -- H(s) = iq_meas / iq_cmd")
print(f"  DC gain (ref)   : {mag_dc:.2f} dB")
if f_bw is not None:
    print(f"  -3dB bandwidth  : {f_bw:.2f} Hz  (design target: {DESIGN_BW_HZ:.0f} Hz)")
else:
    print("  -3dB bandwidth  : not found in swept range -- widen CL_CURRENT_CHIRP_F_END")

# ---------------------------------------------------------------------------
# Two comparison curves, both requested directly rather than left as a bare
# number:
#   (1) "the model that shows 500Hz" -- a genuine zero-cancellation design
#       for BW_TARGET=500Hz against the cited plant fit (PLANT_R/PLANT_L).
#       This is NOT what's deployed; it's what Kp/Ki *would* need to be.
#   (2) the DEPLOYED gains' own theoretical prediction against that same
#       plant fit -- shows whether the 500Hz shortfall is a measurement
#       artifact or just what these Kp/Ki were always going to produce.
#   (3) a direct curve fit of the MEASURED H(f) itself (single-pole model,
#       same fitting approach bode_plot.py uses for its open-loop plant
#       fit) -- a smooth-model BW estimate, less sensitive to any single
#       noisy bin than the raw -3dB crossing above.
# ---------------------------------------------------------------------------
f_th = np.logspace(np.log10(0.3), np.log10(fs * 0.45), 500)

KP_500, KI_500 = zero_cancel_pi(DESIGN_BW_HZ, PLANT_R, PLANT_L)
H_500_th = closed_loop_theory(f_th, KP_500, KI_500, PLANT_R, PLANT_L)

H_deployed_th = closed_loop_theory(f_th, DEPLOYED_KP, DEPLOYED_KI, PLANT_R, PLANT_L)
deployed_bw_hz = DEPLOYED_KP / PLANT_L / (2.0 * np.pi)  # exact for perfect cancellation

print(f"\nComparison models against plant fit R={PLANT_R:.2f} ohm, L={PLANT_L*1000:.2f} mH:")
print(f"  genuine {DESIGN_BW_HZ:.0f}Hz design would need Kp={KP_500:.3g} V/A, Ki={KI_500:.4g} V/(A*s)")
print(f"  deployed Kp={DEPLOYED_KP:.3g}/Ki={DEPLOYED_KI:.4g} predicts BW={deployed_bw_hz:.1f} Hz "
      f"(zero/pole ratio={  (DEPLOYED_KI/DEPLOYED_KP) / (PLANT_R/PLANT_L) :.3f}, 1.0=perfect cancellation)")

K0   = 10.0 ** (mag_dc / 20.0)
tau0 = 1.0 / (2.0 * np.pi * (f_bw if f_bw is not None else DESIGN_BW_HZ))
try:
    y_fit = np.concatenate([H[good].real, H[good].imag])
    popt, _ = curve_fit(first_order_complex, f_good, y_fit, p0=[K0, tau0], maxfev=20_000)
    K_fit, tau_fit = popt
    tau_fit = abs(tau_fit)
    fc_fit = 1.0 / (2.0 * np.pi * tau_fit)
    H_fit_th = (K_fit) / (1j * 2.0 * np.pi * f_th * tau_fit + 1.0)
    print(f"\nCurve fit of measured H(f) (single-pole model, coh>=0.5 bins):")
    print(f"  K   = {K_fit:.3f} ({20*np.log10(abs(K_fit)):.2f} dB)")
    print(f"  -3dB bandwidth (fit) : {fc_fit:.2f} Hz")
except RuntimeError as exc:
    print(f"\nCurve fit of measured H(f) failed: {exc}")
    fc_fit = None
    H_fit_th = None

# ---------------------------------------------------------------------------
# Back out the real open-loop transfer function algebraically:
#   H = L / (1 + L)  =>  L = H / (1 - H)
# No assumption that C(s) is the linear Kp+Ki/s it's designed as -- whatever
# is actually happening (saturation, ADC quantization, PWM deadtime, etc.)
# is baked into the measured H and carried straight through this identity.
#
# Numerical trap: near DC a well-tracking closed loop has H -> 1, so
# (1-H) -> 0 and its phase is dominated by measurement noise, not signal
# (verified directly against this dataset: |1-H| ~0.005-0.12 in the first
# few bins here, vs >0.1 once the loop actually rolls off). np.unwrap()
# chains corrections from the first sample forward, so noise in that one
# bin's phase drags every subsequent (otherwise clean) bin off by a
# spurious +-360 deg -- e.g. a raw run of this data reported PM=474.2 deg,
# which is meaningless (PM can't exceed 180 deg) but 474.2-360=114.2 deg
# matches exactly what this gate below produces. |L|'s magnitude (hence
# gc) was never affected -- only phase(1-H) is ill-conditioned, not the
# ratio's modulus.
# ---------------------------------------------------------------------------
COND_MIN_ONE_MINUS_H = 0.1   # exclude bins where |1-H| is noise-dominated

one_minus_H = 1.0 - H
phase_ok = good & (np.abs(one_minus_H) > COND_MIN_ONE_MINUS_H)
if not np.any(phase_ok):
    raise RuntimeError("No bins pass the |1-H| conditioning gate -- can't "
                        "trust any backed-out open-loop phase for this run")

f_phase_ok = f_csd[phase_ok]
H_ok = H[phase_ok]
L = H_ok / one_minus_H[phase_ok]

loop_mag_db = 20.0 * np.log10(np.abs(L) + 1e-30)
loop_phase_deg = np.degrees(np.unwrap(np.angle(L)))

f_gc = interp_log_x_for_y(f_phase_ok, loop_mag_db, 0.0)
f_pc = interp_log_x_for_y(f_phase_ok, loop_phase_deg, -180.0)

print(f"\nCurrent loop OPEN LOOP (backed out) -- L(s) = H(s) / (1 - H(s)):")

if f_gc is not None:
    phase_at_gc = np.interp(np.log10(f_gc), np.log10(f_phase_ok), loop_phase_deg)
    pm = 180.0 + phase_at_gc
    print(f"  gain crossover  : {f_gc:.2f} Hz")
    print(f"  phase margin    : {pm:.1f} deg  "
          f"(design predicted: {DESIGN_PM_DEG_LO:.0f}-{DESIGN_PM_DEG_HI:.0f} deg)")
else:
    pm = None
    print("  gain crossover  : not found in coherent range")

if f_pc is not None:
    mag_at_fpc = np.interp(np.log10(f_pc), np.log10(f_phase_ok), loop_mag_db)
    gm = -mag_at_fpc
    print(f"  phase crossover : {f_pc:.2f} Hz")
    print(f"  gain margin     : {gm:.1f} dB")
else:
    gm = None
    print("  phase crossover : not found in coherent range (phase never "
          "reaches -180 deg) -> gain margin effectively infinite here")

fig, axes = plt.subplots(5, 1, figsize=(12, 16))
ax1, ax2, ax3, ax4, ax5 = axes

ax1.semilogx(f_csd[good], mag_good, "c.", alpha=0.4, markersize=3, label="coh>=0.5")
ax1.semilogx(f_csd[strong], mag_db[strong], "b.", markersize=4, label="coh>=0.8")
ax1.axhline(mag_dc, color="gray", linestyle=":", label=f"DC ref {mag_dc:.1f} dB")
ax1.axhline(mag_3db, color="orange", linestyle=":", label="-3dB")
if f_bw is not None:
    ax1.axvline(f_bw, color="red", linestyle="--", label=f"BW = {f_bw:.1f} Hz")
ax1.axvline(DESIGN_BW_HZ, color="black", linestyle=":", alpha=0.5,
            label=f"design target {DESIGN_BW_HZ:.0f} Hz")
ax1.semilogx(f_th, 20*np.log10(np.abs(H_500_th)), "k--", linewidth=1.2,
             label=f"genuine {DESIGN_BW_HZ:.0f}Hz model (Kp={KP_500:.3g})")
ax1.semilogx(f_th, 20*np.log10(np.abs(H_deployed_th)), "-", color="darkorange",
             linewidth=1.2,
             label=f"deployed Kp/Ki prediction (BW={deployed_bw_hz:.0f}Hz)")
if H_fit_th is not None:
    ax1.semilogx(f_th, 20*np.log10(np.abs(H_fit_th)), "-", color="crimson",
                 linewidth=1.2, label=f"curve fit (BW={fc_fit:.0f}Hz)")
ax1.set_ylabel("Closed-loop mag (dB)")
ax1.set_title("Closed Current Loop -- H(s) = iq_meas / iq_cmd")
ax1.legend(fontsize=8)
ax1.grid(True, which="both")

ax2.semilogx(f_csd[good], phase_deg[good], "c.", alpha=0.4, markersize=3)
ax2.semilogx(f_csd[strong], phase_deg[strong], "b.", markersize=4)
if f_bw is not None:
    ax2.axvline(f_bw, color="red", linestyle="--")
ax2.semilogx(f_th, np.degrees(np.angle(H_500_th)), "k--", linewidth=1.2)
ax2.semilogx(f_th, np.degrees(np.angle(H_deployed_th)), "-", color="darkorange", linewidth=1.2)
if H_fit_th is not None:
    ax2.semilogx(f_th, np.degrees(np.angle(H_fit_th)), "-", color="crimson", linewidth=1.2)
ax2.set_ylabel("Closed-loop phase (deg)")
ax2.grid(True, which="both")

ax3.semilogx(f_coh[band], Cxy[band], "g.-")
ax3.axhline(0.5, color="r", linestyle="--", label="0.5")
ax3.axhline(0.8, color="orange", linestyle="--", label="0.8")
ax3.set_ylabel("Coherence")
ax3.set_xlabel("Frequency (Hz)")
ax3.set_ylim([0, 1.1])
ax3.legend(fontsize=8)
ax3.grid(True, which="both")

ax4.semilogx(f_phase_ok, loop_mag_db, "m.-", markersize=3, label="L(s) = H/(1-H)")
ax4.axhline(0.0, color="gray", linestyle=":", label="0 dB")
if f_gc is not None:
    ax4.axvline(f_gc, color="purple", linestyle="--",
                label=f"gc={f_gc:.2f} Hz" + (f", PM={pm:.1f} deg" if pm is not None else ""))
if f_pc is not None:
    ax4.axvline(f_pc, color="brown", linestyle="--",
                label=f"pc={f_pc:.2f} Hz, GM={gm:.1f} dB")
ax4.set_ylabel("Loop gain mag (dB)")
ax4.set_title("Current Loop OPEN LOOP (backed out) -- L(s) = H(s) / (1 - H(s))")
ax4.legend(fontsize=8)
ax4.grid(True, which="both")

ax5.semilogx(f_phase_ok, loop_phase_deg, "m.-", markersize=3)
ax5.axhline(-180.0, color="gray", linestyle=":", label="-180 deg")
if f_gc is not None:
    ax5.axvline(f_gc, color="purple", linestyle="--")
if f_pc is not None:
    ax5.axvline(f_pc, color="brown", linestyle="--")
ax5.set_ylabel("Loop gain phase (deg)")
ax5.set_xlabel("Frequency (Hz)")
ax5.legend(fontsize=8)
ax5.grid(True, which="both")

footer = []
if f_bw is not None:
    footer.append(f"measured BW={f_bw:.1f} Hz (design {DESIGN_BW_HZ:.0f} Hz)")
if fc_fit is not None:
    footer.append(f"curve-fit BW={fc_fit:.1f} Hz")
footer.append(f"deployed-gain prediction={deployed_bw_hz:.1f} Hz")
if f_gc is not None and pm is not None:
    footer.append(f"gc={f_gc:.2f} Hz, PM={pm:.1f} deg (design {DESIGN_PM_DEG_LO:.0f}-{DESIGN_PM_DEG_HI:.0f} deg)")
if f_pc is not None and gm is not None:
    footer.append(f"pc={f_pc:.2f} Hz, GM={gm:.1f} dB")
if footer:
    fig.text(0.5, 0.005, "  |  ".join(footer), ha="center", fontsize=9, color="dimgray")

plt.tight_layout()
plt.savefig(out_path, dpi=160)
print(f"\nwrote: {out_path}")
