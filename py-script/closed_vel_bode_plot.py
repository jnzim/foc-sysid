#!/usr/bin/env python3
"""Analyze SYSID_TEST_CL_VEL_CHIRP -- CLOSED velocity loop chirp.

Usage:
    python3 closed_vel_bode_plot.py drive_data/sysid_log.csv

Unlike bode_vel_plot.py (which chirps iq open-loop to identify the raw
mechanical PLANT), this test chirps vel_cmd with the velocity PI already
closed around it -- so this script identifies the CLOSED-loop response
H(s) = vel_meas / vel_cmd directly, and uses its measured -3dB bandwidth
to recommend a P-only gain for an outer position loop wrapped around it
(position loop plant = H(s)/s, since velocity integrates to position).

Telemetry mapping (same convention as CL_VEL_STEP):
    sysid_f   -> vel_cmd [mrad/s]
    iq_cmd_mA -> measured velocity [counts/s]
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

ENCODER_CPR = 8192.0
FLAG_RUN = 1
SETTLE_TIME_S = 2.0          # discard settle transient at the start of RUN
POSITION_LOOP_DECADE = 10.0  # rule of thumb: outer crossover 1 decade below inner BW

# Velocity loop gains actually flashed for this run (loops.c VEL_KP/VEL_KI).
# When VEL_KI_TEST=0, C(s)=VEL_KP_TEST is a real constant and the plant can
# be divided out directly (P(s)=L(s)/VEL_KP_TEST). When it's nonzero (a
# real PI is under test, not the P-only plant-ID trick), that division is
# invalid -- C(s) is frequency-dependent -- so the plant back-out below is
# skipped automatically; only the loop's own backed-out margin (gc/PM/GM,
# which needs no knowledge of C(s) at all) is computed in that case.
VEL_KP_TEST = 0.02053   # A / (rad/s)
VEL_KI_TEST = 1.6505    # A / rad -- 0.0 for a P-only plant-ID run


def first_order_complex(f, K, tau):
    w = 2.0 * np.pi * f
    Hm = K / (1j * w * tau + 1.0)
    return np.concatenate([Hm.real, Hm.imag])


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
    print("Usage: python3 closed_vel_bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "closed_vel_bode_plot.png"

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

vel_cmd  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0                       # rad/s
vel_meas = df["iq_cmd_mA"].to_numpy(dtype=np.float64) * (2.0 * np.pi / ENCODER_CPR)  # rad/s

print(f"samples     : {len(df)}")
print(f"sample rate : {fs:.1f} Hz")
print(f"vel_cmd     : {vel_cmd.min():+.3f} to {vel_cmd.max():+.3f} rad/s")
print(f"vel_meas    : {vel_meas.min():+.3f} to {vel_meas.max():+.3f} rad/s")

cmd_ac  = detrend(vel_cmd,  type="linear")
meas_ac = detrend(vel_meas, type="linear")

nperseg = min(len(df), max(256, int(fs * 2.0)))

f_csd, S_cm = csd(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)   # cmd->meas cross spectrum
_,     S_cc = welch(cmd_ac,        fs=fs, nperseg=nperseg)   # cmd auto spectrum
f_coh, Cxy  = coherence(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)

H = S_cm / (S_cc + 1e-30)   # closed-loop H(f) = vel_meas / vel_cmd

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
low_ref_mask = f_good <= max(f_good.min() * 3.0, f_good.min() + 1.0)
mag_dc = float(np.median(mag_good[low_ref_mask]))
mag_3db = mag_dc - 3.0

f_bw = interp_log_x_for_y(f_csd[good], mag_good, mag_3db)

print(f"\nClosed velocity loop -- H(s) = vel_meas / vel_cmd")
print(f"  DC gain (ref)   : {mag_dc:.2f} dB")
if f_bw is not None:
    print(f"  -3dB bandwidth  : {f_bw:.2f} Hz")
    kp_pos = 2.0 * np.pi * (f_bw / POSITION_LOOP_DECADE)
    print(f"\nRecommended outer POSITION loop (pure P, crossover 1 decade below):")
    print(f"  target position BW : {f_bw / POSITION_LOOP_DECADE:.2f} Hz")
    print(f"  Kp_pos              : {kp_pos:.4g}  (rad/s per rad of position error)")
    print(f"  (valid because at that frequency H(jf) ~= {mag_dc:.1f} dB, ~0 deg --")
    print(f"   the closed velocity loop looks like a flat unity gain there)")
else:
    print("  -3dB bandwidth  : not found in swept range -- widen CL_VEL_CHIRP_F_END")
    kp_pos = None

# ---------------------------------------------------------------------------
# Back out the real open-loop L(s) = C(s)*P(s) from this closed-loop
# measurement -- valid regardless of what C(s) is, P-only or full PI:
#   H = L/(1+L)  =>  L = H/(1-H)      (same identity as the current loop)
# This is the loop's own real margin, directly comparable to the design's
# by-construction prediction (gc=BW_TARGET_HZ, PM=90deg for a pure-integrator
# zero-cancellation design) -- independent of the plant-backout step below.
#
# Same numerical trap as the current loop's L=H/(1-H): near DC a
# well-tracking loop has H->1, so (1-H)->0 and its phase is noise-dominated.
# Gate out bins where |1-H| is too small before trusting phase there.
# ---------------------------------------------------------------------------
COND_MIN_ONE_MINUS_H = 0.1

one_minus_H = 1.0 - H
plant_phase_ok = good & (np.abs(one_minus_H) > COND_MIN_ONE_MINUS_H)

if np.any(plant_phase_ok):
    f_plant_ok = f_csd[plant_phase_ok]
    L_vel = H[plant_phase_ok] / one_minus_H[plant_phase_ok]

    loop_mag_db = 20.0 * np.log10(np.abs(L_vel) + 1e-30)
    loop_phase_deg = np.degrees(np.unwrap(np.angle(L_vel)))
    f_gc_loop = interp_log_x_for_y(f_plant_ok, loop_mag_db, 0.0)
    print(f"\nVelocity loop's own backed-out margin -- L(s) = H(s)/(1-H(s)):")
    if f_gc_loop is not None:
        phase_at_gc_loop = np.interp(np.log10(f_gc_loop), np.log10(f_plant_ok), loop_phase_deg)
        pm_loop = 180.0 + phase_at_gc_loop
        print(f"  gain crossover  : {f_gc_loop:.2f} Hz")
        print(f"  phase margin    : {pm_loop:.1f} deg")
    else:
        print("  gain crossover  : not found in coherent range")
    f_pc_loop = interp_log_x_for_y(f_plant_ok, loop_phase_deg, -180.0)
    if f_pc_loop is not None:
        mag_at_pc_loop = np.interp(np.log10(f_pc_loop), np.log10(f_plant_ok), loop_mag_db)
        print(f"  phase crossover : {f_pc_loop:.2f} Hz")
        print(f"  gain margin     : {-mag_at_pc_loop:.1f} dB")
    else:
        print("  phase crossover : not found -- phase never reaches -180 deg, GM effectively infinite")

if VEL_KI_TEST != 0.0:
    print(f"\nBacked-out mechanical plant: skipped -- C(s) is a full PI here "
          f"(VEL_KI_TEST={VEL_KI_TEST}), not a real constant, so P(s)=L(s)/C "
          f"can't be done with a plain scalar division. Use a P-only run for this.")
    plant_fit_ok = False
elif np.any(plant_phase_ok):
    L_vel_p = L_vel
    P_vel = L_vel_p / VEL_KP_TEST

    plant_mag_db = 20.0 * np.log10(np.abs(P_vel) + 1e-30)
    plant_phase_deg = np.degrees(np.unwrap(np.angle(P_vel)))

    print(f"\nBacked-out mechanical plant -- P(s) = L(s)/VEL_KP_TEST, VEL_KP_TEST={VEL_KP_TEST}:")

    if len(f_plant_ok) >= 5:
        y_fit = np.concatenate([P_vel.real, P_vel.imag])
        K0 = float(np.abs(P_vel[np.argmin(f_plant_ok)]))
        try:
            popt, _ = curve_fit(first_order_complex, f_plant_ok, y_fit,
                                 p0=[K0, 1.0 / (2 * np.pi * 20.0)], maxfev=20_000)
            K_backed, tau_backed = popt
            tau_backed = abs(tau_backed)
            fc_backed = 1.0 / (2.0 * np.pi * tau_backed)
            print(f"  K   = {K_backed:.3f} rad/s per A")
            print(f"  tau = {tau_backed*1000:.2f} ms  ->  fc = {fc_backed:.2f} Hz")

            # Cross-check against the open-loop chirp fit from this same
            # bare-motor setup (SYSID_TEST_VEL_CHIRP, >=10.5Hz only,
            # 0.256A run): K=57.978 rad/s/A, tau=4.10ms, fc=38.84Hz.
            K_OL, TAU_OL_MS, FC_OL = 57.978, 4.10, 38.84
            print(f"\n  vs. open-loop chirp fit (same bare motor, 0.256A, >=10.5Hz):")
            print(f"  K   : backed-out {K_backed:.3f}  vs  OL {K_OL:.3f}  "
                  f"({(K_backed/K_OL - 1)*100:+.0f}%)")
            print(f"  tau : backed-out {tau_backed*1000:.2f}ms  vs  OL {TAU_OL_MS:.2f}ms  "
                  f"({(tau_backed*1000/TAU_OL_MS - 1)*100:+.0f}%)")
            print(f"  fc  : backed-out {fc_backed:.2f}Hz  vs  OL {FC_OL:.2f}Hz  "
                  f"({(fc_backed/FC_OL - 1)*100:+.0f}%)")
            plant_fit_ok = True
        except RuntimeError as exc:
            print(f"  fit failed: {exc}")
            plant_fit_ok = False
    else:
        print(f"  too few conditioned bins ({len(f_plant_ok)}) to fit")
        plant_fit_ok = False
else:
    print("\nBacked-out plant: no bins pass the |1-H| conditioning gate -- can't trust phase here")
    plant_fit_ok = False

# ---------------------------------------------------------------------------
# Phase-margin-targeted design -- less conservative than the decade-below-BW
# rule above. Position loop L(s) = Kp_pos * H(s)/s: the 1/s always contributes
# -90 deg, so PM_pos = 90 - |phase lag of H at the chosen crossover|. Pick the
# crossover where H's phase lag is TARGET_PHASE_LAG_DEG (default 30 deg ->
# ~60 deg position PM, healthy without being overly cautious), and size
# Kp_pos from the ACTUAL measured gain there instead of assuming unity gain.
# ---------------------------------------------------------------------------
TARGET_PHASE_LAG_DEG = 30.0

phase_good = phase_deg[good]
f_pm = interp_log_x_for_y(f_good, phase_good, -TARGET_PHASE_LAG_DEG)

if f_pm is not None:
    mag_at_fpm_db = np.interp(np.log10(f_pm), np.log10(f_good), mag_good)
    h_at_fpm = 10.0 ** (mag_at_fpm_db / 20.0)
    kp_pos_fast = (2.0 * np.pi * f_pm) / h_at_fpm

    print(f"\nFaster POSITION loop option (phase-margin targeted, ~"
          f"{90.0 - TARGET_PHASE_LAG_DEG:.0f} deg PM):")
    print(f"  crossover frequency : {f_pm:.2f} Hz")
    print(f"  H(jf) there         : {mag_at_fpm_db:.2f} dB "
          f"(measured, not assumed unity)")
    print(f"  Kp_pos              : {kp_pos_fast:.4g}  (rad/s per rad of position error)")
    if kp_pos is not None:
        print(f"  ({kp_pos_fast / kp_pos:.1f}x the conservative decade-below-BW gain above)")
else:
    print(f"\nFaster POSITION loop option: phase never reaches "
          f"-{TARGET_PHASE_LAG_DEG:.0f} deg in the coherent range -- can't "
          "target this margin from this data")
    kp_pos_fast = None

# ---------------------------------------------------------------------------
# Position loop OPEN-LOOP Bode -- the P controller actually in the forward
# path: L(s) = Kp_pos * H(s) / s, using the faster (phase-margin-targeted)
# gain. This is the direct stability check, same style as the current/
# velocity "Loop gain" panels elsewhere in this pipeline -- read gain
# crossover and phase margin straight off the measured data instead of
# inferring them from the velocity-loop-alone plot.
# ---------------------------------------------------------------------------
kp_pos_loop = kp_pos_fast if kp_pos_fast is not None else kp_pos

if kp_pos_loop is not None:
    H_good = H[good]
    w_good = 2.0 * np.pi * f_good
    L_pos = kp_pos_loop * H_good / (1j * w_good)

    loop_mag_db = 20.0 * np.log10(np.abs(L_pos) + 1e-30)
    loop_phase_deg = np.degrees(np.unwrap(np.angle(L_pos)))

    f_gc = interp_log_x_for_y(f_good, loop_mag_db, 0.0)
    # Phase crossover (phase(L) = -180 deg) -> gain margin. Kp_pos doesn't
    # shift this frequency (same reasoning as above: Kp only moves gain, not
    # phase), it only sets how far |L| sits below 0dB there.
    f_pc = interp_log_x_for_y(f_good, loop_phase_deg, -180.0)

    print(f"\nPosition loop OPEN LOOP -- L(s) = Kp_pos * H(s) / s "
          f"(Kp_pos={kp_pos_loop:.4g}):")

    if f_gc is not None:
        phase_at_gc = np.interp(np.log10(f_gc), np.log10(f_good), loop_phase_deg)
        pm_pos = 180.0 + phase_at_gc
        print(f"  gain crossover  : {f_gc:.2f} Hz")
        print(f"  phase margin    : {pm_pos:.1f} deg")
    else:
        pm_pos = None
        print("  gain crossover  : not found in coherent range")

    if f_pc is not None:
        mag_at_fpc = np.interp(np.log10(f_pc), np.log10(f_good), loop_mag_db)
        gm_pos = -mag_at_fpc
        print(f"  phase crossover : {f_pc:.2f} Hz")
        print(f"  gain margin     : {gm_pos:.1f} dB")
    else:
        gm_pos = None
        print("  phase crossover : not found in coherent range (phase never "
              "reaches -180 deg) -> gain margin effectively infinite here")
else:
    L_pos = None
    f_gc = None
    f_pc = None
    pm_pos = None
    gm_pos = None

fig, axes = plt.subplots(5, 1, figsize=(12, 16))
ax1, ax2, ax3, ax4, ax5 = axes

ax1.semilogx(f_csd[good], mag_good, "c.", alpha=0.4, markersize=3, label="coh>=0.5")
ax1.semilogx(f_csd[strong], mag_db[strong], "b.", markersize=4, label="coh>=0.8")
ax1.axhline(mag_dc, color="gray", linestyle=":", label=f"DC ref {mag_dc:.1f} dB")
ax1.axhline(mag_3db, color="orange", linestyle=":", label="-3dB")
if f_bw is not None:
    ax1.axvline(f_bw, color="red", linestyle="--", label=f"BW = {f_bw:.1f} Hz")
if f_pm is not None:
    ax1.axvline(f_pm, color="purple", linestyle="--",
                label=f"fast crossover = {f_pm:.1f} Hz")
ax1.set_ylabel("Closed-loop mag (dB)")
ax1.set_title("Closed Velocity Loop -- H(s) = vel_meas / vel_cmd")
ax1.legend(fontsize=8)
ax1.grid(True, which="both")

ax2.semilogx(f_csd[good], phase_deg[good], "c.", alpha=0.4, markersize=3)
ax2.semilogx(f_csd[strong], phase_deg[strong], "b.", markersize=4)
if f_bw is not None:
    ax2.axvline(f_bw, color="red", linestyle="--")
if f_pm is not None:
    ax2.axvline(f_pm, color="purple", linestyle="--")
    ax2.axhline(-TARGET_PHASE_LAG_DEG, color="purple", linestyle=":", alpha=0.6)
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

if L_pos is not None:
    ax4.semilogx(f_good, loop_mag_db, "m.-", markersize=3,
                 label=f"L(s) = Kp_pos*H(s)/s, Kp_pos={kp_pos_loop:.4g}")
    ax4.axhline(0.0, color="gray", linestyle=":", label="0 dB")
    if f_gc is not None:
        ax4.axvline(f_gc, color="purple", linestyle="--",
                    label=f"gc={f_gc:.2f} Hz, PM={pm_pos:.1f} deg")
    if f_pc is not None:
        ax4.axvline(f_pc, color="brown", linestyle="--",
                    label=f"pc={f_pc:.2f} Hz, GM={gm_pos:.1f} dB")
    ax4.set_ylabel("Position loop mag (dB)")
    ax4.set_title("Position Loop OPEN LOOP -- P controller in forward path")
    ax4.legend(fontsize=8)
    ax4.grid(True, which="both")

    ax5.semilogx(f_good, loop_phase_deg, "m.-", markersize=3)
    ax5.axhline(-180.0, color="gray", linestyle=":", label="-180 deg")
    if f_gc is not None:
        ax5.axvline(f_gc, color="purple", linestyle="--")
    if f_pc is not None:
        ax5.axvline(f_pc, color="brown", linestyle="--")
    ax5.set_ylabel("Position loop phase (deg)")
    ax5.set_xlabel("Frequency (Hz)")
    ax5.legend(fontsize=8)
    ax5.grid(True, which="both")
else:
    ax4.axis("off")
    ax5.axis("off")

footer = []
if kp_pos is not None:
    footer.append(f"conservative: BW={f_bw/POSITION_LOOP_DECADE:.2f} Hz, Kp_pos={kp_pos:.4g}")
if kp_pos_fast is not None:
    footer.append(f"faster: BW={f_pm:.2f} Hz, Kp_pos={kp_pos_fast:.4g}")
if pm_pos is not None:
    footer.append(f"open-loop check: gc={f_gc:.2f} Hz, PM={pm_pos:.1f} deg")
if gm_pos is not None:
    footer.append(f"pc={f_pc:.2f} Hz, GM={gm_pos:.1f} dB")
if footer:
    fig.text(0.5, 0.005, "  |  ".join(footer),
              ha="center", fontsize=9, color="dimgray")

plt.tight_layout()
plt.savefig(out_path, dpi=160)
print(f"\nwrote: {out_path}")
