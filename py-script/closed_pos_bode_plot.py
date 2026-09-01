#!/usr/bin/env python3
"""Analyze SYSID_TEST_CL_POS_CHIRP -- CLOSED position loop chirp, whole system.

Usage:
    python3 closed_pos_bode_plot.py drive_data/sysid_log.csv

Chirps pos_cmd with the FULL cascade closed (position P -> velocity PI ->
current PI, all closed) -- this measures the ACTUAL closed-loop response
H_pos(s) = pos_meas / pos_cmd directly, rather than the H(s)/s construction
(measured velocity-loop response + an assumed kinematic integrator) used to
design POSITION_LOOP_KP in closed_vel_bode_plot.py. This is the real thing,
not the theoretical stand-in for it.

Telemetry mapping (same convention as SYSID_TEST_POSITION_STEP):
    sysid_f   -> position command [mrad]
    iq_cmd_mA -> measured position [mrad]
"""

import sys
import os
import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import coherence, csd, welch, detrend

FLAG_RUN = 1
SETTLE_TIME_S = 1.5   # firmware settles for 1s (POSITION_STEP_SETTLE_TICKS @ 1kHz) before chirping

# pos_cmd/pos_meas only actually update at 1kHz (DT_POSITION, POS_LOOP_DECIMATE)
# even though the host telemetry link samples much faster (~10kHz) -- that 1kHz
# zero-order-hold shows up as spurious, high (>0.8) coherence spikes at 1/2/3/4
# kHz that are pure sampling artifacts, not plant response. There's no real
# excitation anywhere near there anyway (the chirp only sweeps up to
# CL_POS_CHIRP_F_END=150Hz in config.h), so cap the analysis band well below
# those images instead of the usual fs*0.45.
MAX_ANALYSIS_HZ = 225.0


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
    print("Usage: python3 closed_pos_bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "closed_pos_bode_plot.png"

csv_mtime = datetime.datetime.fromtimestamp(os.path.getmtime(csv_path))
plot_gen_time = datetime.datetime.now()
print(f"data file  : {csv_path.name}  (written {csv_mtime:%Y-%m-%d %H:%M:%S})")
print(f"plot gen'd : {plot_gen_time:%Y-%m-%d %H:%M:%S}")

df = pd.read_csv(csv_path)
required = ["host_time_s", "flags", "sysid_f", "iq_cmd_mA", "dt"]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing CSV columns: {', '.join(missing)}")

flags_numeric = parse_flags(df["flags"])
df = df[(flags_numeric == FLAG_RUN) & (df["dt"] > 0)].copy()
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

pos_cmd  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0       # rad
pos_meas = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0     # rad

print(f"samples     : {len(df)}")
print(f"sample rate : {fs:.1f} Hz")
print(f"pos_cmd     : {pos_cmd.min():+.4f} to {pos_cmd.max():+.4f} rad")
print(f"pos_meas    : {pos_meas.min():+.4f} to {pos_meas.max():+.4f} rad")

cmd_ac  = detrend(pos_cmd,  type="linear")
meas_ac = detrend(pos_meas, type="linear")

nperseg = min(len(df), max(256, int(fs * 2.0)))

f_csd, S_cm = csd(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)   # cmd->meas cross spectrum
_,     S_cc = welch(cmd_ac,        fs=fs, nperseg=nperseg)   # cmd auto spectrum
f_coh, Cxy  = coherence(cmd_ac, meas_ac, fs=fs, nperseg=nperseg)

H = S_cm / (S_cc + 1e-30)   # closed-loop H_pos(f) = pos_meas / pos_cmd

mag_db = 20.0 * np.log10(np.abs(H) + 1e-30)

# Cap the band BEFORE unwrapping -- unwrap() accumulates sequentially over the
# whole array, so a handful of garbage bins out at the 1kHz ZOH images would
# otherwise poison every bin after them with a huge cumulative phase runoff,
# even ones that get excluded from "good" later.
band = (f_csd >= 0.2) & (f_csd <= min(fs * 0.45, MAX_ANALYSIS_HZ)) & np.isfinite(mag_db)
phase_deg_band = np.degrees(np.unwrap(np.angle(H[band])))

phase_deg = np.full_like(mag_db, np.nan)
phase_deg[band] = phase_deg_band

good   = band & (Cxy >= 0.5)
strong = band & (Cxy >= 0.8)

if not np.any(good):
    raise RuntimeError("No frequency bins have coherence >= 0.5")

f_good   = f_csd[good]
mag_good = mag_db[good]
low_ref_mask = f_good <= max(f_good.min() * 3.0, f_good.min() + 1.0)
mag_dc = float(np.median(mag_good[low_ref_mask]))
mag_3db = mag_dc - 3.0

f_bw = interp_log_x_for_y(f_good, mag_good, mag_3db)

print(f"\nClosed position loop -- H_pos(s) = pos_meas / pos_cmd (whole system)")
print(f"  DC gain (ref)   : {mag_dc:.2f} dB")
if f_bw is not None:
    print(f"  -3dB bandwidth  : {f_bw:.2f} Hz")
else:
    print("  -3dB bandwidth  : not found in swept range -- widen CL_POS_CHIRP_F_END")

# ---------------------------------------------------------------------------
# Open-loop reconstruction directly from the MEASURED closed loop. For a
# standard unity-feedback loop, H = L/(1+L), so L = H/(1-H) -- no model
# assumptions needed beyond unity feedback (which is what the encoder
# actually gives you here). Unlike closed_vel_bode_plot.py, which built
# L(s) = Kp*H(s)/s from a measured velocity-loop response plus an ASSUMED
# kinematic integrator, this L(s) comes entirely from this test's own
# whole-system closed-loop data.
# ---------------------------------------------------------------------------
H_good     = H[good]
phase_good = phase_deg[good]

L_pos = H_good / (1.0 - H_good + 1e-30)
loop_mag_db    = 20.0 * np.log10(np.abs(L_pos) + 1e-30)
loop_phase_deg = np.degrees(np.unwrap(np.angle(L_pos)))

f_gc = interp_log_x_for_y(f_good, loop_mag_db, 0.0)
f_pc = interp_log_x_for_y(f_good, loop_phase_deg, -180.0)

print(f"\nPosition loop OPEN LOOP -- L(s) = H_pos(s) / (1 - H_pos(s)), "
      f"reconstructed directly from the measured closed-loop data:")

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
    print("  phase crossover : not found in swept range -- if you need a real "
          "gain margin number, widen CL_POS_CHIRP_F_END in config.h and rerun "
          "(gain margin is effectively infinite within what was actually swept)")

fig, axes = plt.subplots(5, 1, figsize=(12, 16))
ax1, ax2, ax3, ax4, ax5 = axes

ax1.semilogx(f_csd[good], mag_good, "c.", alpha=0.4, markersize=3, label="coh>=0.5")
ax1.semilogx(f_csd[strong], mag_db[strong], "b.", markersize=4, label="coh>=0.8")
ax1.axhline(mag_dc, color="gray", linestyle=":", label=f"DC ref {mag_dc:.1f} dB")
ax1.axhline(mag_3db, color="orange", linestyle=":", label="-3dB")
if f_bw is not None:
    ax1.axvline(f_bw, color="red", linestyle="--", label=f"BW = {f_bw:.1f} Hz")
ax1.set_ylabel("Closed-loop mag (dB)")
ax1.set_title("Closed Position Loop, Whole System -- H_pos(s) = pos_meas / pos_cmd")
ax1.legend(fontsize=8)
ax1.grid(True, which="both")

ax2.semilogx(f_csd[good], phase_deg[good], "c.", alpha=0.4, markersize=3)
ax2.semilogx(f_csd[strong], phase_deg[strong], "b.", markersize=4)
if f_bw is not None:
    ax2.axvline(f_bw, color="red", linestyle="--")
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

ax4.semilogx(f_good, loop_mag_db, "m.-", markersize=3, label="L(s) = H_pos/(1-H_pos)")
ax4.axhline(0.0, color="gray", linestyle=":", label="0 dB")
if f_gc is not None:
    ax4.axvline(f_gc, color="purple", linestyle="--",
                label=f"gc={f_gc:.2f} Hz, PM={pm_pos:.1f} deg")
if f_pc is not None:
    ax4.axvline(f_pc, color="brown", linestyle="--",
                label=f"pc={f_pc:.2f} Hz, GM={gm_pos:.1f} dB")
ax4.set_ylabel("Position loop OPEN loop mag (dB)")
ax4.set_title("Position Loop OPEN LOOP -- reconstructed from measured H_pos(s)")
ax4.legend(fontsize=8)
ax4.grid(True, which="both")

ax5.semilogx(f_good, loop_phase_deg, "m.-", markersize=3)
ax5.axhline(-180.0, color="gray", linestyle=":", label="-180 deg")
if f_gc is not None:
    ax5.axvline(f_gc, color="purple", linestyle="--")
if f_pc is not None:
    ax5.axvline(f_pc, color="brown", linestyle="--")
ax5.set_ylabel("Position loop OPEN loop phase (deg)")
ax5.set_xlabel("Frequency (Hz)")
ax5.legend(fontsize=8)
ax5.grid(True, which="both")

footer = []
if f_bw is not None:
    footer.append(f"whole-system BW = {f_bw:.2f} Hz")
footer.append(f"DC gain = {mag_dc:.1f} dB")
if pm_pos is not None:
    footer.append(f"gc={f_gc:.2f} Hz, PM={pm_pos:.1f} deg")
if gm_pos is not None:
    footer.append(f"pc={f_pc:.2f} Hz, GM={gm_pos:.1f} dB")
footer.append(f"data: {csv_path.name} @ {csv_mtime:%Y-%m-%d %H:%M:%S}")
fig.text(0.5, 0.005, "  |  ".join(footer), ha="center", fontsize=9, color="dimgray")

plt.tight_layout()
plt.savefig(out_path, dpi=160)
print(f"\nwrote: {out_path}")
