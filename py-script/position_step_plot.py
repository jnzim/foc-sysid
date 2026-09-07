#!/usr/bin/env python3
"""Analyze SYSID_TEST_POSITION_STEP -- position loop step response.

Usage:
    python3 position_step_plot.py drive_data/sysid_log.csv

Cascade under test: position P -> velocity PI -> current PI (all closed).
position_loop.kp is designed from the closed-velocity-loop chirp fit
(see closed_vel_bode_plot.py).

Telemetry mapping:
    sysid_f   -> position command [mrad]
    iq_cmd_mA -> measured position [mrad]
    enc_hi_raw -> vel_cmd_rad_sec, the position P controller's output [mrad/s]
    enc_lo_raw -> measured velocity [counts/s] (encoder_position is fully
                  redundant with pos_meas for this test, so those two slots
                  are repurposed -- see foc_sysid.c telemetry section)
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import medfilt

FLAG_RUN = 1


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def contiguous_regions(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def step_metrics(t, pos, cmd_before, cmd_after):
    """Classic step-response metrics for a transition from cmd_before to
    cmd_after, using the first index as t=0 for this segment."""
    step_size = cmd_after - cmd_before
    if abs(step_size) < 1e-9:
        return None

    # The telemetry link has no CRC check -- a single corrupted SPI sample
    # shows up as an instantaneous, physically-impossible jump. A light
    # median filter kills those single-sample spikes (kernel is ~0.5ms at
    # this sample rate, negligible next to a multi-ms rise time) without
    # smearing the real transition.
    pos = medfilt(pos, kernel_size=5)

    t0 = t[0]
    tt = t - t0
    err_band = 0.02 * abs(step_size)  # 2% settling band

    # Rise time: 10% -> 90% of the step. first_cross()'s sign(step_size)
    # term already handles rising vs. falling steps correctly -- do NOT
    # swap lo/hi here, that inverts which crossing is "first" in time and
    # produces a negative rise time for falling steps.
    lo = cmd_before + 0.1 * step_size
    hi = cmd_before + 0.9 * step_size

    def first_cross(level):
        idx = np.where((pos - level) * np.sign(step_size) >= 0)[0]
        return tt[idx[0]] if len(idx) else None

    t_lo = first_cross(lo)
    t_hi = first_cross(hi)
    rise_time = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    # Overshoot beyond the final command. Uses a 99.5th-percentile "peak"
    # instead of true max/min -- the telemetry link has no CRC check, and a
    # single corrupted SPI sample (an instantaneous, physically-impossible
    # jump) would otherwise blow this number out.
    if step_size > 0:
        peak = np.percentile(pos, 99.5)
        overshoot_pct = 100.0 * max(0.0, peak - cmd_after) / abs(step_size)
    else:
        peak = np.percentile(pos, 0.5)
        overshoot_pct = 100.0 * max(0.0, cmd_after - peak) / abs(step_size)

    # Settling time: last time the response leaves the 2% band and never returns
    outside = np.where(np.abs(pos - cmd_after) > err_band)[0]
    settling_time = tt[outside[-1]] if len(outside) else 0.0

    return {
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "step_size_rad": step_size,
    }


if len(sys.argv) != 2:
    print("Usage: python3 position_step_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "position_step_plot.png"

ENCODER_CPR = 8192.0
VEL_TELEM_DIV = 8.0  # firmware sends vel_meas_counts / VEL_TELEM_DIV as int16

df = pd.read_csv(csv_path)
required = ["host_time_s", "flags", "sysid_f", "iq_cmd_mA", "iq_mA", "vq_mV", "dt",
            "enc_hi_raw", "enc_lo_raw"]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing CSV columns: {', '.join(missing)}")

flags = parse_flags(df["flags"]).to_numpy()
run = flags == FLAG_RUN
df = df[run & (df["dt"] > 0)].copy()
if len(df) < 100:
    raise RuntimeError("Too few RUN-stage samples in the CSV")

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]

pos_cmd  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0       # rad
pos_meas = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0     # rad
iq_meas  = df["iq_mA"].to_numpy(dtype=np.float64)
vq_mV    = df["vq_mV"].to_numpy(dtype=np.float64)

vel_cmd  = df["enc_hi_raw"].to_numpy(dtype=np.float64) / 1000.0                       # rad/s
vel_meas = df["enc_lo_raw"].to_numpy(dtype=np.float64) * VEL_TELEM_DIV * (2.0 * np.pi / ENCODER_CPR)  # rad/s

print(f"samples   : {len(df)}")
print(f"pos_cmd   : {pos_cmd.min():+.4f} to {pos_cmd.max():+.4f} rad")
print(f"pos_meas  : {pos_meas.min():+.4f} to {pos_meas.max():+.4f} rad")

# Find each constant-command plateau and measure the step INTO it from the
# previous plateau.
cmd_rounded = np.round(pos_cmd, 3)
change_idx = np.where(np.diff(cmd_rounded) != 0)[0] + 1
bounds = np.r_[0, change_idx, len(pos_cmd)]

print("\n-- Step transitions --")
segments = []
for i in range(1, len(bounds) - 1):
    seg_start, seg_end = bounds[i], bounds[i + 1]
    if seg_end - seg_start < 20:
        continue
    cmd_before = cmd_rounded[bounds[i - 1]]
    cmd_after = cmd_rounded[seg_start]
    seg_t = t[seg_start:seg_end]
    seg_pos = pos_meas[seg_start:seg_end]
    m = step_metrics(seg_t, seg_pos, cmd_before, cmd_after)
    if m is None:
        continue
    segments.append((seg_t[0], seg_t[-1], cmd_before, cmd_after, m))
    rt = f"{m['rise_time_s']*1000:.1f} ms" if m["rise_time_s"] is not None else "n/a"
    print(
        f"  {cmd_before:+.3f} -> {cmd_after:+.3f} rad  "
        f"rise={rt}  overshoot={m['overshoot_pct']:.1f}%  "
        f"settle={m['settling_time_s']*1000:.1f} ms"
    )

pos_err_deg = np.degrees(pos_cmd - pos_meas)

fig, (ax_pos, ax_err, ax_vel, ax_iq, ax_vq) = plt.subplots(5, 1, figsize=(12, 15), sharex=True)

ax_pos.plot(t, np.degrees(pos_cmd), "--", color="C1", linewidth=1.4, label="pos_cmd")
ax_pos.plot(t, np.degrees(pos_meas), color="C0", linewidth=1.0, label="pos_meas")
ax_pos.set_ylabel("Position (deg)")
ax_pos.set_title("Position Loop Step Response")
ax_pos.legend()
ax_pos.grid(True, alpha=0.3)

ax_err.plot(t, pos_err_deg, color="C4", linewidth=0.8)
ax_err.axhline(0.0, color="gray", linewidth=0.8, linestyle=":")
ax_err.set_ylabel("Position error (deg)")
ax_err.grid(True, alpha=0.3)

ax_vel.plot(t, vel_cmd, "--", color="C1", linewidth=1.0, label="vel_cmd (P output)")
ax_vel.plot(t, vel_meas, color="C0", linewidth=0.8, label="vel_meas")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.legend(fontsize=8)
ax_vel.grid(True, alpha=0.3)

ax_iq.plot(t, iq_meas, color="C2", linewidth=0.8)
ax_iq.set_ylabel("iq_meas (mA)")
ax_iq.grid(True, alpha=0.3)

ax_vq.plot(t, vq_mV, color="C3", linewidth=0.8)
ax_vq.set_ylabel("vq (mV)")
ax_vq.set_xlabel("Time (s)")
ax_vq.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(out_path, dpi=160)
print(f"\nwrote: {out_path}")
