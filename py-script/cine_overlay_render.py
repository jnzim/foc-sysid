#!/usr/bin/env python3
"""Render SYSID_TEST_CINE_SWEEP as a synced scrolling video overlay.

Usage:
    python3 cine_overlay_render.py drive_data/sysid_log.csv

Unlike the other scripts here, this isn't an analysis plot -- it renders an
MP4 of the position/velocity command-vs-measured traces scrolling in real
time, meant to sit side by side (or picture-in-picture) with phone footage
of the motor actually doing the sweep. Video length matches the real
capture duration, so it stays in sync when cut alongside the footage.

Telemetry mapping (same convention as SYSID_TEST_POSITION_STEP):
    sysid_f    -> position command [mrad]
    iq_cmd_mA  -> measured position [mrad]
    enc_hi_raw -> vel_cmd_rad_sec, the position P controller's output [mrad/s]
    enc_lo_raw -> measured velocity [counts/s]
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np
import pandas as pd

ENCODER_CPR = 8192.0
FLAG_RUN = 1
SETTLE_TIME_S = 1.2     # trim the firmware's 1s hold-at-zero before the sweep starts
FPS = 30.0
WINDOW_S = 2.0          # scrolling time window width -- shrunk from 6s now that the
                        # sweep reaches 80Hz; a 6s window there would just be a solid smear
RENDER_HZ = 2000.0      # resample rate for the plotted line -- needs enough resolution
                        # to render an 80Hz sine cleanly, still far lighter than raw ~10kHz link

# Mirrors CINE_SWEEP_* in config.h -- used only to recompute the instantaneous
# swept frequency for the on-screen readout, not for any analysis.
CINE_F_START  = 3.0
CINE_F_END    = 80.0
CINE_DURATION = 20.0


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


if len(sys.argv) != 2:
    print("Usage: python3 cine_overlay_render.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "cine_overlay.mp4"

df = pd.read_csv(csv_path)
required = ["host_time_s", "flags", "sysid_f", "iq_cmd_mA", "dt", "enc_hi_raw", "enc_lo_raw"]
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
    raise RuntimeError(f"Too few samples remain after discarding the first {SETTLE_TIME_S}s settle")

t_raw = df["host_time_s"].to_numpy(dtype=np.float64)
t_raw = t_raw - t_raw[0]
duration = t_raw[-1]

pos_cmd_deg  = np.degrees(df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0)
pos_meas_deg = np.degrees(df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0)
vel_cmd      = df["enc_hi_raw"].to_numpy(dtype=np.float64) / 1000.0                       # rad/s
vel_meas     = df["enc_lo_raw"].to_numpy(dtype=np.float64) * (2.0 * np.pi / ENCODER_CPR)  # rad/s

print(f"duration    : {duration:.1f} s")
print(f"pos_cmd     : {pos_cmd_deg.min():+.1f} to {pos_cmd_deg.max():+.1f} deg")
print(f"frames      : {int(duration * FPS)} @ {FPS:.0f} fps")

# Resample onto a uniform grid -- much lighter to animate than the raw
# ~10kHz telemetry rate, still far more resolution than a <=3Hz sweep needs.
t_grid     = np.arange(0.0, duration, 1.0 / RENDER_HZ)
pos_cmd_g  = np.interp(t_grid, t_raw, pos_cmd_deg)
pos_meas_g = np.interp(t_grid, t_raw, pos_meas_deg)
vel_cmd_g  = np.interp(t_grid, t_raw, vel_cmd)
vel_meas_g = np.interp(t_grid, t_raw, vel_meas)

pos_lim = 1.15 * max(np.abs(pos_cmd_g).max(), np.abs(pos_meas_g).max(), 1e-6)
vel_lim = 1.15 * max(np.abs(vel_cmd_g).max(), np.abs(vel_meas_g).max(), 1e-6)

plt.style.use("dark_background")
fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(12.8, 7.2), dpi=150)
fig.subplots_adjust(hspace=0.35, left=0.09, right=0.97, top=0.90, bottom=0.11)

for ax, lim, ylabel, title in (
    (ax_pos, pos_lim, "Position (deg)", "Closed Position Loop -- Sine Sweep"),
    (ax_vel, vel_lim, "Velocity (rad/s)", None),
):
    ax.set_xlim(-WINDOW_S, 0.0)
    ax.set_ylim(-lim, lim)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    if title:
        ax.set_title(title, fontsize=14)
ax_vel.set_xlabel("Time (s)")

cmd_pos_line,  = ax_pos.plot([], [], "--", color="#ff8a3d", linewidth=1.8, label="commanded")
meas_pos_line, = ax_pos.plot([], [], "-",  color="#4fc3f7", linewidth=2.0, label="measured")
ax_pos.legend(loc="upper left", fontsize=9, framealpha=0.3)

cmd_vel_line,  = ax_vel.plot([], [], "--", color="#ff8a3d", linewidth=1.4)
meas_vel_line, = ax_vel.plot([], [], "-",  color="#4fc3f7", linewidth=1.6)

freq_text = fig.text(0.97, 0.955, "", ha="right", fontsize=11, color="#dddddd")

n_frames = int(duration * FPS)


def render_frame(now):
    lo = now - WINDOW_S
    i0 = np.searchsorted(t_grid, lo)
    i1 = max(i0 + 1, np.searchsorted(t_grid, now))
    xw = t_grid[i0:i1] - now   # relative time, always ending at 0

    cmd_pos_line.set_data(xw, pos_cmd_g[i0:i1])
    meas_pos_line.set_data(xw, pos_meas_g[i0:i1])
    cmd_vel_line.set_data(xw, vel_cmd_g[i0:i1])
    meas_vel_line.set_data(xw, vel_meas_g[i0:i1])

    f_now = CINE_F_START * (CINE_F_END / CINE_F_START) ** min(now / CINE_DURATION, 1.0)
    freq_text.set_text(f"sweep freq: {f_now:.2f} Hz")


writer = FFMpegWriter(fps=FPS, bitrate=4000)
with writer.saving(fig, str(out_path), dpi=150):
    for i in range(n_frames):
        render_frame(i / FPS)
        writer.grab_frame()
        if i % 150 == 0:
            print(f"  frame {i}/{n_frames}")

print(f"\nwrote: {out_path}")
