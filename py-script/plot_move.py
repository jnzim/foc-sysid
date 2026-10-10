#!/usr/bin/env python3
# plot_move.py -- plots move_telem's output. Column names match the
# TelemetryFrame struct fields directly, no relabeling.
#
# The time axis is cropped to the move itself: from the first position-loop
# tick (samples_consumed > 0 while SERVO_ON) to the drive returning to IDLE,
# plus a short margin. Uncropped, the ~5s ALIGN hold squeezes a sub-second
# move into a sliver of the plot.
#
# Usage: plot_move.py [csv] [out_png]   (defaults: drive_data/latest/)

import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

csv = sys.argv[1] if len(sys.argv) > 1 else "../drive_data/latest/move_telem.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "../drive_data/latest/move_telem_plot.png"

COUNTS_PER_REV = 8192.0
SCREW_PITCH_MM = 2.0
COUNTS_PER_MM  = COUNTS_PER_REV / SCREW_PITCH_MM
DRIVE_SERVO_ON = 3
MARGIN_MS      = 50

# One frame per STM millisecond: the Pi reads faster than the 1kHz position
# tick, so consecutive frames repeat.
df = pd.read_csv(csv).drop_duplicates("timestamp_ms").reset_index(drop=True)
print(f"n = {len(df)} unique ms, drive_state values seen: {sorted(df.drive_state.unique())}")

moving = df[(df.drive_state == DRIVE_SERVO_ON) & (df.samples_consumed > 0)]
if moving.empty:
    print("no position-loop ticks in this capture -- plotting everything")
    w, t0 = df, df.timestamp_ms.iloc[0]
else:
    t0 = moving.timestamp_ms.iloc[0]
    t1 = moving.timestamp_ms.iloc[-1]
    w  = df[(df.timestamp_ms >= t0 - MARGIN_MS) & (df.timestamp_ms <= t1 + MARGIN_MS)]

t      = w.timestamp_ms - t0                                    # ms from move start

# Following error only while the position loop is running. Before ALIGN
# finishes, traj_enc_offset is still 0 and pos_fbk is the raw count since
# boot -- the offset there is the rotor being pulled into alignment, not an
# error the loop is responsible for.
in_loop = (w.drive_state == DRIVE_SERVO_ON) & (w.samples_consumed > 0)
err_um  = ((w.pos_cmd - w.pos_fbk) / COUNTS_PER_MM * 1000.0).where(in_loop)

if not moving.empty:
    e = (moving.pos_cmd - moving.pos_fbk) / COUNTS_PER_MM * 1000.0
    print(f"move: {len(moving)} ms, {moving.pos_cmd.iloc[0]/COUNTS_PER_MM:.4f} -> "
          f"{moving.pos_cmd.iloc[-1]/COUNTS_PER_MM:.4f} mm")
    print(f"following error: peak {e.abs().max():.2f} um, rms {np.sqrt((e**2).mean()):.2f} um, "
          f"final {e.iloc[-1]:.2f} um")
    print(f"peak vel_cmd {moving.vel_cmd.abs().max()/COUNTS_PER_MM:.3f} mm/s, "
          f"peak vel_fbk {moving.vel_fbk.abs().max()/COUNTS_PER_MM:.3f} mm/s")

fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

axes[0].plot(t, w.pos_cmd / COUNTS_PER_MM, label="pos_cmd (profile)")
axes[0].plot(t, w.pos_fbk / COUNTS_PER_MM, label="pos_fbk", alpha=0.8)
axes[0].set_ylabel("Position (mm)")
axes[0].set_title(f"move_telem capture -- {os.path.basename(csv)}")

axes[1].plot(t, w.vel_cmd / COUNTS_PER_MM, label="vel_cmd")
axes[1].plot(t, w.vel_fbk / COUNTS_PER_MM, label="vel_fbk (encoder boxcar)", alpha=0.8)
axes[1].set_ylabel("Velocity (mm/s)")

# Following error on its own axis and in microns: overlaid on the position
# panel in mm it is invisible. One encoder count is 0.244 um.
axes[2].plot(t, err_um, color="#c0392b", label="pos_cmd - pos_fbk")
axes[2].axhline(0, color="#777777", lw=0.8)
axes[2].set_ylabel("Following error (um)")

axes[3].plot(t, w.iq_cmd, label="iq_cmd")
axes[3].plot(t, w.i_q_fbk, label="i_q_fbk", alpha=0.7)
axes[3].set_ylabel("Current (mA)")
axes[3].set_xlabel("time from move start (ms)")

for ax in axes:
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")

plt.tight_layout()
plt.savefig(out, dpi=130)
print(f"saved {out}")
