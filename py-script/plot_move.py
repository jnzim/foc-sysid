#!/usr/bin/env python3
# plot_move.py -- plots move_telem's output. Column names match the
# TelemetryFrame struct fields directly, no relabeling.

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("../drive_data/latest/move_telem.csv")

COUNTS_PER_REV = 8192.0
SCREW_PITCH_MM = 2.0
COUNTS_PER_MM = COUNTS_PER_REV / SCREW_PITCH_MM

t = df["host_time_s"]

print(f"n = {len(df)}")
print(f"duration (s) = {t.iloc[-1] - t.iloc[0]:.3f}")
print(f"drive_state values seen: {sorted(df['drive_state'].unique())}")

fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

axes[0].plot(t, df.pos_cmd / COUNTS_PER_MM, label="pos_cmd")
axes[0].plot(t, df.pos_fbk / COUNTS_PER_MM, label="pos_fbk")
axes[0].set_ylabel("Position (mm)")
axes[0].legend(loc="lower right")
axes[0].grid(alpha=0.3)
axes[0].set_title("move_telem capture")

axes[1].plot(t, df.vel_cmd / COUNTS_PER_MM, label="vel_cmd")
axes[1].plot(t, df.vel_fbk / COUNTS_PER_MM, label="vel_fbk")
axes[1].set_ylabel("Velocity (mm/s)")
axes[1].legend(loc="lower right")
axes[1].grid(alpha=0.3)

axes[2].plot(t, df.iq_cmd, label="iq_cmd (mA)")
axes[2].plot(t, df.i_q_fbk, label="i_q_fbk (mA)", alpha=0.7)
axes[2].set_ylabel("Current (mA)")
axes[2].set_xlabel("time (s)")
axes[2].legend(loc="lower right")
axes[2].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("../drive_data/latest/move_telem_plot.png", dpi=130)
print("saved drive_data/latest/move_telem_plot.png")
