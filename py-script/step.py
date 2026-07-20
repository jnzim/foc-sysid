#!/usr/bin/env python3
"""
step.py — closed-loop current step-response plotter

Usage:
    python3 step.py <csv_file>
"""

import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if len(sys.argv) != 2:
    print("Usage: python3 step.py <csv_file>")
    sys.exit(1)

csv_path = Path(sys.argv[1])
df = pd.read_csv(csv_path)

required = ["iq_mA", "vq_mV", "vd_mV", "ia_mA", "ib_mA"]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing required CSV columns: {', '.join(missing)}")

# Scale engineering units.
df["iq_A"] = df["iq_mA"] / 1000.0
df["vq_V"] = df["vq_mV"] / 1000.0
df["vd_V"] = df["vd_mV"] / 1000.0
df["ia_A"] = df["ia_mA"] / 1000.0
df["ib_A"] = df["ib_mA"] / 1000.0

# iq_cmd may be stored directly or in the repurposed sysid_f field.
if "iq_cmd_mA" in df.columns:
    df["iq_cmd_A"] = df["iq_cmd_mA"] / 1000.0
elif "sysid_f" in df.columns:
    df["iq_cmd_A"] = df["sysid_f"] / 1000.0
else:
    raise KeyError("CSV must contain either 'iq_cmd_mA' or 'sysid_f'")

# Time axis.
if "host_time_s" in df.columns:
    t = df["host_time_s"] - df["host_time_s"].iloc[0]
else:
    t = df.index / 10000.0

# Show only the actual current-step portion.
PLOT_START = 0.14
PLOT_END = 0.30

mask = (t >= PLOT_START) & (t <= PLOT_END)
t = t[mask]
df = df.loc[mask]

if df.empty:
    raise ValueError(
        f"No samples found between {PLOT_START:.3f} s and {PLOT_END:.3f} s"
    )

fig = plt.figure(figsize=(12, 9))
fig.suptitle("Closed-Loop Current Step Response", fontsize=14)
gs = gridspec.GridSpec(3, 1, hspace=0.45)

# q-axis current tracking only.
ax0 = fig.add_subplot(gs[0])
ax0.plot(t, df["iq_A"], label="iq_meas", linewidth=0.9)
ax0.plot(
    t,
    df["iq_cmd_A"],
    label="iq_cmd",
    linewidth=1.6,
    linestyle="--",
    zorder=10,
)
ax0.set_ylabel("Current (A)")
ax0.set_title("q-axis Current Tracking")
ax0.legend(loc="upper right")
ax0.grid(True, alpha=0.3)

# PI output.
ax1 = fig.add_subplot(gs[1])
ax1.plot(t, df["vq_V"], label="vq_cmd", linewidth=0.9)
ax1.plot(t, df["vd_V"], label="vd_cmd", linewidth=0.9, alpha=0.7)
ax1.set_ylabel("Voltage (V)")
ax1.set_title("PI Controller Output")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)

# Phase currents.
ax2 = fig.add_subplot(gs[2])
ax2.plot(t, df["ia_A"], label="ia", linewidth=0.9)
ax2.plot(t, df["ib_A"], label="ib", linewidth=0.9)
ax2.set_ylabel("Current (A)")
ax2.set_xlabel("Time (s)")
ax2.set_title("Phase Currents")
ax2.legend(loc="upper right")
ax2.grid(True, alpha=0.3)

for ax in (ax0, ax1, ax2):
    ax.set_xlim(PLOT_START, PLOT_END)

# Save to drive_data directory.
out_path = Path("drive_data") / "step_response.png"
plt.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"Saved {out_path}")
plt.show()