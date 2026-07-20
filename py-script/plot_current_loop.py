#!/usr/bin/env python3
"""Plot measured phase currents for the fixed phase-voltage polarity test.

Usage:
    python3 plot_phase_currents.py path/to/sysid_log.csv
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def parse_flag(value):
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


if len(sys.argv) != 2:
    print("Usage: python3 plot_phase_currents.py path/to/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "phase_current_polarity_plot.png"

df = pd.read_csv(csv_path)

required = ["host_time_s", "ia_mA", "ib_mA"]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing CSV columns: {', '.join(missing)}")

if "flags" in df.columns:
    flags = df["flags"].map(parse_flag)
    run_df = df[flags == 1].copy()
    if len(run_df) > 0:
        df = run_df

df = df.replace([np.inf, -np.inf], np.nan)
df = df.dropna(subset=required).copy()

if len(df) == 0:
    raise RuntimeError("No valid samples remain after filtering")

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]

ia = df["ia_mA"].to_numpy(dtype=np.float64) / 1000.0
ib = df["ib_mA"].to_numpy(dtype=np.float64) / 1000.0
ic = -(ia + ib)

fig, ax = plt.subplots(figsize=(13, 6))

ax.plot(t, ia, label="ia measured", linewidth=1.0)
ax.plot(t, ib, label="ib measured", linewidth=1.0)
ax.plot(t, ic, label="ic reconstructed", linewidth=1.0)

ax.axhline(0.0, color="black", linewidth=0.8)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Phase current (A)")
ax.set_title(
    "Phase-current polarity test: "
    "Va = +0.50 V, Vb = -0.25 V, Vc = -0.25 V"
)
ax.grid(True)
ax.legend()

plt.tight_layout()
plt.savefig(out_path, dpi=160)
plt.close(fig)

print(f"samples : {len(df)}")
print(f"ia mean : {np.mean(ia):.4f} A")
print(f"ib mean : {np.mean(ib):.4f} A")
print(f"ic mean : {np.mean(ic):.4f} A")
print(f"wrote   : {out_path}")
