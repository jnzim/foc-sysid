#!/usr/bin/env python3

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ENCODER_CPR = 8192.0





if len(sys.argv) != 2:
    print(
        "Usage: python3 plot_vel_position.py "
        "drive_data/sysid_log.csv"
    )
    sys.exit(1)


csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "velocity_chirp_position.png"

df = pd.read_csv(csv_path)

required = [
    "host_time_s",
    "encoder_position",
    "sysid_f",
    "iq_cmd_mA",
    "iq_mA",
    "flags",
]

missing = [name for name in required if name not in df.columns]

if missing:
    raise KeyError(
        f"Missing CSV columns: {', '.join(missing)}"
    )


# Convert hexadecimal strings such as "0x0001" into integers.
def parse_flag(value):
    if isinstance(value, str):
        return int(value, 0)

    return int(value)


flags = df["flags"].map(parse_flag)

# Keep only SYSID_STAGE_RUN.
df = df[flags == 1].copy()

if len(df) == 0:
    raise RuntimeError("No SYSID_STAGE_RUN samples found")


t = df["host_time_s"].to_numpy(
    dtype=np.float64,
    copy=True
)

t -= t[0]

enc_counts = df["encoder_position"].to_numpy(dtype=np.float64)

theta_mech = enc_counts * (
    2.0 * np.pi / ENCODER_CPR
)

iq_cmd = (
    df["iq_cmd_mA"].to_numpy(dtype=np.float64)
    / 1000.0
)

iq_meas = (
    df["iq_mA"].to_numpy(dtype=np.float64)
    / 1000.0
)


fig, (ax_pos, ax_iq) = plt.subplots(
    2,
    1,
    figsize=(13, 8),
    sharex=True
)


# Mechanical position
ax_pos.plot(
    t,
    theta_mech,
    color="blue",
    linewidth=1.0
)

ax_pos.set_ylabel("Position (rad)")
ax_pos.set_title(
    "Mechanical Position During iq Chirp"
)

ax_pos.grid(True)


# Current command and measured current
ax_iq.plot(
    t,
    iq_cmd,
    color="red",
    linewidth=0.9,
    label="iq command"
)

ax_iq.plot(
    t,
    iq_meas,
    color="green",
    linewidth=0.8,
    alpha=0.8,
    label="iq measured"
)

ax_freq = ax_pos.twinx()

ax_freq.plot(
    t,
    df["sysid_f"].to_numpy(dtype=np.float64),
    color="purple",
    linewidth=1.0,
    alpha=0.35,
    label="chirp frequency"
)

ax_freq.set_ylabel("Chirp Frequency (Hz)", color="purple")
ax_freq.tick_params(axis="y", labelcolor="purple")

ax_iq.set_xlabel("Time (s)")
ax_iq.set_ylabel("Current (A)")
ax_iq.legend()
ax_iq.grid(True)


plt.tight_layout()
plt.savefig(out_path, dpi=160)
plt.close(fig)


print(f"samples         : {len(df)}")
print(
    f"encoder counts  : "
    f"{enc_counts.min():.0f} to {enc_counts.max():.0f}"
)
print(
    f"position        : "
    f"{theta_mech.min():.3f} to "
    f"{theta_mech.max():.3f} rad"
)
print(
    f"iq command      : "
    f"{iq_cmd.min():.3f} to {iq_cmd.max():.3f} A"
)
print(
    f"iq measured     : "
    f"{iq_meas.min():.3f} to {iq_meas.max():.3f} A"
)

print(f"wrote: {out_path}")


print(f"command mean:  {np.mean(iq_cmd):+.6f} A")
print(f"measured mean: {np.mean(iq_meas):+.6f} A")
print(f"command range: {iq_cmd.min():+.3f} to {iq_cmd.max():+.3f} A")
print(f"measured range:{iq_meas.min():+.3f} to {iq_meas.max():+.3f} A")