#!/usr/bin/env python3
"""Closed-loop velocity step response plot.

Usage:
    python3 velocity_step_plot.py drive_data/vel_step_log.csv

CSV column remapping for SYSID_TEST_CL_VEL_STEP:
    sysid_f    → vel_cmd  [mrad/s]   decode: val / 1000.0        → rad/s
    iq_cmd_mA  → vel_meas [counts/s] decode: val * (2π / 8192)   → rad/s

All other columns are unchanged.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ENCODER_CPR    = 8192.0
VEL_TELEM_DIV  = 8.0  # firmware sends vel_meas_counts / VEL_TELEM_DIV as int16
SAVGOL_WINDOW  = 15
SAVGOL_ORDER   = 3

FLAG_ALIGN = 0
FLAG_RUN   = 1
FLAG_IDLE  = 2


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def smooth(x, window=SAVGOL_WINDOW, order=SAVGOL_ORDER):
    n = len(x)
    w = window if window % 2 == 1 else window + 1
    w = min(w, n if n % 2 == 1 else n - 1)
    if w < order + 2:
        return x.copy()
    return savgol_filter(x, w, order)


def step_metrics(t, vel_meas, vel_cmd, run_mask):
    results = {}
    unique_cmds = [c for c in np.unique(vel_cmd) if c != 0.0]
    labels = {c: ("HIGH" if c > 0 else "LOW") for c in unique_cmds}

    for target, label in labels.items():
        mask = run_mask & (vel_cmd == target)
        if not np.any(mask):
            continue
        idx = np.where(mask)[0]
        ts, vs = t[idx], vel_meas[idx]
        if len(vs) < 5:
            continue

        lo = 0.10 * target
        hi = 0.90 * target
        if target > 0:
            cross_lo = np.where(vs >= lo)[0]
            cross_hi = np.where(vs >= hi)[0]
        else:
            cross_lo = np.where(vs <= lo)[0]
            cross_hi = np.where(vs <= hi)[0]

        t_rise = (ts[cross_hi[0]] - ts[cross_lo[0]]) * 1000 \
                 if (len(cross_lo) and len(cross_hi)) else None

        n  = len(vs)
        ss = np.mean(vs[int(0.75 * n):])
        os_pct = 100.0 * (np.max(vs) - target) / abs(target) if target > 0 else \
                 100.0 * (target - np.min(vs)) / abs(target)

        tol     = 0.02 * abs(target)
        outside = np.where(np.abs(vs - target) > tol)[0]
        t_settle = (ts[outside[-1]] - ts[0]) * 1000 if len(outside) else 0.0

        results[label] = dict(target=target, ss=ss,
                              t_rise_ms=t_rise, os_pct=os_pct,
                              t_settle_ms=t_settle)
    return results


# ---------------------------------------------------------------------------
# Load
# ---------------------------------------------------------------------------
if len(sys.argv) != 2:
    print("Usage: python3 velocity_step_plot.py drive_data/vel_step_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "velocity_step_plot.png"

df    = pd.read_csv(csv_path)
flags = parse_flags(df["flags"]).to_numpy()

t  = df["host_time_s"].to_numpy(dtype=np.float64)
t  = t - t[0]
fs = 1.0 / np.median(np.diff(t))

# Decode velocity signals from repurposed slots
vel_cmd_raw  = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0        # mrad/s → rad/s
vel_meas_raw = df["iq_cmd_mA"].to_numpy(dtype=np.float64) * VEL_TELEM_DIV * (2.0 * np.pi / ENCODER_CPR)
vel_meas     = smooth(vel_meas_raw)

# Standard signals
iq_mA = df["iq_mA"].to_numpy(dtype=np.float64)
id_mA = df["id_mA"].to_numpy(dtype=np.float64)
ia_mA = df["ia_mA"].to_numpy(dtype=np.float64)
ib_mA = df["ib_mA"].to_numpy(dtype=np.float64)
vq_mV = df["vq_mV"].to_numpy(dtype=np.float64)
vd_mV = df["vd_mV"].to_numpy(dtype=np.float64)
theta = df["theta_mrad"].to_numpy(dtype=np.float64) if "theta_mrad" in df.columns else None

run_mask = flags == FLAG_RUN
amplitude = float(np.max(np.abs(vel_cmd_raw)))

print(f"samples      : {len(df)}")
print(f"sample rate  : {fs:.1f} Hz")
print(f"duration     : {t[-1]:.3f} s")
print(f"cmd amplitude: ±{amplitude:.1f} rad/s")

metrics = step_metrics(t, vel_meas, vel_cmd_raw, run_mask)

print("\n── Step metrics ─────────────────────────────────────────────────")
for label, m in metrics.items():
    rise = f"{m['t_rise_ms']:.1f} ms" if m['t_rise_ms'] is not None else "n/a"
    print(f"  {label}  target={m['target']:.1f} rad/s  ss={m['ss']:.2f}  "
          f"rise={rise}  OS={m['os_pct']:.1f}%  settle={m['t_settle_ms']:.1f} ms")


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
plt.rcParams.update(plt.rcParamsDefault)
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size":   9,
    "axes.grid":   True,
    "grid.alpha":  0.4,
    "lines.linewidth": 1.4,
})

fig = plt.figure(figsize=(14, 15))
gs  = gridspec.GridSpec(4, 2, figure=fig,
                        hspace=0.40, wspace=0.28,
                        height_ratios=[2.2, 1.1, 1.1, 1.1])

ax_vel = fig.add_subplot(gs[0, :])
ax_err = fig.add_subplot(gs[1, :], sharex=ax_vel)
ax_iq  = fig.add_subplot(gs[2, 0])
ax_vq  = fig.add_subplot(gs[2, 1])
ax_ph  = fig.add_subplot(gs[3, 0])
ax_th  = fig.add_subplot(gs[3, 1])

# Stage shading
stage_colors = {FLAG_ALIGN: "#cce5ff", FLAG_RUN: "#d4f0d4", FLAG_IDLE: "#ffe5cc"}
stage_labels = {FLAG_ALIGN: "ALIGN",   FLAG_RUN: "RUN",     FLAG_IDLE: "IDLE"}

def shade(ax):
    i = 0
    while i < len(flags):
        j = i + 1
        while j < len(flags) and flags[j] == flags[i]:
            j += 1
        ax.axvspan(t[i], t[j - 1], alpha=0.15,
                   color=stage_colors.get(flags[i], "#eee"), linewidth=0)
        i = j

for ax in [ax_vel, ax_err, ax_iq, ax_vq, ax_ph, ax_th]:
    shade(ax)

import matplotlib.patches as mpatches
stage_patches = [mpatches.Patch(color=stage_colors[f], alpha=0.6,
                                label=stage_labels[f])
                 for f in [FLAG_ALIGN, FLAG_RUN, FLAG_IDLE]
                 if f in set(flags)]

# ── Velocity ──────────────────────────────────────────────────────────────────
ax_vel.set_title(f"Closed-Loop Velocity Step Response  (cmd ±{amplitude:.1f} rad/s)",
                 fontsize=11, fontweight="bold", pad=5)
ax_vel.plot(t, vel_meas_raw, color="lightsteelblue", linewidth=0.6,
            alpha=0.5, label="ω meas (raw)")
ax_vel.plot(t, vel_meas, color="C0", linewidth=1.8, label="ω meas (filtered)")
ax_vel.step(t, vel_cmd_raw, color="C1", linewidth=1.5, where="post",
            linestyle="--", label="ω cmd")
ax_vel.axhline(0, color="black", linewidth=0.6, alpha=0.4)
ax_vel.set_ylabel("Velocity  (rad/s)")

for label, m in metrics.items():
    mask = run_mask & (vel_cmd_raw == m["target"])
    if not np.any(mask):
        continue
    t_start = t[np.where(mask)[0][0]]
    parts = []
    if m["t_rise_ms"] is not None:
        parts.append(f"t_rise = {m['t_rise_ms']:.1f} ms")
    parts.append(f"OS = {m['os_pct']:.1f}%")
    parts.append(f"t_settle = {m['t_settle_ms']:.1f} ms")
    ypos = m["target"] * 1.05
    ax_vel.text(t_start + (t[-1] - t[0]) * 0.01, ypos,
                "\n".join(parts), fontsize=7.5, color="C2",
                va="center", fontfamily="monospace",
                bbox=dict(boxstyle="round,pad=0.3", fc="white",
                          ec="C2", alpha=0.85))

h, l = ax_vel.get_legend_handles_labels()
ax_vel.legend(h + stage_patches, l + [p.get_label() for p in stage_patches],
              fontsize=8, loc="upper right", ncol=2)
plt.setp(ax_vel.get_xticklabels(), visible=False)

# ── Error ──────────────────────────────────────────────────────────────────────
ax_err.plot(t, vel_cmd_raw - vel_meas, color="C3", linewidth=1.2,
            label="Error  (ω_cmd − ω_meas)")
ax_err.axhline(0, color="black", linewidth=0.6, alpha=0.4)
ax_err.set_ylabel("Error  (rad/s)")
ax_err.legend(fontsize=8, loc="upper right")
plt.setp(ax_err.get_xticklabels(), visible=False)

# ── dq Currents ────────────────────────────────────────────────────────────────
ax_iq.set_title("dq Currents", fontsize=9, fontweight="bold")
ax_iq.plot(t, iq_mA, color="C0", label="iq_meas  (mA)")
ax_iq.plot(t, id_mA, color="C1", linestyle="--", alpha=0.8, label="id_meas  (mA)")
ax_iq.set_ylabel("Current  (mA)")
ax_iq.set_xlabel("Time  (s)")
ax_iq.legend(fontsize=8)

# ── dq Voltages ────────────────────────────────────────────────────────────────
ax_vq.set_title("dq Voltages  (PI output)", fontsize=9, fontweight="bold")
ax_vq.plot(t, vq_mV, color="C2", label="vq_applied  (mV)")
ax_vq.plot(t, vd_mV, color="C4", linestyle="--", alpha=0.8, label="vd_applied  (mV)")
ax_vq.set_ylabel("Voltage  (mV)")
ax_vq.set_xlabel("Time  (s)")
ax_vq.legend(fontsize=8)

# ── Phase currents ─────────────────────────────────────────────────────────────
ax_ph.set_title("Phase Currents", fontsize=9, fontweight="bold")
ax_ph.plot(t, ia_mA, color="C0", linewidth=0.9, label="ia  (mA)")
ax_ph.plot(t, ib_mA, color="C1", linewidth=0.9, alpha=0.8, label="ib  (mA)")
ax_ph.set_ylabel("Current  (mA)")
ax_ph.set_xlabel("Time  (s)")
ax_ph.legend(fontsize=8)

# ── Theta / encoder ────────────────────────────────────────────────────────────
if theta is not None:
    ax_th.set_title("Electrical Angle", fontsize=9, fontweight="bold")
    ax_th.plot(t, theta, color="C5", linewidth=0.9, label="θ_elec  (mrad)")
    ax_th.set_ylabel("θ  (mrad)")
else:
    enc = df["encoder_position"].to_numpy(dtype=np.float64)
    ax_th.set_title("Encoder Position", fontsize=9, fontweight="bold")
    ax_th.plot(t, enc, color="C5", linewidth=0.9, label="encoder (counts)")
    ax_th.set_ylabel("counts")
ax_th.set_xlabel("Time  (s)")
ax_th.legend(fontsize=8)

fig.text(
    0.5, 0.003,
    (f"source: {csv_path.name}   |   fs≈{fs:.0f} Hz   "
     f"cmd=±{amplitude:.1f} rad/s   CPR={int(ENCODER_CPR)}   "
     f"SG w={SAVGOL_WINDOW} ord={SAVGOL_ORDER}   "
     f"sysid_f→vel_cmd   iq_cmd_mA→vel_meas"),
    ha="center", va="bottom", fontsize=7.5,
    color="dimgray", fontfamily="monospace"
)

plt.savefig(out_path, dpi=160, bbox_inches="tight")
print(f"\nwrote: {out_path}")