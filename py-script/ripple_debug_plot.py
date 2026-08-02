#!/usr/bin/env python3
"""Analyze the constant-iq RIPPLE_DEBUG test.

Usage:
    python3 ripple_debug_plot.py drive_data/ripple_debug_log.csv

RIPPLE_DEBUG telemetry mapping:
    sysid_f   -> iq_cmd [mA]
    iq_cmd_mA -> STM32 filtered velocity [counts/s]
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
from scipy.signal import detrend


ENCODER_CPR = 8192.0
POLE_PAIRS = 3
VEL_TELEM_DIV = 8.0  # firmware sends encoder velocity counts/s divided by 8
FLAG_ALIGN = 0
FLAG_RUN = 1
FLAG_IDLE = 2


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def contiguous_regions(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def analyze_segment(t, velocity, start, stop, label):
    # Discard 10% at each end to avoid the current-step transitions.
    margin = max(1, int(0.10 * (stop - start)))
    i0, i1 = start + margin, stop - margin
    ts = t[i0:i1]
    vs = velocity[i0:i1]
    if len(vs) < 16:
        return None

    fs = 1.0 / np.median(np.diff(ts))
    ripple = detrend(vs, type="linear")
    window = np.hanning(len(ripple))
    spectrum = np.fft.rfft(ripple * window)
    freq = np.fft.rfftfreq(len(ripple), d=1.0 / fs)
    amplitude = 2.0 * np.abs(spectrum) / np.sum(window)

    # Ignore DC and frequencies below the resolution needed to separate trend.
    f_min = max(2.0, 2.0 / (ts[-1] - ts[0]))
    valid = (freq >= f_min) & (freq <= min(500.0, 0.45 * fs))
    if np.any(valid):
        k = np.where(valid)[0][np.argmax(amplitude[valid])]
        peak_frequency = freq[k]
        peak_amplitude = amplitude[k]
    else:
        peak_frequency = np.nan
        peak_amplitude = np.nan

    mean_velocity = float(np.mean(vs))
    f_mech = abs(mean_velocity) / (2.0 * np.pi)
    f_elec = POLE_PAIRS * f_mech

    # Fold the ripple into one electrical revolution. Reverse negative motion
    # so the two directions use the same increasing-angle convention.
    theta_seg = np.mod(theta[i0:i1] / 1000.0, 2.0 * np.pi)
    if mean_velocity < 0.0:
        theta_seg = np.mod(2.0 * np.pi - theta_seg, 2.0 * np.pi)

    bin_edges = np.linspace(0.0, 2.0 * np.pi, 73)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])
    bin_index = np.digitize(theta_seg, bin_edges) - 1
    folded = np.full(len(bin_centers), np.nan)
    for b in range(len(bin_centers)):
        values = ripple[bin_index == b]
        if len(values):
            folded[b] = np.mean(values)

    orders = np.arange(1, 13)
    order_amplitude = np.empty_like(orders, dtype=np.float64)
    for j, order in enumerate(orders):
        a = 2.0 * np.mean(ripple * np.cos(order * theta_seg))
        b = 2.0 * np.mean(ripple * np.sin(order * theta_seg))
        order_amplitude[j] = np.hypot(a, b)

    return {
        "label": label,
        "i0": i0,
        "i1": i1,
        "time": ts,
        "ripple": ripple,
        "frequency": freq,
        "amplitude": amplitude,
        "mean_velocity": mean_velocity,
        "ripple_rms": float(np.sqrt(np.mean(ripple ** 2))),
        "ripple_pp": float(np.percentile(ripple, 99) - np.percentile(ripple, 1)),
        "peak_frequency": float(peak_frequency),
        "peak_amplitude": float(peak_amplitude),
        "f_mech": f_mech,
        "f_elec": f_elec,
        "mechanical_order": peak_frequency / f_mech if f_mech > 0 else np.nan,
        "electrical_order": peak_frequency / f_elec if f_elec > 0 else np.nan,
        "angle_deg": np.degrees(bin_centers),
        "folded_ripple": folded,
        "orders": orders,
        "order_amplitude": order_amplitude,
        "order6_amplitude": float(order_amplitude[5]),
    }


if len(sys.argv) != 2:
    print("Usage: python3 ripple_debug_plot.py drive_data/ripple_debug_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "ripple_debug_plot.png"
df = pd.read_csv(csv_path)

required = [
    "host_time_s", "flags", "sysid_f", "iq_cmd_mA", "iq_mA", "id_mA",
    "vq_mV", "vd_mV", "ia_mA", "ib_mA", "theta_mrad",
]
missing = [name for name in required if name not in df.columns]
if missing:
    raise KeyError(f"Missing CSV columns: {', '.join(missing)}")

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]
fs = 1.0 / np.median(np.diff(t))
flags = parse_flags(df["flags"]).to_numpy()
run = flags == FLAG_RUN

# Exact RIPPLE_DEBUG packet decoding.
iq_cmd_mA = df["sysid_f"].to_numpy(dtype=np.float64)
vel_counts_s = df["iq_cmd_mA"].to_numpy(dtype=np.float64) * VEL_TELEM_DIV
velocity = vel_counts_s * (2.0 * np.pi / ENCODER_CPR)

iq_meas = df["iq_mA"].to_numpy(dtype=np.float64)
id_meas = df["id_mA"].to_numpy(dtype=np.float64)
vq_mV = df["vq_mV"].to_numpy(dtype=np.float64)
vd_mV = df["vd_mV"].to_numpy(dtype=np.float64)
ia_mA = df["ia_mA"].to_numpy(dtype=np.float64)
ib_mA = df["ib_mA"].to_numpy(dtype=np.float64)
theta = df["theta_mrad"].to_numpy(dtype=np.float64)

if "encoder_position" in df.columns:
    encoder = df["encoder_position"].to_numpy(dtype=np.float64)
else:
    hi = df["enc_hi"].to_numpy(dtype=np.int64)
    lo = df["enc_lo"].to_numpy(dtype=np.int64) & 0xFFFF
    encoder = ((hi << 16) | lo).astype(np.int32).astype(np.float64)

# Find positive and negative current plateaus separately so a direct
# +iq -> -iq transition cannot be merged into one FFT segment.
segments = []
for plateau_mask, label in [
    (run & (iq_cmd_mA >= 1.0), "+iq"),
    (run & (iq_cmd_mA <= -1.0), "-iq"),
]:
    for start, stop in contiguous_regions(plateau_mask):
        result = analyze_segment(t, velocity, start, stop, label)
        if result is not None:
            segments.append(result)

segments.sort(key=lambda item: item["i0"])

print(f"samples     : {len(df)}")
print(f"sample rate : {fs:.1f} Hz")
print(f"duration    : {t[-1]:.3f} s")
print("\n-- Ripple measurements (inner 80% of each current plateau) --")
for s in segments:
    print(
        f"{s['label']:>3}  mean={s['mean_velocity']:+8.3f} rad/s  "
        f"rms={s['ripple_rms']:.3f} rad/s  "
        f"p98-p2={s['ripple_pp']:.3f} rad/s  "
        f"f_peak={s['peak_frequency']:.2f} Hz  "
        f"mech_order={s['mechanical_order']:.2f}  "
        f"elec_order={s['electrical_order']:.2f}  "
        f"A6={s['order6_amplitude']:.3f} rad/s"
    )

plt.rcParams.update(plt.rcParamsDefault)
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.35,
})

fig = plt.figure(figsize=(14, 19))
gs = gridspec.GridSpec(5, 2, figure=fig, hspace=0.40, wspace=0.28,
                       height_ratios=[1.4, 1.2, 1.1, 1.0, 1.0])
ax_vel = fig.add_subplot(gs[0, :])
ax_ripple = fig.add_subplot(gs[1, 0])
ax_fft = fig.add_subplot(gs[1, 1])
ax_fold = fig.add_subplot(gs[2, 0])
ax_order = fig.add_subplot(gs[2, 1])
ax_iq = fig.add_subplot(gs[3, 0])
ax_id = fig.add_subplot(gs[3, 1])
ax_v = fig.add_subplot(gs[4, 0])
ax_theta = fig.add_subplot(gs[4, 1])

ax_vel.set_title("Constant-iq Ripple Debug", fontsize=12, fontweight="bold")
ax_vel.plot(t, velocity, color="C0", linewidth=1.0, label="STM32 velocity")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.axhline(0, color="black", linewidth=0.6)
ax_cmd = ax_vel.twinx()
ax_cmd.step(t, iq_cmd_mA, where="post", color="C1", linestyle="--",
            linewidth=1.2, label="iq_cmd")
ax_cmd.set_ylabel("iq command (mA)", color="C1")
lines = [line for line in ax_vel.get_lines() + ax_cmd.get_lines()
         if not line.get_label().startswith("_")]
ax_vel.legend(lines, [line.get_label() for line in lines], loc="upper right")

for s in segments:
    ax_ripple.plot(s["time"], s["ripple"], linewidth=1.0, label=s["label"])
    ax_fft.semilogy(s["frequency"], np.maximum(s["amplitude"], 1e-6),
                    linewidth=1.1,
                    label=f"{s['label']}: {s['peak_frequency']:.1f} Hz, "
                          f"order_e={s['electrical_order']:.1f}")
ax_ripple.set_title("Detrended velocity ripple")
ax_ripple.set_xlabel("Time (s)")
ax_ripple.set_ylabel("Ripple (rad/s)")
ax_ripple.legend()
ax_fft.set_title("Velocity-ripple spectrum")
ax_fft.set_xlim(0, min(200, 0.45 * fs))
ax_fft.set_xlabel("Frequency (Hz)")
ax_fft.set_ylabel("Amplitude (rad/s peak)")
ax_fft.legend()

for s in segments:
    ax_fold.plot(s["angle_deg"], s["folded_ripple"], linewidth=1.4,
                 marker=".", markersize=3, label=s["label"])
    ax_order.plot(s["orders"], s["order_amplitude"], marker="o",
                  linewidth=1.2, label=s["label"])
ax_fold.set_title("Velocity ripple folded by electrical angle")
ax_fold.set_xlabel("Direction-normalized electrical angle (deg)")
ax_fold.set_ylabel("Mean ripple (rad/s)")
ax_fold.set_xlim(0, 360)
ax_fold.legend()
ax_order.set_title("Electrical-angle order spectrum")
ax_order.set_xlabel("Electrical order")
ax_order.set_ylabel("Amplitude (rad/s peak)")
ax_order.set_xticks(np.arange(1, 13))
ax_order.axvline(6, color="C3", linestyle="--", linewidth=1.0,
                 alpha=0.7, label="6th order")
ax_order.legend()

ax_iq.set_title("q-axis current")
ax_iq.step(t, iq_cmd_mA, where="post", color="C1", linestyle="--", label="iq_cmd")
ax_iq.plot(t, iq_meas, color="C0", linewidth=0.8, alpha=0.8, label="iq_meas")
ax_iq.set_xlabel("Time (s)")
ax_iq.set_ylabel("Current (mA)")
ax_iq.legend()

ax_id.set_title("d-axis current")
ax_id.plot(t, id_meas, color="C3", linewidth=0.8, label="id_meas")
ax_id.axhline(0, color="black", linewidth=0.6, linestyle="--")
ax_id.set_xlabel("Time (s)")
ax_id.set_ylabel("Current (mA)")
ax_id.legend()

ax_v.set_title("Current-loop voltage output")
ax_v.plot(t, vq_mV, color="C2", linewidth=0.8, label="vq")
ax_v.plot(t, vd_mV, color="C4", linewidth=0.8, label="vd")
ax_v.set_xlabel("Time (s)")
ax_v.set_ylabel("Voltage (mV)")
ax_v.legend()

ax_theta.set_title("Electrical angle")
ax_theta.plot(t, theta, color="C5", linewidth=0.8, label="theta_e")
ax_theta.set_xlabel("Time (s)")
ax_theta.set_ylabel("Angle (mrad)")
ax_theta.legend()

fig.text(
    0.5, 0.005,
    f"source: {csv_path.name} | fs={fs:.0f} Hz | CPR={int(ENCODER_CPR)} | "
    f"sysid_f->iq_cmd(mA), iq_cmd_mA->STM32 velocity(counts/s/{VEL_TELEM_DIV:g})",
    ha="center", fontsize=8, color="dimgray", family="monospace",
)
plt.savefig(out_path, dpi=160, bbox_inches="tight")
print(f"\nwrote: {out_path}")