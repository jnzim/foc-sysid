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
from scipy.signal import detrend, medfilt


ENCODER_CPR = 8192.0
POLE_PAIRS = 3
VEL_TELEM_DIV = 8.0  # firmware sends encoder velocity counts/s divided by 8
FLAG_ALIGN = 0
FLAG_RUN = 1
FLAG_IDLE = 2

# Mechanical plant P(s) = omega/iq = K/(tau*s + 1), from bode_vel_plot.py's
# curve_fit against a vel-chirp run. Used to refer velocity-ripple amplitude
# back through the plant to an equivalent current/torque-domain disturbance —
# raw velocity ripple is misleading across different speeds because the
# plant's own ~12 Hz corner attenuates higher-frequency ripple (i.e. faster
# speeds, since ripple frequency = electrical order * f_elec) more than
# slower ones, even when the underlying disturbance is unchanged.
# Update these if you re-run bode_vel_plot.py and get a different fit.
PLANT_K_RAD_S_PER_A = 238.05
PLANT_TAU_S = 12.98e-3


def plant_mag(f_hz):
    """|P(jf)| for the fitted first-order mechanical plant, in rad/s per A."""
    w = 2.0 * np.pi * f_hz
    return PLANT_K_RAD_S_PER_A / np.sqrt(1.0 + (w * PLANT_TAU_S) ** 2)


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def contiguous_regions(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def analyze_segment(t, velocity, id_meas, start, stop, label):
    # Discard 10% at each end to avoid the current-step transitions.
    margin = max(1, int(0.10 * (stop - start)))
    i0, i1 = start + margin, stop - margin
    ts = t[i0:i1]
    vs = velocity[i0:i1]
    if len(vs) < 16:
        return None

    # DC bias in id -- if this grows across runs at different speeds/currents,
    # that points at a current-sensor offset that drifts with current level
    # (thermal/common-mode), not a fixed calibration residual. RMS/p98-p2 are
    # reported separately since a near-zero mean can still sit on a large
    # noise floor -- mean alone would hide that.
    id_seg_mA = id_meas[i0:i1]
    id_mean_mA = float(np.mean(id_seg_mA))
    id_rms_mA  = float(np.sqrt(np.mean((id_seg_mA - id_mean_mA) ** 2)))
    id_pp_mA   = float(np.percentile(id_seg_mA, 99) - np.percentile(id_seg_mA, 1))

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

    def order_amplitude_of(rip, th):
        amp = np.empty_like(orders, dtype=np.float64)
        for j, order in enumerate(orders):
            a = 2.0 * np.mean(rip * np.cos(order * th))
            b = 2.0 * np.mean(rip * np.sin(order * th))
            amp[j] = np.hypot(a, b)
        return amp

    order_amplitude = order_amplitude_of(ripple, theta_seg)

    # Split-half repeatability check: a real, angle-locked disturbance gives
    # the same order-6 amplitude in both halves; noise randomly correlating
    # with cos(6*theta) over a finite window would not. This is what tells
    # us whether the order spectrum is a real signal or a statistical fluke.
    half = len(ripple) // 2
    order_amplitude_h1 = order_amplitude_of(ripple[:half], theta_seg[:half])
    order_amplitude_h2 = order_amplitude_of(ripple[half:], theta_seg[half:])

    # Refer each order's velocity-domain amplitude back through the plant to
    # an equivalent current-domain disturbance, so runs at different speeds
    # (different ripple frequencies) are actually comparable.
    order_freq_hz = orders * f_elec
    order_amplitude_mA = 1000.0 * order_amplitude / plant_mag(order_freq_hz)

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
        "order_amplitude_mA": order_amplitude_mA,
        "order6_amplitude": float(order_amplitude[5]),
        "order6_amplitude_mA": float(order_amplitude_mA[5]),
        "order6_h1": float(order_amplitude_h1[5]),
        "order6_h2": float(order_amplitude_h2[5]),
        # Two failure modes, either one means don't trust this segment:
        #  - halves disagree with each other (not repeatable)
        #  - combined is much smaller than either half (phase cancellation
        #    across the segment -- the reference angle/phase isn't stable,
        #    e.g. a settling transient bleeding into the window)
        "split_half_verdict": (
            "consistent (real signal)"
            if (abs(order_amplitude_h1[5] - order_amplitude_h2[5])
                < 0.35 * max(order_amplitude_h1[5], order_amplitude_h2[5], 1e-9)
                and order_amplitude[5]
                    >= 0.5 * 0.5 * (order_amplitude_h1[5] + order_amplitude_h2[5]))
            else "INCONSISTENT -- likely contaminated/unreliable segment"
        ),
        "id_mean_mA": id_mean_mA,
        "id_rms_mA": id_rms_mA,
        "id_pp_mA": id_pp_mA,
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

# iq_cmd_mA here is the velocity-PI's continuously-computed output (this test
# dispatches to run_cl_vel_step()), not a clean commanded step — it ripples
# across the +-1 mA threshold near zero, which fragments a single steady
# plateau into dozens of spurious slivers. Median-filter it (detection only —
# the actual ripple analysis below still runs on the raw, unsmoothed signal)
# and require a minimum plateau duration to reject what's left.
smooth_win = int(fs * 0.05) | 1  # ~50 ms, forced odd for medfilt
iq_cmd_smooth = medfilt(iq_cmd_mA, kernel_size=max(3, smooth_win))
min_segment_len = int(fs * 0.3)  # 300 ms

# Find positive and negative current plateaus separately so a direct
# +iq -> -iq transition cannot be merged into one FFT segment.
segments = []
for plateau_mask, label in [
    (run & (iq_cmd_smooth >= 1.0), "+iq"),
    (run & (iq_cmd_smooth <= -1.0), "-iq"),
]:
    for start, stop in contiguous_regions(plateau_mask):
        if (stop - start) < min_segment_len:
            continue
        result = analyze_segment(t, velocity, id_meas, start, stop, label)
        # Settle/zero-crossing plateaus give near-zero mean velocity, which
        # makes f_mech/electrical_order divide-by-near-zero garbage — these
        # aren't real ripple-test plateaus, so drop them.
        if result is not None and abs(result["mean_velocity"]) > 1.0:
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
        f"A6={s['order6_amplitude']:.3f} rad/s  "
        f"A6_current={s['order6_amplitude_mA']:.2f} mA (plant-referred)  "
        f"[split-half: 1st={s['order6_h1']:.3f}, 2nd={s['order6_h2']:.3f} rad/s, "
        f"{s['split_half_verdict']}]  "
        f"id_mean={s['id_mean_mA']:+.2f} mA  "
        f"id_rms={s['id_rms_mA']:.2f} mA  "
        f"id_p98-p2={s['id_pp_mA']:.2f} mA  "
        f"(mean growing across runs => sensor offset drift; "
        f"rms/pp is the noise floor, separate from the mean)"
    )

plt.rcParams.update(plt.rcParamsDefault)
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.35,
})

fig = plt.figure(figsize=(14, 22))
gs = gridspec.GridSpec(6, 2, figure=fig, hspace=0.45, wspace=0.28,
                       height_ratios=[1.4, 1.2, 1.1, 1.1, 1.0, 1.0])
ax_vel = fig.add_subplot(gs[0, :])
ax_ripple = fig.add_subplot(gs[1, 0])
ax_fft = fig.add_subplot(gs[1, 1])
ax_fold = fig.add_subplot(gs[2, 0])
ax_order = fig.add_subplot(gs[2, 1])
ax_order_i = fig.add_subplot(gs[3, :])
ax_iq = fig.add_subplot(gs[4, 0])
ax_id = fig.add_subplot(gs[4, 1])
ax_v = fig.add_subplot(gs[5, 0])
ax_theta = fig.add_subplot(gs[5, 1])

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
def dedup_legend(ax, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), **kwargs)


ax_ripple.set_title("Detrended velocity ripple")
ax_ripple.set_xlabel("Time (s)")
ax_ripple.set_ylabel("Ripple (rad/s)")
dedup_legend(ax_ripple)
ax_fft.set_title("Velocity-ripple spectrum")
ax_fft.set_xlim(0, min(200, 0.45 * fs))
ax_fft.set_xlabel("Frequency (Hz)")
ax_fft.set_ylabel("Amplitude (rad/s peak)")
dedup_legend(ax_fft, fontsize=7)

for s in segments:
    ax_fold.plot(s["angle_deg"], s["folded_ripple"], linewidth=1.4,
                 marker=".", markersize=3, label=s["label"])
    ax_order.plot(s["orders"], s["order_amplitude"], marker="o",
                  linewidth=1.2, label=s["label"])
    ax_order_i.plot(s["orders"], s["order_amplitude_mA"], marker="o",
                     linewidth=1.4,
                     label=f"{s['label']} (mean {s['mean_velocity']:+.1f} rad/s)")
ax_fold.set_title("Velocity ripple folded by electrical angle")
ax_fold.set_xlabel("Direction-normalized electrical angle (deg)")
ax_fold.set_ylabel("Mean ripple (rad/s)")
ax_fold.set_xlim(0, 360)
dedup_legend(ax_fold)
ax_order.set_title("Electrical-angle order spectrum")
ax_order.set_xlabel("Electrical order")
ax_order.set_ylabel("Amplitude (rad/s peak)")
ax_order.set_xticks(np.arange(1, 13))
ax_order.axvline(6, color="C3", linestyle="--", linewidth=1.0,
                 alpha=0.7, label="6th order")
dedup_legend(ax_order)

ax_order_i.set_title(
    "Electrical-angle order spectrum, referred through the plant to current "
    "(comparable across speeds)"
)
ax_order_i.set_xlabel("Electrical order")
ax_order_i.set_ylabel("Equivalent current (mA peak)")
ax_order_i.set_xticks(np.arange(1, 13))
ax_order_i.axvline(6, color="C3", linestyle="--", linewidth=1.0,
                   alpha=0.7, label="6th order")
dedup_legend(ax_order_i, fontsize=8)

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