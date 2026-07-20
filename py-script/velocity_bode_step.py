#!/usr/bin/env python3
"""Mechanical plant Bode plot from a stepped-sine iq test.

Usage:
    python3 velocity_bode_plot.py drive_data/sysid_log.csv

Output:
    velocity_bode_plot.png next to the input CSV

Plant:
    P_mech(s) = omega(s) / iq(s)

For each constant-frequency segment:

1. Discard the first few cycles.
2. Remove the local position trend.
3. Fit sine and cosine coefficients to measured iq and encoder position.
4. Form theta / iq.
5. Multiply by jw to obtain omega / iq.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ENCODER_CPR = 8192.0

# Must match, or be less than, the firmware test configuration.
DISCARD_CYCLES = 3

# Reject segments that contain too little usable data.
MIN_ANALYSIS_CYCLES = 5

# Optional quality threshold based on sinusoidal least-squares fit.
MIN_IQ_FIT_R2 = 0.50
MIN_POSITION_FIT_R2 = 0.50


def require_columns(df, names):
    missing = [name for name in names if name not in df.columns]
    if missing:
        raise KeyError(f"Missing CSV columns: {', '.join(missing)}")


def parse_flags(series):
    """Accept decimal flags or strings such as 0x0001."""

    def parse_one(value):
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    return series.map(parse_one)



def find_constant_frequency_segments(frequency_hz):
    """Return contiguous index ranges having the same positive sysid_f."""

    segments = []

    start = None
    current_frequency = None

    for index, frequency in enumerate(frequency_hz):
        frequency = float(frequency)

        if frequency <= 0.0:
            if start is not None:
                segments.append((start, index, current_frequency))
                start = None
                current_frequency = None

            continue

        if start is None:
            start = index
            current_frequency = frequency
            continue

        if frequency != current_frequency:
            segments.append((start, index, current_frequency))
            start = index
            current_frequency = frequency

    if start is not None:
        segments.append((start, len(frequency_hz), current_frequency))

    return segments


def fit_sinusoid(t, y, frequency_hz, include_trend=False):
    """Least-squares sinusoidal fit at one known frequency.

    Model without trend:
        y = a*cos(wt) + b*sin(wt) + c

    Model with trend:
        y = a*cos(wt) + b*sin(wt) + c + d*t

    Returns complex phasor a - j*b, because:

        Re{(a - j*b) exp(jwt)}
        = a*cos(wt) + b*sin(wt)
    """

    omega = 2.0 * np.pi * frequency_hz
    local_t = t - t[0]

    columns = [
        np.cos(omega * local_t),
        np.sin(omega * local_t),
        np.ones_like(local_t),
    ]

    if include_trend:
        columns.append(local_t)

    matrix = np.column_stack(columns)

    coefficients, _, _, _ = np.linalg.lstsq(
        matrix,
        y,
        rcond=None,
    )

    fitted = matrix @ coefficients

    residual = y - fitted
    total = y - np.mean(y)

    residual_power = np.sum(residual**2)
    total_power = np.sum(total**2)

    if total_power > 0.0:
        r_squared = 1.0 - residual_power / total_power
    else:
        r_squared = 0.0

    phasor = coefficients[0] - 1j * coefficients[1]

    return phasor, fitted, r_squared


if len(sys.argv) != 2:
    print(
        "Usage: python3 velocity_bode_plot.py "
        "drive_data/sysid_log.csv"
    )
    sys.exit(1)


csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "velocity_bode_plot.png"

df = pd.read_csv(csv_path)

require_columns(
    df,
    [
        "host_time_s",
        "dt",
        "enc_hi",
        "enc_lo",
        "sysid_f",
        "iq_cmd_mA",
        "iq_mA",
        "flags",
    ],
)

flags_numeric = parse_flags(df["flags"])

# SYSID_STAGE_RUN = 1.
df = df[
    (flags_numeric == 1)
    & (df["dt"] > 0)
].copy()

if len(df) < 100:
    raise RuntimeError(
        "Too few RUN-stage samples in the CSV"
    )

t_all = df["host_time_s"].to_numpy(dtype=np.float64)

if np.any(np.diff(t_all) <= 0.0):
    raise RuntimeError(
        "host_time_s must be strictly increasing"
    )

enc_counts = df["encoder_position"].to_numpy(dtype=np.float64)

theta_mech = (
    enc_counts
    * (2.0 * np.pi / ENCODER_CPR)
)

iq_cmd = (
    df["iq_cmd_mA"]
    .to_numpy(dtype=np.float64)
    / 1000.0
)

iq_meas = (
    df["iq_mA"]
    .to_numpy(dtype=np.float64)
    / 1000.0
)

sysid_f = df["sysid_f"].to_numpy(dtype=np.float64)

segments = find_constant_frequency_segments(sysid_f)

if not segments:
    raise RuntimeError(
        "No positive constant-frequency segments found"
    )


results = []

print()
print("Stepped-sine analysis")
print("---------------------")

for start, stop, frequency_hz in segments:
    segment_t = t_all[start:stop]
    segment_theta = theta_mech[start:stop]
    segment_iq = iq_meas[start:stop]
    segment_iq_cmd = iq_cmd[start:stop]

    if len(segment_t) < 2:
        continue

    discard_time = DISCARD_CYCLES / frequency_hz
    analysis_start_time = segment_t[0] + discard_time

    keep = segment_t >= analysis_start_time

    segment_t = segment_t[keep]
    segment_theta = segment_theta[keep]
    segment_iq = segment_iq[keep]
    segment_iq_cmd = segment_iq_cmd[keep]

    if len(segment_t) < 10:
        print(
            f"{frequency_hz:7.2f} Hz: skipped, "
            "not enough samples after discard"
        )
        continue

    duration_s = segment_t[-1] - segment_t[0]
    cycles_available = duration_s * frequency_hz

    if cycles_available < MIN_ANALYSIS_CYCLES:
        print(
            f"{frequency_hz:7.2f} Hz: skipped, "
            f"only {cycles_available:.1f} usable cycles"
        )
        continue

    iq_phasor, iq_fit, iq_r2 = fit_sinusoid(
        segment_t,
        segment_iq,
        frequency_hz,
        include_trend=False,
    )

    cmd_phasor, cmd_fit, cmd_r2 = fit_sinusoid(
        segment_t,
        segment_iq_cmd,
        frequency_hz,
        include_trend=False,
    )

    theta_phasor, theta_fit, theta_r2 = fit_sinusoid(
        segment_t,
        segment_theta,
        frequency_hz,
        include_trend=True,
    )

    iq_amplitude = np.abs(iq_phasor)
    theta_amplitude = np.abs(theta_phasor)

    if iq_amplitude < 1.0e-6:
        print(
            f"{frequency_hz:7.2f} Hz: skipped, "
            "measured iq amplitude is nearly zero"
        )
        continue

    h_theta_iq = theta_phasor / iq_phasor

    omega = 2.0 * np.pi * frequency_hz
    h_omega_iq = 1j * omega * h_theta_iq

    magnitude_db = 20.0 * np.log10(
        np.abs(h_omega_iq)
    )

    phase_deg = np.degrees(
        np.angle(h_omega_iq)
    )

    quality_good = (
        iq_r2 >= MIN_IQ_FIT_R2
        and theta_r2 >= MIN_POSITION_FIT_R2
    )

    results.append(
        {
            "frequency_hz": frequency_hz,
            "h_omega_iq": h_omega_iq,
            "magnitude_db": magnitude_db,
            "phase_deg": phase_deg,
            "iq_amplitude_a": iq_amplitude,
            "cmd_amplitude_a": np.abs(cmd_phasor),
            "theta_amplitude_rad": theta_amplitude,
            "iq_r2": iq_r2,
            "cmd_r2": cmd_r2,
            "theta_r2": theta_r2,
            "quality_good": quality_good,
            "cycles": cycles_available,
        }
    )

    quality_text = "good" if quality_good else "weak"

    print(
        f"{frequency_hz:7.2f} Hz | "
        f"|Iq|={iq_amplitude:8.4f} A | "
        f"|Theta|={theta_amplitude:9.5f} rad | "
        f"mag={magnitude_db:8.2f} dB | "
        f"phase={phase_deg:8.2f} deg | "
        f"R2 iq={iq_r2:5.3f} | "
        f"R2 pos={theta_r2:5.3f} | "
        f"{quality_text}"
    )


if not results:
    raise RuntimeError(
        "No valid stepped-sine frequency points were extracted"
    )


results.sort(
    key=lambda result: result["frequency_hz"]
)

frequencies = np.array(
    [result["frequency_hz"] for result in results],
    dtype=np.float64,
)

h_values = np.array(
    [result["h_omega_iq"] for result in results],
    dtype=np.complex128,
)

magnitudes_db = (
    20.0
    * np.log10(np.abs(h_values))
)

phases_deg = np.degrees(
    np.unwrap(np.angle(h_values))
)

iq_r2_values = np.array(
    [result["iq_r2"] for result in results],
    dtype=np.float64,
)

theta_r2_values = np.array(
    [result["theta_r2"] for result in results],
    dtype=np.float64,
)

quality_good = np.array(
    [result["quality_good"] for result in results],
    dtype=bool,
)


fig, axes = plt.subplots(
    3,
    1,
    figsize=(12, 11),
    sharex=True,
)

ax_mag, ax_phase, ax_quality = axes

ax_mag.semilogx(
    frequencies,
    magnitudes_db,
    "o-",
    linewidth=1.5,
    markersize=6,
)

if np.any(~quality_good):
    ax_mag.semilogx(
        frequencies[~quality_good],
        magnitudes_db[~quality_good],
        "x",
        markersize=9,
        label="Weak sinusoidal fit",
    )

ax_mag.set_ylabel("Magnitude (dB rad/s/A)")
ax_mag.set_title(
    "Mechanical Plant: "
    r"$P(j\omega)=\omega(j\omega)/i_q(j\omega)$"
)
ax_mag.grid(True, which="both")
ax_mag.legend(fontsize=9)


ax_phase.semilogx(
    frequencies,
    phases_deg,
    "o-",
    linewidth=1.5,
    markersize=6,
)

if np.any(~quality_good):
    ax_phase.semilogx(
        frequencies[~quality_good],
        phases_deg[~quality_good],
        "x",
        markersize=9,
        label="Weak sinusoidal fit",
    )

ax_phase.set_ylabel("Phase (deg)")
ax_phase.grid(True, which="both")
ax_phase.legend(fontsize=9)


ax_quality.semilogx(
    frequencies,
    iq_r2_values,
    "o-",
    linewidth=1.5,
    markersize=6,
    label="Measured iq fit R²",
)

ax_quality.semilogx(
    frequencies,
    theta_r2_values,
    "o-",
    linewidth=1.5,
    markersize=6,
    label="Position fit R²",
)

ax_quality.axhline(
    MIN_IQ_FIT_R2,
    linestyle="--",
    label="Fit threshold",
)

ax_quality.set_ylabel("Sinusoidal fit R²")
ax_quality.set_xlabel("Frequency (Hz)")
ax_quality.set_ylim(0.0, 1.05)
ax_quality.grid(True, which="both")
ax_quality.legend(fontsize=9)


frequency_min = frequencies.min()
frequency_max = frequencies.max()

ax_quality.set_xlim(
    frequency_min * 0.8,
    frequency_max * 1.25,
)

plt.tight_layout()
plt.savefig(out_path, dpi=160)

print()
print(f"wrote: {out_path}")