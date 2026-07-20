#!/usr/bin/env python3
"""Mechanical plant Bode plot from the closed-current-loop velocity chirp.

Usage:
    python3 velocity_bode_plot.py drive_data/sysid_log.csv

Output:
    velocity_bode_plot.png next to the input CSV

Plant:
    P_mech(s) = omega(s) / iq(s)

The script first estimates encoder position / measured iq, then multiplies the
frequency response by jw. This avoids differentiating quantized encoder samples
in the time domain.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import coherence, csd, welch, detrend
from scipy.optimize import curve_fit


# Match the firmware definition used by encoder_get_position().
ENCODER_CPR = 8192.0

# Welch window length. Two seconds gives 0.5 Hz bin spacing.
WELCH_WINDOW_S = 4.0
COHERENCE_GOOD = 0.5
COHERENCE_STRONG = 0.8

# Seconds to discard from the start of the RUN stage before spectral
# estimation. The first current step excites a broadband mechanical
# transient (position settling toward a new average, decaying oscillation)
# that contaminates the estimate, worst at low frequency. This is a step
# response artifact, not something a higher chirp start frequency avoids.
SETTLE_TIME_S = 15.0


def require_columns(df, names):
    missing = [name for name in names if name not in df.columns]
    if missing:
        raise KeyError(f"Missing CSV columns: {', '.join(missing)}")


def parse_flags(series):
    """Accept either decimal flags or strings such as 0x0001."""
    return series.map(
        lambda value: int(str(value), 0)
        if isinstance(value, str)
        else int(value)
    )


def first_order_complex(f, K, tau):
    """P(s) = K / (tau*s + 1), evaluated at s = j*2*pi*f.

    Returns a stacked [real..., imag...] vector so curve_fit can operate on
    a real-valued residual while fitting a complex-valued model.
    """
    w = 2.0 * np.pi * f
    H = K / (1j * w * tau + 1.0)
    return np.concatenate([H.real, H.imag])


if len(sys.argv) != 2:
    print("Usage: python3 velocity_bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "velocity_bode_plot.png"

df = pd.read_csv(csv_path)
require_columns(
    df,
    [
        "host_time_s",
        "dt",
        "encoder_position",
        "sysid_f",
        "iq_cmd_mA",
        "iq_mA",
        "flags",
    ],
)

# SYSID_STAGE_RUN is encoded as 1 in the firmware.
flags_numeric = parse_flags(df["flags"])
df = df[(flags_numeric == 1) & (df["dt"] > 0)].copy()
df = df[df["sysid_f"] > 0].copy()

if len(df) < 100:
    raise RuntimeError("Too few RUN-stage velocity-chirp samples in the CSV")

# Discard the startup settling transient (see SETTLE_TIME_S above) before
# doing anything else with this data, so it never enters t/dt/fs or the
# spectral estimate.
run_start_time = df["host_time_s"].to_numpy(dtype=np.float64)[0]
df = df[df["host_time_s"] >= run_start_time + SETTLE_TIME_S].copy()

if len(df) < 100:
    raise RuntimeError(
        f"Too few samples remain after discarding the first {SETTLE_TIME_S}s "
        "settling transient -- lengthen the chirp capture or reduce SETTLE_TIME_S"
    )

t = df["host_time_s"].to_numpy(dtype=np.float64)
t = t - t[0]

dt = np.diff(t)
if np.any(dt <= 0):
    raise RuntimeError("host_time_s must be strictly increasing after filtering")

fs = 1.0 / np.median(dt)

enc_counts = df["encoder_position"].to_numpy(dtype=np.float64)
theta_mech = enc_counts * (2.0 * np.pi / ENCODER_CPR)

iq_cmd = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0
iq_meas = df["iq_mA"].to_numpy(dtype=np.float64) / 1000.0

# Remove offsets AND any linear drift (rotor may not be perfectly centered /
# may free-spin slowly on top of the chirp) before estimating the frequency
# response. De-meaning alone leaves a ramp in place, which concentrates huge,
# run-dependent power at low frequency and can bias or flip the measured
# sign of the cross-spectrum.
theta_ac = detrend(theta_mech, type="linear")
iq_ac = detrend(iq_cmd, type="linear")

nperseg = min(len(df), max(256, int(fs * WELCH_WINDOW_S)))

# scipy.signal.csd(x, y) returns conj(X) * Y. Therefore this is theta / iq.
f_csd, S_iq_theta = csd(iq_ac, theta_ac, fs=fs, nperseg=nperseg)
_, S_iq_iq = welch(iq_ac, fs=fs, nperseg=nperseg)
f_coh, coh = coherence(iq_ac, theta_ac, fs=fs, nperseg=nperseg)

H_theta_iq = S_iq_theta / (S_iq_iq + 1e-30)
H_omega_iq = 1j * 2.0 * np.pi * f_csd * H_theta_iq


mag_db = 20.0 * np.log10(np.abs(H_omega_iq) + 1e-30)
phase_deg = np.degrees(np.unwrap(np.angle(H_omega_iq)))
phase_deg = ((phase_deg + 180) % 360) - 180

chirp_f_min = max(0.1, float(df["sysid_f"].min()))
chirp_f_max = float(df["sysid_f"].max())

band = (
    (f_csd >= chirp_f_min)
    & (f_csd <= chirp_f_max)
    & np.isfinite(mag_db)
    & np.isfinite(phase_deg)
)
good = band & (coh >= COHERENCE_GOOD)
strong = band & (coh >= COHERENCE_STRONG)

if not np.any(good):
    raise RuntimeError("No frequency bins have coherence >= 0.5")

print(f"samples       : {len(df)}")
print(f"sample rate   : {fs:.1f} Hz")
print(f"chirp range   : {chirp_f_min:.1f} to {chirp_f_max:.1f} Hz")
print(f"iq command    : {iq_cmd.min():.3f} to {iq_cmd.max():.3f} A")
print(f"iq measured   : {iq_meas.min():.3f} to {iq_meas.max():.3f} A")
print(f"encoder travel: {enc_counts.min():.0f} to {enc_counts.max():.0f} counts")

# ---------------------------------------------------------------------------
# Fit P(s) = K / (tau*s + 1) against the complex frequency response, using
# only the coherence>=0.5 ("good") points. Fitting real+imag jointly (rather
# than reading corner freq off the -45deg phase point) is required here since
# the sweep only reaches ~-40deg, so the true corner is likely above 10 Hz --
# treat tau as extrapolated, not directly observed.
# ---------------------------------------------------------------------------
f_fit = f_csd[good]
H_fit = H_omega_iq[good]
y_fit = np.concatenate([H_fit.real, H_fit.imag])

# Initial guess: K from low-frequency magnitude, tau from a generous guess
# above the swept range since we never see -45 deg.
K0 = float(np.abs(H_fit[np.argmin(f_fit)]))
tau0 = 1.0 / (2.0 * np.pi * 20.0)  # guess corner near 20 Hz

try:
    popt, pcov = curve_fit(
        first_order_complex, f_fit, y_fit, p0=[K0, tau0], maxfev=20000
    )
    K_fit, tau_fit = popt
    fc_fit = 1.0 / (2.0 * np.pi * tau_fit)
    perr = np.sqrt(np.diag(pcov))
    print("\nFirst-order fit: P(s) = K / (tau*s + 1)")
    print(f"  K   = {K_fit:.2f} rad/s per A   (+/- {perr[0]:.2f})")
    print(f"  tau = {tau_fit * 1000.0:.2f} ms   (+/- {perr[1] * 1000.0:.2f} ms)")
    print(f"  fc  = {fc_fit:.2f} Hz  <-- check against swept range (max {chirp_f_max:.1f} Hz)")
    fit_ok = True
except RuntimeError as exc:
    print(f"\nFit failed: {exc}")
    fit_ok = False

fig, (ax_mag, ax_phase, ax_coh) = plt.subplots(3, 1, figsize=(12, 11), sharex=True)

mag_good = np.where(good, mag_db, np.nan)
mag_strong = np.where(strong, mag_db, np.nan)

ax_mag.semilogx(
    f_csd,
    mag_good,
    "c.-",
    linewidth=1.0,
    markersize=4,
    alpha=0.5,
    label="Measured, coherence >= 0.5",
)
ax_mag.semilogx(
    f_csd,
    mag_strong,
    "bo-",
    linewidth=1.5,
    markersize=5,
    label="Measured, coherence >= 0.8",
)

phase_good = np.where(good, phase_deg, np.nan)
phase_strong = np.where(strong, phase_deg, np.nan)

if fit_ok:
    f_model = np.logspace(np.log10(f_csd[band].min()), np.log10(f_csd[band].max() * 3), 300)
    H_model = K_fit / (1j * 2.0 * np.pi * f_model * tau_fit + 1.0)
    mag_model_db = 20.0 * np.log10(np.abs(H_model))
    phase_model_deg = np.degrees(np.angle(H_model))

    ax_mag.semilogx(
        f_model, mag_model_db, "g-", linewidth=1.5,
        label=f"Fit: K={K_fit:.0f}, tau={tau_fit*1000:.1f}ms, fc={fc_fit:.1f}Hz",
    )
    ax_phase.semilogx(f_model, phase_model_deg, "g-", linewidth=1.5, label="Fit")

ax_mag.set_ylabel("Magnitude (dB rad/s/A)")
ax_mag.set_title("Mechanical Plant Bode: $P(s)=\\omega(s)/i_q(s)$")
ax_mag.legend(fontsize=9)
ax_mag.grid(True, which="both")

ax_phase.semilogx(
    f_csd,
    phase_good,
    "c.-",
    linewidth=1.0,
    markersize=4,
    alpha=0.5,
    label="Measured, coherence >= 0.5",
)
ax_phase.semilogx(
    f_csd,
    phase_strong,
    "bo-",
    linewidth=1.5,
    markersize=5,
    label="Measured, coherence >= 0.8",
)
ax_phase.set_ylabel("Phase (deg)")
ax_phase.legend(fontsize=9)
ax_phase.grid(True, which="both")

ax_coh.semilogx(f_coh[band], coh[band], "g.-", label="Coherence")
ax_coh.axhline(COHERENCE_GOOD, color="red", linestyle="--", label="0.5 threshold")
ax_coh.axhline(COHERENCE_STRONG, color="orange", linestyle="--", label="0.8 threshold")
ax_coh.set_ylabel("Coherence")
ax_coh.set_xlabel("Frequency (Hz)")
ax_coh.set_ylim(0.0, 1.05)

ax_coh.legend(fontsize=9)
ax_coh.grid(True, which="both")

plt.tight_layout()
plt.savefig(out_path, dpi=160)
print(f"wrote: {out_path}")