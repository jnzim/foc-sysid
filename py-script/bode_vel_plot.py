#!/usr/bin/env python3
"""Mechanical plant Bode plot + velocity PI controller design.

Usage:
    python3 velocity_bode_plot.py drive_data/sysid_log.csv

Output:
    velocity_bode_plot.png  — plant characterisation (K, B) + PI design

Plant model (first-order mechanical):
    P_mech(s) = omega(s) / iq(s) = K / (tau*s + 1)

    where:
        K   = steady-state gain      [rad/s per A]  ≈ kt / B_visc
        tau = mechanical time const  [s]            ≈ J / B_visc
        B   = 1/K                    [A·s/rad]      proxy for viscous friction
              (current needed to hold 1 rad/s against back-drag)

PI controller design:
    C(s) = Kp * (1 + 1/(Ti*s))  =  Kp * (Ti*s + 1) / (Ti*s)

    Strategy – integrator + zero cancellation:
        1. Place PI zero at the plant pole:  Ti = tau  →  L(s) = K*Kp / (Ti*s)
           Open loop becomes a pure integrator, giving –20 dB/dec rolloff through
           crossover, guaranteed 90° PM before lag effects.
        2. Set Kp so that crossover frequency matches BW_target (50 Hz):
           |L(j*wc)| = 1  →  Kp = wc*Ti / K
        3. Bandwidth target = 10x less than closed current-loop BW (500 Hz assumed).
           Gain and phase margins are computed numerically over a fine frequency grid.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
from scipy.signal import coherence, csd, welch, detrend
from scipy.optimize import curve_fit


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ENCODER_CPR      = 8192.0
WELCH_WINDOW_S   = 4.0
COHERENCE_GOOD   = 0.5
COHERENCE_STRONG = 0.8
SETTLE_TIME_S    = 15.0

CURRENT_LOOP_BW_HZ = 500.0
BW_TARGET_HZ       = CURRENT_LOOP_BW_HZ / 10.0   # 50 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def require_columns(df, names):
    missing = [n for n in names if n not in df.columns]
    if missing:
        raise KeyError(f"Missing CSV columns: {', '.join(missing)}")


def parse_flags(series):
    return series.map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))


def first_order_complex(f, K, tau):
    w = 2.0 * np.pi * f
    H = K / (1j * w * tau + 1.0)
    return np.concatenate([H.real, H.imag])


def crossover_freq(f_hz, mag_db):
    for i in range(len(mag_db) - 1):
        if mag_db[i] >= 0.0 > mag_db[i + 1]:
            t = mag_db[i] / (mag_db[i] - mag_db[i + 1])
            return f_hz[i] * (f_hz[i + 1] / f_hz[i]) ** t
    return None


def phase_margin_at(f_hz, mag_db, phase_deg):
    fc = crossover_freq(f_hz, mag_db)
    if fc is None:
        return None, None
    phase_at_fc = np.interp(fc, f_hz, phase_deg)
    pm = 180.0 + phase_at_fc
    return fc, pm


def gain_margin_info(f_hz, phase_deg, mag_db):
    for i in range(len(phase_deg) - 1):
        if phase_deg[i] >= -180.0 > phase_deg[i + 1]:
            t = (phase_deg[i] + 180.0) / (phase_deg[i] - phase_deg[i + 1])
            f_pc = f_hz[i] * (f_hz[i + 1] / f_hz[i]) ** t
            mag_pc = np.interp(f_pc, f_hz, mag_db)
            gm = -mag_pc
            return f_pc, gm
    return None, None


# ---------------------------------------------------------------------------
# Load & filter
# ---------------------------------------------------------------------------
if len(sys.argv) != 2:
    print("Usage: python3 velocity_bode_plot.py drive_data/sysid_log.csv")
    sys.exit(1)

csv_path = Path(sys.argv[1]).resolve()
out_path = csv_path.parent / "velocity_bode_plot.png"

df = pd.read_csv(csv_path)
require_columns(df, ["host_time_s", "dt", "encoder_position",
                     "sysid_f", "iq_cmd_mA", "iq_mA", "flags"])

flags_numeric = parse_flags(df["flags"])
df = df[(flags_numeric == 1) & (df["dt"] > 0)].copy()
df = df[df["sysid_f"] > 0].copy()
if len(df) < 100:
    raise RuntimeError("Too few RUN-stage velocity-chirp samples in the CSV")

run_start_time = df["host_time_s"].to_numpy(dtype=np.float64)[0]
df = df[df["host_time_s"] >= run_start_time + SETTLE_TIME_S].copy()
if len(df) < 100:
    raise RuntimeError(
        f"Too few samples remain after discarding the first {SETTLE_TIME_S}s "
        "settling transient — lengthen the chirp or reduce SETTLE_TIME_S"
    )

t        = df["host_time_s"].to_numpy(dtype=np.float64)
t        = t - t[0]
dt_arr   = np.diff(t)
if np.any(dt_arr <= 0):
    raise RuntimeError("host_time_s must be strictly increasing after filtering")
fs = 1.0 / np.median(dt_arr)

enc_counts = df["encoder_position"].to_numpy(dtype=np.float64)
theta_mech = enc_counts * (2.0 * np.pi / ENCODER_CPR)
iq_cmd     = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0
iq_meas    = df["iq_mA"].to_numpy(dtype=np.float64)  / 1000.0

theta_ac = detrend(theta_mech, type="linear")
iq_ac    = detrend(iq_cmd,    type="linear")

nperseg  = min(len(df), max(256, int(fs * WELCH_WINDOW_S)))

f_csd, S_iq_theta = csd(iq_ac, theta_ac, fs=fs, nperseg=nperseg)
_,     S_iq_iq    = welch(iq_ac,          fs=fs, nperseg=nperseg)
f_coh, coh        = coherence(iq_ac, theta_ac, fs=fs, nperseg=nperseg)

H_theta_iq = S_iq_theta / (S_iq_iq + 1e-30)
H_omega_iq = 1j * 2.0 * np.pi * f_csd * H_theta_iq

mag_db    = 20.0 * np.log10(np.abs(H_omega_iq) + 1e-30)
phase_deg = np.degrees(np.unwrap(np.angle(H_omega_iq)))
phase_deg = ((phase_deg + 180) % 360) - 180

chirp_f_min = max(0.1, float(df["sysid_f"].min()))
chirp_f_max = float(df["sysid_f"].max())

band   = ((f_csd >= chirp_f_min) & (f_csd <= chirp_f_max)
          & np.isfinite(mag_db) & np.isfinite(phase_deg))
good   = band & (coh >= COHERENCE_GOOD)
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
# First-order fit:  P(s) = K / (tau*s + 1)
# ---------------------------------------------------------------------------
f_fit = f_csd[good]
H_fit = H_omega_iq[good]
y_fit = np.concatenate([H_fit.real, H_fit.imag])

K0   = float(np.abs(H_fit[np.argmin(f_fit)]))
tau0 = 1.0 / (2.0 * np.pi * 20.0)

try:
    popt, pcov = curve_fit(first_order_complex, f_fit, y_fit,
                           p0=[K0, tau0], maxfev=20_000)
    K_fit, tau_fit = popt
    fc_plant = 1.0 / (2.0 * np.pi * abs(tau_fit))
    perr     = np.sqrt(np.diag(pcov))
    B_fit    = 1.0 / K_fit
    fit_ok   = True
except RuntimeError as exc:
    print(f"\nFit failed: {exc}")
    fit_ok = False
    K_fit, tau_fit, fc_plant, B_fit = 1.0, 0.01, 10.0, 1.0

print("\n── Plant identification ──────────────────────────────────────────")
print(f"  K (gain)    = {K_fit:.3f}  rad/s per A")
print(f"  tau         = {tau_fit*1000:.2f} ms  →  fc_plant = {fc_plant:.2f} Hz")
print(f"  B (1/K)     = {B_fit*1000:.4f} mA·s/rad  (viscous friction proxy)")


# ---------------------------------------------------------------------------
# PI controller design
# ---------------------------------------------------------------------------
wc_target = 2.0 * np.pi * BW_TARGET_HZ
Ti_pi     = abs(tau_fit)
Kp_pi     = (wc_target * Ti_pi) / K_fit

print("\n── PI controller design ─────────────────────────────────────────")
print(f"  BW target   = {BW_TARGET_HZ:.0f} Hz  (current loop BW {CURRENT_LOOP_BW_HZ:.0f} Hz / 10)")
print(f"  Ti          = tau_fit = {Ti_pi*1000:.2f} ms  (zero cancels plant pole)")
print(f"  Kp          = {Kp_pi:.4f}  A / (rad/s)")
print(f"  Ki          = Kp/Ti  = {Kp_pi/Ti_pi:.4f}  A / rad")

f_ctrl  = np.logspace(-1, np.log10(min(fs / 2, 1000)), 4000)
w_ctrl  = 2.0 * np.pi * f_ctrl
s       = 1j * w_ctrl

P_s = K_fit / (tau_fit * s + 1.0)
C_s = Kp_pi * (Ti_pi * s + 1.0) / (Ti_pi * s)
L_s = C_s * P_s

L_mag_db    = 20.0 * np.log10(np.abs(L_s) + 1e-30)
L_phase_deg = np.degrees(np.unwrap(np.angle(L_s)))

fc_ol, pm   = phase_margin_at(f_ctrl, L_mag_db, L_phase_deg)
f_pc, gm_db = gain_margin_info(f_ctrl, L_phase_deg, L_mag_db)

print(f"\n── Loop stability margins ───────────────────────────────────────")
if fc_ol is not None:
    print(f"  Crossover   = {fc_ol:.1f} Hz   (target {BW_TARGET_HZ:.0f} Hz)")
    print(f"  Phase margin= {pm:.1f}°        (target ≥ 45°, ideal ~60°)")
else:
    print("  No 0-dB crossover found — check K_fit sign")

if f_pc is not None:
    print(f"  Phase cross = {f_pc:.1f} Hz")
    print(f"  Gain margin = {gm_db:.1f} dB   (target ≥ 6 dB)")
else:
    print("  Phase never reaches -180 deg in swept range -- GM >> 6 dB (good)")


# ---------------------------------------------------------------------------
# Model curves for plant subplot
# ---------------------------------------------------------------------------
f_model = np.logspace(np.log10(max(f_csd[band].min(), 0.1)),
                      np.log10(f_csd[band].max() * 3), 600)
H_model = K_fit / (1j * 2.0 * np.pi * f_model * tau_fit + 1.0)
mag_model_db    = 20.0 * np.log10(np.abs(H_model))
phase_model_deg = np.degrees(np.angle(H_model))


# ---------------------------------------------------------------------------
# Figure — clean white engineering style
# ---------------------------------------------------------------------------
plt.rcParams.update(plt.rcParamsDefault)   # reset to matplotlib defaults
plt.rcParams.update({
    "font.family":  "sans-serif",
    "font.size":    9,
    "axes.grid":    True,
    "grid.alpha":   0.4,
    "lines.linewidth": 1.5,
})

# Color palette: straightforward engineering defaults
C_MEAS_WEAK   = "steelblue"       # low-coherence data
C_MEAS_STRONG = "C0"              # high-coherence data  (matplotlib blue)
C_FIT         = "C1"              # fit / model          (matplotlib orange)
C_PLANT_POLE  = "C3"              # plant pole marker    (red)
C_CROSSOVER   = "C2"              # crossover marker     (green)
C_GM          = "C3"              # gain margin          (red)
C_THRESHOLD   = "gray"

fig = plt.figure(figsize=(14, 20))

gs = gridspec.GridSpec(
    7, 1,
    figure=fig,
    height_ratios=[1.4, 1.6, 1.1, 0.9, 0.9, 1.6, 1.1],
    hspace=0.15,
)

ax_doc    = fig.add_subplot(gs[0])
ax_pmag   = fig.add_subplot(gs[1])
ax_pphase = fig.add_subplot(gs[2], sharex=ax_pmag)
ax_coh    = fig.add_subplot(gs[3], sharex=ax_pmag)
ax_notes  = fig.add_subplot(gs[4])
ax_lmag   = fig.add_subplot(gs[5])
ax_lphase = fig.add_subplot(gs[6], sharex=ax_lmag)


def style_ax(ax, ylabel, title=None):
    ax.set_ylabel(ylabel, fontsize=9)
    if title:
        ax.set_title(title, fontsize=10, fontweight="bold", pad=5)
    ax.grid(True, which="both", alpha=0.4)


# ── Documentation block ──────────────────────────────────────────────────────
ax_doc.axis("off")

ax_doc.text(0.5, 0.97,
            "Velocity Loop System Identification & PI Controller Design",
            transform=ax_doc.transAxes, ha="center", va="top",
            fontsize=13, fontweight="bold")

ax_doc.axhline(0.78, xmin=0.02, xmax=0.98,   # won't render on axis("off")
               color="black", linewidth=0.8)   # kept for reference; use text separator

col_x = [0.03, 0.36, 0.68]
col_titles = ["Plant Identification", "PI Controller Design", "Stability Margins"]

if fit_ok:
    plant_lines = [
        "Model:  P(s) = K / (τs + 1)",
        "",
        f"K  = {K_fit:.3f} rad/s per A",
        f"     (≈ kt / B_visc)",
        "",
        f"B  = 1/K = {B_fit*1000:.4f} mA·s/rad",
        f"     (viscous friction proxy)",
        "",
        f"τ  = {tau_fit*1000:.2f} ms",
        f"fc = {fc_plant:.2f} Hz  (mech. pole)",
    ]
else:
    plant_lines = ["Fit did not converge", "Check input data"]

pi_lines = [
    "C(s) = Kp·(Ti·s + 1)/(Ti·s)",
    "",
    "Strategy: zero @ plant pole",
    f"  Ti  = τ = {Ti_pi*1000:.2f} ms",
    "  → L(s) ≈ K·Kp / (Ti·s)",
    "     pure integrator thru xover",
    "",
    f"BW target = {BW_TARGET_HZ:.0f} Hz",
    f"  Kp  = wc·Ti/K = {Kp_pi:.4f} A/(rad/s)",
    f"  Ki  = Kp/Ti   = {Kp_pi/Ti_pi:.4f} A/rad",
]

if fc_ol is not None and f_pc is not None:
    gm_str = f"{gm_db:.1f} dB"
    margin_lines = [
        "Open-loop crossover:",
        f"  fc = {fc_ol:.1f} Hz",
        f"     (target {BW_TARGET_HZ:.0f} Hz)",
        "",
        "Phase margin:",
        f"  PM = {pm:.1f}°",
        "     (target ≥ 45°, ideal 60°)",
        "",
        "Gain margin:",
        f"  GM = {gm_str}",
        "     (target ≥ 6 dB)",
    ]
elif fc_ol is not None:
    margin_lines = [
        f"Crossover:  {fc_ol:.1f} Hz",
        f"Phase margin: {pm:.1f}°",
        "",
        "Phase never reaches -180°",
        "in swept range  GM >> 6 dB",
    ]
else:
    margin_lines = ["No crossover found", "Check sign of K"]

for col, header, lines in zip(col_x, col_titles, [plant_lines, pi_lines, margin_lines]):
    ax_doc.text(col, 0.88, header,
                transform=ax_doc.transAxes, ha="left", va="top",
                fontsize=9, fontweight="bold")
    ax_doc.text(col, 0.78, "\n".join(lines),
                transform=ax_doc.transAxes, ha="left", va="top",
                fontsize=8, fontfamily="monospace", linespacing=1.5)

# horizontal rule via a thin line artist
ax_doc.plot([0.01, 0.99], [0.92, 0.92], transform=ax_doc.transAxes,
            color="black", linewidth=0.8, clip_on=False)

ax_doc.set_xlim(0, 1)
ax_doc.set_ylim(0, 1)


# ── Plant magnitude ──────────────────────────────────────────────────────────
mag_good   = np.where(good,   mag_db, np.nan)
mag_strong = np.where(strong, mag_db, np.nan)

style_ax(ax_pmag, "Magnitude  (dB  ·  rad/s / A)",
         title="Plant  P(s) = ω(s) / iq(s)  —  Measured vs First-Order Fit")

ax_pmag.semilogx(f_csd, mag_good, color=C_MEAS_WEAK, linewidth=0.8,
                 linestyle="-", marker=".", markersize=3, alpha=0.5,
                 label=f"Measured  coh ≥ {COHERENCE_GOOD}")
ax_pmag.semilogx(f_csd, mag_strong, color=C_MEAS_STRONG, linewidth=1.5,
                 linestyle="-", marker="o", markersize=3,
                 label=f"Measured  coh ≥ {COHERENCE_STRONG}")
if fit_ok:
    ax_pmag.semilogx(f_model, mag_model_db, color=C_FIT, linewidth=2.0,
                     label=f"Fit  K={K_fit:.1f} rad/s/A  τ={tau_fit*1000:.1f}ms  fc={fc_plant:.1f}Hz")
    ax_pmag.axvline(fc_plant, color=C_PLANT_POLE, linewidth=1.0,
                    linestyle="--", alpha=0.8, label=f"Plant pole {fc_plant:.1f} Hz")

ax_pmag.axvspan(chirp_f_min, chirp_f_max, alpha=0.06, color="steelblue")
ax_pmag.legend(fontsize=8, loc="upper right")
plt.setp(ax_pmag.get_xticklabels(), visible=False)


# ── Plant phase ───────────────────────────────────────────────────────────────
phase_good   = np.where(good,   phase_deg, np.nan)
phase_strong = np.where(strong, phase_deg, np.nan)

style_ax(ax_pphase, "Phase  (deg)")
ax_pphase.semilogx(f_csd, phase_good, color=C_MEAS_WEAK, linewidth=0.8,
                   linestyle="-", marker=".", markersize=3, alpha=0.5)
ax_pphase.semilogx(f_csd, phase_strong, color=C_MEAS_STRONG, linewidth=1.5,
                   linestyle="-", marker="o", markersize=3)
if fit_ok:
    ax_pphase.semilogx(f_model, phase_model_deg, color=C_FIT, linewidth=2.0,
                       label="Fit")
    ax_pphase.axvline(fc_plant, color=C_PLANT_POLE, linewidth=1.0,
                      linestyle="--", alpha=0.8)

ax_pphase.axhline(-45, color=C_THRESHOLD, linewidth=0.8, linestyle=":",
                  label="−45°")
ax_pphase.legend(fontsize=8)
ax_pphase.axvspan(chirp_f_min, chirp_f_max, alpha=0.06, color="steelblue")
plt.setp(ax_pphase.get_xticklabels(), visible=False)


# ── Coherence ─────────────────────────────────────────────────────────────────
style_ax(ax_coh, "Coherence  γ²")
ax_coh.semilogx(f_coh[band], coh[band], color=C_MEAS_STRONG, linewidth=1.2,
                marker=".", markersize=3, label="γ²")
ax_coh.axhline(COHERENCE_GOOD,   color="C3", linewidth=1.0, linestyle="--",
               label=f"{COHERENCE_GOOD} (good)")
ax_coh.axhline(COHERENCE_STRONG, color="C1", linewidth=1.0, linestyle="--",
               label=f"{COHERENCE_STRONG} (strong)")
ax_coh.axvspan(chirp_f_min, chirp_f_max, alpha=0.06, color="steelblue")
ax_coh.set_ylim(0.0, 1.05)
ax_coh.set_xlabel("Frequency  (Hz)", fontsize=9)
ax_coh.legend(fontsize=8)


# ── Design notes ─────────────────────────────────────────────────────────────
ax_notes.axis("off")
ax_notes.text(0.5, 0.92,
              "Velocity PI Design  —  Pole-Zero Cancellation + Integrator",
              transform=ax_notes.transAxes, ha="center", va="top",
              fontsize=11, fontweight="bold")

notes = (
    f"PI zero at mechanical pole  →  Ti = τ = {Ti_pi*1000:.2f} ms  "
    f"→  L(s) = K·Kp / (Ti·s)   [pure integrator, −20 dB/dec, 90° PM floor]   |   "
    f"Kp set so |L(j·ωc)| = 1 at ωc = 2π·{BW_TARGET_HZ:.0f} rad/s   |   "
    f"BW = {BW_TARGET_HZ:.0f} Hz = current-loop BW {CURRENT_LOOP_BW_HZ:.0f} Hz / 10"
)
ax_notes.text(0.5, 0.52, notes,
              transform=ax_notes.transAxes, ha="center", va="center",
              fontsize=8.5, fontfamily="monospace",
              color="dimgray", wrap=True)
ax_notes.plot([0.01, 0.99], [0.98, 0.98], transform=ax_notes.transAxes,
              color="black", linewidth=0.8, clip_on=False)
ax_notes.set_xlim(0, 1)
ax_notes.set_ylim(0, 1)


# ── Open-loop magnitude ───────────────────────────────────────────────────────
style_ax(ax_lmag, "Magnitude  (dB)",
         title=(f"Open Loop  L(s) = C(s)·P(s)   PI zero cancels plant pole"
                f"   →   BW = {BW_TARGET_HZ:.0f} Hz"))

ax_lmag.semilogx(f_ctrl, L_mag_db, color=C_MEAS_STRONG, linewidth=2.0,
                 label=f"L(s)  Kp={Kp_pi:.4f}  Ti={Ti_pi*1000:.1f} ms")
ax_lmag.axhline(0, color="black", linewidth=0.8, linestyle="-", alpha=0.5)

if fc_ol is not None:
    ax_lmag.axvline(fc_ol, color=C_CROSSOVER, linewidth=1.2, linestyle="--")
    ax_lmag.scatter([fc_ol], [0], color=C_CROSSOVER, s=50, zorder=5)
    ax_lmag.text(fc_ol * 1.08, 2,
                 f"fc = {fc_ol:.1f} Hz\nPM = {pm:.1f}°",
                 color=C_CROSSOVER, fontsize=8, va="bottom")

if f_pc is not None:
    mag_at_pc = np.interp(f_pc, f_ctrl, L_mag_db)
    ax_lmag.axvline(f_pc, color=C_GM, linewidth=1.2, linestyle="--")
    ax_lmag.scatter([f_pc], [mag_at_pc], color=C_GM, s=50, zorder=5)
    ax_lmag.text(f_pc * 1.08, mag_at_pc + 1,
                 f"GM = {gm_db:.1f} dB\n@ {f_pc:.1f} Hz",
                 color=C_GM, fontsize=8, va="bottom")

ax_lmag.legend(fontsize=8, loc="upper right")
plt.setp(ax_lmag.get_xticklabels(), visible=False)


# ── Open-loop phase ───────────────────────────────────────────────────────────
style_ax(ax_lphase, "Phase  (deg)")
ax_lphase.semilogx(f_ctrl, L_phase_deg, color=C_MEAS_STRONG, linewidth=2.0)
ax_lphase.axhline(-180, color="black", linewidth=0.8, linestyle="-", alpha=0.5)
ax_lphase.axhline(-135, color=C_THRESHOLD, linewidth=0.8, linestyle=":",
                  label="−135°  (45° PM line)")

if fc_ol is not None:
    phase_at_fc = np.interp(fc_ol, f_ctrl, L_phase_deg)
    ax_lphase.axvline(fc_ol, color=C_CROSSOVER, linewidth=1.2, linestyle="--")
    ax_lphase.scatter([fc_ol], [phase_at_fc], color=C_CROSSOVER, s=50, zorder=5)
    ax_lphase.annotate(
        "", xy=(fc_ol * 0.45, -180), xytext=(fc_ol * 0.45, phase_at_fc),
        arrowprops=dict(arrowstyle="<->", color=C_CROSSOVER, lw=1.2)
    )
    mid_phase = (phase_at_fc + (-180)) / 2
    ax_lphase.text(fc_ol * 0.48, mid_phase,
                   f"PM\n{pm:.1f}°", color=C_CROSSOVER,
                   fontsize=7.5, ha="left", va="center")

if f_pc is not None:
    ax_lphase.axvline(f_pc, color=C_GM, linewidth=1.2, linestyle="--")
    ax_lphase.scatter([f_pc], [-180], color=C_GM, s=50, zorder=5)

ax_lphase.legend(fontsize=8)
ax_lphase.set_xlabel("Frequency  (Hz)", fontsize=9)

# Shared x limits
x_lo = max(0.1, chirp_f_min * 0.5)
x_hi = max(chirp_f_max * 4, BW_TARGET_HZ * 8)
for ax in [ax_pmag, ax_pphase, ax_coh]:
    ax.set_xlim(x_lo, x_hi)
for ax in [ax_lmag, ax_lphase]:
    ax.set_xlim(x_lo, x_hi)

fig.text(
    0.5, 0.004,
    (f"source: {csv_path.name}   |   "
     f"K={K_fit:.3f} rad/s/A   B={B_fit*1000:.4f} mA·s/rad   "
     f"τ={tau_fit*1000:.2f}ms   Kp={Kp_pi:.4f}   Ti={Ti_pi*1000:.2f}ms"),
    ha="center", va="bottom", fontsize=7.5, color="dimgray",
    fontfamily="monospace"
)

plt.savefig(out_path, dpi=160, bbox_inches="tight")
print(f"\nwrote: {out_path}")