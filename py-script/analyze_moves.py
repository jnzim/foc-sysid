#!/usr/bin/env python3
"""analyze_moves.py -- what limits tracking, and what the current says about
the plant, from move_telem captures.

Usage:
    python3 analyze_moves.py [label=csv ...] [--out png]

With no captures given, analyzes the clean 2026-10-10 runs in drive_data/archive.

Per-sample least-squares fits, 1 kHz telemetry, only while the position loop
runs (SERVO_ON and samples_consumed > 0):

    following error   e  = c_a*a_cmd + c_v*v_cmd + c0                  [um]
    current           iq = m*a + Fv*v + Fc*sign(v) + i0                [mA]
                      (v, a in motor rad/s, rad/s^2; 2mm lead -> pi rad/mm)

a_cmd is the finite difference of vel_cmd -- the generator's own velocity, so
it is free of measurement noise. Standard errors are the usual OLS ones and
assume white residuals; the residuals are correlated, so treat them as lower
bounds on the uncertainty.

Caveats that matter when reading the output:
  - 1 encoder count is 0.244 um. A fitted c_a whose predicted effect is well
    under a count is below what the encoder resolves.
  - vel_fbk is the encoder boxcar in captures before drive jz-dev@0369baa and
    the observer after it, so velocity residuals are not comparable across
    that boundary.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CPM      = 4096.0          # counts per mm (8192 counts/rev, 2mm lead)
RAD_PMM  = np.pi           # motor rad per mm of travel
UM_COUNT = 1000.0 / CPM    # 0.244 um
FF_FC_MA = 94.0            # FRICTION_FF_COULOMB_MA (drive config.h)
FF_FV_MA = 1.9             # FRICTION_FF_VISCOUS_MA_PER_RADS

A = "../drive_data/archive/"
DEFAULT_RUNS = [
    ("trap 2mm 5mm/s",     A + "move_trap_2mm_5mms_merged_2026-10-10.csv"),
    ("min-jerk 2mm 5mm/s", A + "move_minjerk_2mm_5mms_2026-10-10.csv"),
    ("min-jerk +10mm 10mm/s", A + "move_minjerk_10mm_10mms_pair_move1_2026-10-10.csv"),
    ("min-jerk -10mm 10mm/s", A + "move_minjerk_10mm_10mms_pair_move2_2026-10-10.csv"),
]

# Categorical slots 1-4 in fixed order (validated: CVD dE >= 9.1 adjacent).
# Slots 3 and 4 are under 3:1 on white, so every series also has its own
# marker and the legend always names it.
COLORS  = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
MARKERS = ["o", "s", "^", "D"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def load(path):
    d = pd.read_csv(path).drop_duplicates("timestamp_ms")
    m = d[(d.drive_state == 3) & (d.samples_consumed > 0)].copy()
    if m.empty:
        raise SystemExit(f"{path}: no position-loop ticks")
    m["t_s"] = (m.timestamp_ms - m.timestamp_ms.iloc[0]) / 1000.0
    m["v"]   = m.vel_cmd / CPM                                   # mm/s
    m["a"]   = np.gradient(m.v.values, m.t_s.values)             # mm/s^2
    m["e"]   = (m.pos_cmd - m.pos_fbk) / CPM * 1000.0            # um
    m["vfb"] = m.vel_fbk / CPM
    return m


def ols(y, cols):
    X = np.column_stack(cols)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ beta
    s2 = (r @ r) / (len(y) - X.shape[1])
    se = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X)))
    return beta, se, np.sqrt(s2)


def binned(x, y, edges):
    idx = np.digitize(x, edges) - 1
    cx, cy, ce = [], [], []
    for i in range(len(edges) - 1):
        sel = idx == i
        if sel.sum() >= 8:
            cx.append(x[sel].mean()); cy.append(y[sel].mean())
            ce.append(y[sel].std() / np.sqrt(sel.sum()))
    return np.array(cx), np.array(cy), np.array(ce)


def parse_args(argv):
    runs, out = [], "../drive_data/latest/analyze_moves.png"
    it = iter(argv)
    for a in it:
        if a == "--out":
            out = next(it)
        elif "=" in a:
            label, path = a.split("=", 1); runs.append((label, path))
        else:
            runs.append((os.path.basename(a), a))
    return (runs or DEFAULT_RUNS), out


def main():
    run_specs, out = parse_args(sys.argv[1:])
    runs = [(label, load(path)) for label, path in run_specs]

    # ---- numbers -----------------------------------------------------------
    print("=== per-run summary ===")
    print(f"{'run':24s} {'ms':>5s} {'peak a':>7s} {'e pk':>5s} {'e rms':>6s} {'vel resid':>9s} {'iq std':>6s}")
    for n, m in runs:
        print(f"{n:24s} {len(m):5d} {m.a.abs().max():7.1f} {m.e.abs().max():5.2f} "
              f"{np.sqrt((m.e**2).mean()):6.2f} {np.sqrt(((m.vfb-m.v)**2).mean()):9.3f} {m.iq_cmd.std():6.0f}")
    print("  peak a mm/s^2, error um, vel resid mm/s (vel_fbk - vel_cmd), iq mA")

    print("\n=== following error  e = c_a*a + c_v*v + c0 ===")
    fits_e = {}
    for n, m in runs:
        b, se, s = ols(m.e.values, [m.a.values, m.v.values, np.ones(len(m))])
        fits_e[n] = b
        pk = abs(b[0]) * m.a.abs().max()
        note = "" if pk >= UM_COUNT else "   <- accel effect under 1 count, not resolved"
        print(f"{n:24s} c_a {b[0]*1e3:+6.2f} +- {se[0]*1e3:4.2f} um/(m/s^2)  "
              f"c_v {b[1]:+.3f} +- {se[1]:.3f} um/(mm/s)  resid {s:.2f} um{note}")

    allm = pd.concat([m for _, m in runs])
    mv = allm[allm.v.abs() > 0.5]                  # sign(v) undefined near rest
    v_rad, a_rad = mv.v.values * RAD_PMM, mv.a.values * RAD_PMM
    bi, sei, si = ols(mv.iq_cmd.values.astype(float),
                      [a_rad, v_rad, np.sign(v_rad), np.ones(len(mv))])
    print("\n=== current  iq = m*a + Fv*v + Fc*sign(v) + i0  (pooled) ===")
    print(f"  m   {bi[0]:+.4f} +- {sei[0]:.4f} mA/(rad/s^2)", end="")
    if bi[0] > 0:
        lo = bi[0] - 2*sei[0]
        print(f"  -> {1000/bi[0]:.0f} rad/s^2/A (2-sigma {1000/(bi[0]+2*sei[0]):.0f} .. "
              f"{'unbounded' if lo <= 0 else f'{1000/lo:.0f}'})")
    else:
        print()
    print(f"  Fv  {bi[1]:+.3f} +- {sei[1]:.3f} mA/(rad/s)   (feedforward {FF_FV_MA})")
    print(f"  Fc  {bi[2]:+.1f} +- {sei[2]:.1f} mA          (feedforward {FF_FC_MA:.0f})")
    print(f"  i0  {bi[3]:+.1f} +- {sei[3]:.1f} mA;  residual std {si:.0f} mA, n={len(mv)}")
    print(f"  largest inertia term in these moves: {abs(bi[0])*allm.a.abs().max()*RAD_PMM:.1f} mA "
          f"vs {si:.0f} mA noise")

    # ---- plot --------------------------------------------------------------
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})
    fig, ax = plt.subplots(2, 2, figsize=(13, 9.5))
    fig.suptitle("Move analysis -- what limits tracking, and what the current says about the plant",
                 color=INK, fontsize=12, x=0.01, ha="left")

    # (a) following error vs commanded acceleration, binned
    a0 = ax[0, 0]
    edges = np.linspace(-105, 105, 22)
    for k, (n, m) in enumerate(runs):
        cx, cy, ce = binned(m.a.values, m.e.values, edges)
        a0.errorbar(cx, cy, yerr=ce, fmt=MARKERS[k], ms=6, color=COLORS[k], ecolor=COLORS[k],
                    elinewidth=1, capsize=0, mec="white", mew=1.0, label=n, zorder=3)
    k_tr = max(range(len(runs)), key=lambda i: runs[i][1].a.abs().max())
    n_tr = runs[k_tr][0]; b = fits_e[n_tr]
    xa = np.linspace(-105, 105, 50)
    a0.plot(xa, b[0]*xa + b[2], color=COLORS[k_tr], lw=2, zorder=2,
            label=f"fit, {n_tr}: {b[0]*1e3:+.1f} um per m/s^2")
    a0.axhspan(-UM_COUNT/2, UM_COUNT/2, color=GRID, zorder=0, lw=0)
    a0.text(103, UM_COUNT/2 + 0.02, "+-1/2 count", color=INK2, fontsize=8, ha="right", va="bottom")
    a0.set_xlabel("commanded acceleration (mm/s^2)")
    a0.set_ylabel("following error, bin mean +- SE (um)")
    a0.set_title("(a) Error tracks acceleration: no accel feedforward", loc="left")

    # (b) current vs commanded velocity, inertia term removed
    a1 = ax[0, 1]
    vedges = np.linspace(-35, 35, 29)
    for k, (n, m) in enumerate(runs):
        mm = m[m.v.abs() > 0.5]
        vr = mm.v.values * RAD_PMM
        iq_fr = mm.iq_cmd.values - bi[0] * mm.a.values * RAD_PMM
        cx, cy, ce = binned(vr, iq_fr, vedges)
        a1.errorbar(cx, cy, yerr=ce, fmt=MARKERS[k], ms=6, color=COLORS[k], ecolor=COLORS[k],
                    elinewidth=1, capsize=0, mec="white", mew=1.0, label=n, zorder=3)
    for sgn in (-1, 1):
        xv = np.linspace(0.5, 35, 40) * sgn
        a1.plot(xv, bi[2]*np.sign(xv) + bi[1]*xv + bi[3], color=INK, lw=2, zorder=2,
                label=(f"fit: Fc {bi[2]:.0f} mA, Fv {bi[1]:.2f} mA/(rad/s)" if sgn > 0 else None))
        a1.plot(xv, FF_FC_MA*np.sign(xv) + FF_FV_MA*xv, color=INK2, lw=1.5, ls="--", zorder=2,
                label=(f"feedforward: Fc {FF_FC_MA:.0f}, Fv {FF_FV_MA}" if sgn > 0 else None))
    a1.axhline(0, color=INK2, lw=0.8); a1.axvline(0, color=INK2, lw=0.8)
    a1.set_xlabel("commanded motor velocity (rad/s)   [10 mm/s = 31.4 rad/s]")
    a1.set_ylabel("iq_cmd - fitted inertia term, bin mean +- SE (mA)")
    a1.set_title("(b) Friction from the moves matches the feedforward", loc="left")

    # (c) the largest-acceleration run: measured error vs the c_a*a prediction
    a2 = ax[1, 0]
    m = runs[k_tr][1]
    a2.plot(m.t_s*1000, m.e, color=COLORS[k_tr], lw=1.2, drawstyle="steps-post",
            label=f"measured error ({n_tr})")
    a2.plot(m.t_s*1000, b[0]*m.a + b[1]*m.v + b[2], color=INK, lw=2,
            label="model c_a*a + c_v*v + c0")
    a2.axhline(0, color=INK2, lw=0.8)
    a2.set_xlabel("time from move start (ms)")
    a2.set_ylabel("following error (um)   [1 count = 0.244 um]")
    a2.set_title("(c) The lag appears during the accel/decel ramps", loc="left")

    # (d) iq_cmd noise spectrum
    a3 = ax[1, 1]
    for k, (n, m) in enumerate(runs):
        x = m.iq_cmd.values.astype(float)
        if len(x) < 512:
            continue
        x = x - np.convolve(x, np.ones(51)/51, mode="same")      # remove the slow, profile-driven part
        x = x[30:-30]
        seg = 256; segs = [x[i:i+seg] for i in range(0, len(x)-seg+1, seg//2)]
        w = np.hanning(seg)
        P = np.mean([np.abs(np.fft.rfft(s*w))**2 for s in segs], axis=0) / (w**2).sum() / 500.0
        f = np.fft.rfftfreq(seg, 1e-3)
        a3.semilogy(f[1:], P[1:], color=COLORS[k], lw=2, label=n)
    a3.set_xlim(0, 500)
    a3.set_xlabel("frequency (Hz)   [1 kHz telemetry: 500 Hz is Nyquist]")
    a3.set_ylabel("iq_cmd PSD, high-passed (mA^2/Hz)")
    # A flat spectrum puts ~60% of its power in 200-500 Hz simply because that
    # is 60% of the band -- so band percentages alone do not say "high
    # frequency". Read the shape. Below ~20 Hz is removed by the 51-sample
    # moving-average high-pass above, not by the drive.
    a3.axvspan(0, 20, color=GRID, lw=0, zorder=0)
    a3.text(22, a3.get_ylim()[0]*1.5, "removed by the\nhigh-pass", color=INK2, fontsize=8)
    a3.set_title(f"(d) Current noise ({si:.0f} mA std) is broadband, flat to Nyquist", loc="left")

    for a in ax.flat:
        a.grid(color=GRID, lw=0.8); a.set_axisbelow(True)
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)
        a.legend(loc="best", fontsize=8, frameon=False)

    fig.text(0.01, 0.005,
             f"Inertia from these moves: {1000/bi[0]:.0f} rad/s^2/A, 2-sigma "
             f"{1000/(bi[0]+2*sei[0]):.0f} .. "
             f"{'unbounded' if bi[0]-2*sei[0] <= 0 else f'{1000/(bi[0]-2*sei[0]):.0f}'} -- "
             f"not resolved (largest inertia term {abs(bi[0])*allm.a.abs().max()*RAD_PMM:.0f} mA vs "
             f"{si:.0f} mA noise).  OLS standard errors assume white residuals: lower bounds.",
             color=INK2, fontsize=8.5)
    plt.tight_layout(rect=(0, 0.02, 1, 0.97))
    plt.savefig(out, dpi=130, facecolor="white")
    print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
