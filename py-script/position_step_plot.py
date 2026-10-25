#!/usr/bin/env python3
"""Analyze SYSID_TEST_POSITION_STEP (test 7) -- position step on the test 8 cascade.

Usage:
    python3 position_step_plot.py drive_data/latest/sysid_log.csv [bandwidth_hz]

Same loop as test 8 (position P -> velocity PI, no friction FF -> current PI),
pos_cmd stepping 0 <-> POS_STEP_AMPLITUDE. Measures 10-90% rise time and
overshoot on every step and compares the rise time with what the closed-loop
-3 dB bandwidth predicts (default 125 Hz, the test 8 result on slide 10).

Telemetry mapping (SYSID_TEST_POSITION_STEP):
    sysid_f   -> position command [mrad]
    iq_cmd_mA -> encoder position read every tick [mrad]
    t         -> firmware control tick, 50 us
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

FLAG_RUN = 1
TICK_S = 50e-6
PRE_S = 0.020          # baseline: mean of the 20 ms before each edge
FINAL_S = 0.050        # final value: mean of the last 50 ms of each hold
PLOT_MS = 30.0


def crossing_time(t, y, level):
    """First time y crosses level, linearly interpolated. None if never."""
    idx = np.where(y >= level)[0]
    if len(idx) == 0 or idx[0] == 0:
        return None
    i = idx[0]
    return t[i - 1] + (level - y[i - 1]) * (t[i] - t[i - 1]) / (y[i] - y[i - 1])


def second_order_tr(bw_hz, zeta):
    """10-90% rise time of a standard 2nd-order system with -3 dB bandwidth bw_hz."""
    k = np.sqrt(1 - 2 * zeta**2 + np.sqrt(2 - 4 * zeta**2 + 4 * zeta**4))
    wn = 2 * np.pi * bw_hz / k
    wd = wn * np.sqrt(1 - zeta**2)
    t = np.linspace(0, 20 / wn, 20001)
    phi = np.arccos(zeta)
    y = 1 - np.exp(-zeta * wn * t) * np.sin(wd * t + phi) / np.sqrt(1 - zeta**2)
    return crossing_time(t, y, 0.9) - crossing_time(t, y, 0.1)


def zeta_from_overshoot(os_frac):
    if os_frac <= 0:
        return 1.0
    l = np.log(os_frac)
    return -l / np.sqrt(np.pi**2 + l**2)


def main():
    csv = sys.argv[1] if len(sys.argv) > 1 else "../drive_data/latest/sysid_log.csv"
    bw_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 125.0

    df = pd.read_csv(csv)
    flags = df["flags"].map(lambda v: int(str(v), 0))
    df = df[(flags & 0xFF) == FLAG_RUN].drop_duplicates("t").sort_values("t")
    t = df["t"].to_numpy(dtype=np.float64) * TICK_S
    cmd = df["sysid_f"].to_numpy(dtype=np.float64) / 1000.0
    pos = df["iq_cmd_mA"].to_numpy(dtype=np.float64) / 1000.0
    print(f"{len(df)} RUN samples, median spacing {np.median(np.diff(t))*1e6:.0f} us")

    edges = np.where(np.diff(cmd) != 0)[0] + 1
    if len(edges) == 0:
        raise SystemExit("no step edges in pos_cmd -- was the firmware built with SYSID_TEST 7?")
    bounds = list(edges) + [len(t)]

    results = []
    fig, ax = plt.subplots(figsize=(10, 6))
    for k, e in enumerate(edges):
        t0 = t[e]
        end = bounds[k + 1]
        pre = (t >= t0 - PRE_S) & (t < t0)
        seg = np.arange(e, end)
        if pre.sum() < 5 or t[seg[-1]] - t0 < FINAL_S * 2:
            continue
        y0 = pos[pre].mean()
        dcmd = cmd[e] - cmd[e - 1]
        tail = seg[t[seg] >= t[seg[-1]] - FINAL_S]
        yf = pos[tail].mean()
        yn = (pos[seg] - y0) / (yf - y0)          # 0 -> 1 on the measured final value
        ts = t[seg] - t0
        t10, t90 = crossing_time(ts, yn, 0.1), crossing_time(ts, yn, 0.9)
        if t10 is None or t90 is None:
            continue
        os_frac = yn.max() - 1.0
        results.append(dict(dir="up" if dcmd > 0 else "down", tr=t90 - t10,
                            os=os_frac, final=(yf - y0) / dcmd))
        ax.plot(ts * 1e3, yn, lw=1, color="#1a1f2b" if dcmd > 0 else "#b4480e", alpha=0.7)

    if not results:
        raise SystemExit("no complete steps found")

    tr = np.array([r["tr"] for r in results])
    os_ = np.array([r["os"] for r in results])
    fin = np.array([r["final"] for r in results])
    zeta = zeta_from_overshoot(os_.mean())
    tr_1st = 0.35 / bw_hz
    tr_2nd = second_order_tr(bw_hz, zeta)

    print(f"steps: {len(results)} ({sum(r['dir']=='up' for r in results)} up, "
          f"{sum(r['dir']=='down' for r in results)} down)")
    for i, r in enumerate(results):
        print(f"  {i+1:2d} {r['dir']:4s}  tr {r['tr']*1e3:.2f} ms  overshoot {r['os']*100:5.1f}%  "
              f"final/cmd {r['final']:.3f}")
    print(f"rise time 10-90%: mean {tr.mean()*1e3:.2f} ms, min {tr.min()*1e3:.2f}, max {tr.max()*1e3:.2f}")
    print(f"overshoot: mean {os_.mean()*100:.1f}%  -> zeta {zeta:.2f}")
    print(f"predicted from BW {bw_hz:g} Hz: 1st order 0.35/BW = {tr_1st*1e3:.2f} ms, "
          f"2nd order at zeta {zeta:.2f} = {tr_2nd*1e3:.2f} ms")
    print(f"measured / 2nd-order prediction = {tr.mean()/tr_2nd:.2f}")

    ax.axhline(1.0, color="#8a8f99", lw=0.8)
    ax.axhline(0.1, color="#8a8f99", lw=0.5, ls=":")
    ax.axhline(0.9, color="#8a8f99", lw=0.5, ls=":")
    ax.set_xlim(0, PLOT_MS)
    ax.set_xlabel("time since step [ms]")
    ax.set_ylabel("position / final value")
    ax.set_title(f"Position step, {len(results)} steps: tr {tr.mean()*1e3:.2f} ms, "
                 f"overshoot {os_.mean()*100:.1f}%  |  BW {bw_hz:g} Hz predicts "
                 f"{tr_2nd*1e3:.2f} ms (2nd order, zeta {zeta:.2f})", fontsize=10)
    ax.grid(alpha=0.3)
    out = Path(csv).with_name("position_step_plot.png")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
