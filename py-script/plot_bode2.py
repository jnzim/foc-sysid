#!/usr/bin/env python3
"""
bode_from_sysid.py

Drop-in replacement for the old Welch/CSD Bode script.

Designed for your current STM/RPi CSV format:

    host_time_s,frame,t,dt,ia_mA,ib_mA,ic_mA,id_mA,iq_mA,
    vd_mV,vq_mV,theta_mrad,adc_a,adc_b,adc_c,flags,crc,pad

Current hack:
    ic_mA is hijacked as the commanded test frequency in Hz.

Default behavior:
    - Keeps only rows where flags == 0x0001
    - Drops rows before host_time_s > 1.5
    - Uses host_time_s as actual sample time
    - Uses ic_mA as frequency tag
    - Uses vd_mV as input
    - Uses id_mA as output
    - Groups rows by frequency
    - Discards startup cycles from each frequency segment
    - Fits sine waves to input and output at the known command frequency
    - Computes H = output/input
    - Plots measured vs RL theory

Typical use:
    python bode_from_sysid.py capture.csv

Useful variants:
    python bode_from_sysid.py capture.csv --output ia_mA --save bode_ia.png
    python bode_from_sysid.py capture.csv --input vq_mV --output iq_mA --save bode_iq.png
    python bode_from_sysid.py capture.csv --start-time 0 --flag any

Why this script instead of the old one:
    The old script used ia_mA while labeling the plot id/Vd, and used Welch/CSD
    over the whole mixed stepped-sine file. This script makes one Bode point per
    commanded frequency, which is the right method for a stepped-sine capture.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def normalize_flags(s: pd.Series) -> pd.Series:
    """
    Convert flags column to normalized lowercase strings like '0x0001'.
    Handles strings or integer-looking values.
    """
    def one(v):
        if pd.isna(v):
            return ""
        if isinstance(v, str):
            return v.strip().lower()
        try:
            return f"0x{int(v):04x}"
        except Exception:
            return str(v).strip().lower()
    return s.map(one)


def require_col(df: pd.DataFrame, col: str) -> None:
    if col not in df.columns:
        raise ValueError(f"Missing column '{col}'. Available: {', '.join(df.columns)}")


def as_float_array(series: pd.Series) -> np.ndarray:
    return pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)


def convert_to_si(series: pd.Series, col: str, role: str) -> np.ndarray:
    """
    Convert telemetry units to SI.
    role is only for readability and to avoid converting frequency if someone names it *_mA.
    """
    x = as_float_array(series)
    name = col.lower()

    if role == "freq":
        return x

    if name.endswith("_mv"):
        return x / 1000.0

    if name.endswith("_ma"):
        return x / 1000.0

    if name.endswith("_mrad"):
        return x / 1000.0

    return x


def sine_fit(t: np.ndarray, x: np.ndarray, freq_hz: float):
    """
    Fit:
        x(t) = C*cos(wt) + S*sin(wt) + offset

    Complex convention:
        x(t) = Re{X * exp(j*w*t)}
        X = C - j*S

    Then transfer function is simply:
        H = Y / U
    """
    if freq_hz <= 0:
        raise ValueError(f"Bad frequency: {freq_hz}")

    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)

    # Improve conditioning
    tt = t - t[0]
    w = 2.0 * math.pi * freq_hz

    M = np.column_stack((
        np.cos(w * tt),
        np.sin(w * tt),
        np.ones_like(tt),
    ))

    coeff, *_ = np.linalg.lstsq(M, x, rcond=None)

    c_cos = coeff[0]
    c_sin = coeff[1]
    offset = coeff[2]

    X = c_cos - 1j * c_sin

    fit = M @ coeff
    residual = x - fit

    rms_sig = float(np.sqrt(np.mean((x - np.mean(x)) ** 2)))
    rms_err = float(np.sqrt(np.mean(residual ** 2)))

    if rms_sig > 1e-12:
        fit_quality = 1.0 - rms_err / rms_sig
    else:
        fit_quality = np.nan

    return X, offset, fit_quality, fit


def trim_and_fit_group(
    g: pd.DataFrame,
    time_col: str,
    freq_col: str,
    input_col: str,
    output_col: str,
    discard_cycles: float,
    min_cycles_total: float,
    min_cycles_used: float,
    min_samples: int,
):
    # Frequency tag for this segment
    fvals = convert_to_si(g[freq_col], freq_col, "freq")
    fvals = fvals[np.isfinite(fvals)]
    if len(fvals) == 0:
        return None

    freq_hz = float(np.median(fvals))
    if not np.isfinite(freq_hz) or freq_hz <= 0:
        return None

    t = convert_to_si(g[time_col], time_col, "time")
    u = convert_to_si(g[input_col], input_col, "input")
    y = convert_to_si(g[output_col], output_col, "output")

    valid = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t = t[valid]
    u = u[valid]
    y = y[valid]

    if len(t) < min_samples:
        return None

    order = np.argsort(t)
    t = t[order]
    u = u[order]
    y = y[order]

    # Remove duplicate or non-increasing host timestamps.
    keep = np.ones_like(t, dtype=bool)
    keep[1:] = np.diff(t) > 0
    t = t[keep]
    u = u[keep]
    y = y[keep]

    if len(t) < min_samples:
        return None

    duration_total = t[-1] - t[0]
    cycles_total = duration_total * freq_hz

    if cycles_total < min_cycles_total:
        return None

    # Discard initial transient cycles for this frequency.
    if discard_cycles > 0:
        t0_keep = t[0] + discard_cycles / freq_hz
        keep = t >= t0_keep
        t = t[keep]
        u = u[keep]
        y = y[keep]

    if len(t) < min_samples:
        return None

    cycles_used = (t[-1] - t[0]) * freq_hz
    if cycles_used < min_cycles_used:
        return None

    U, u_offset, u_q, _ = sine_fit(t, u, freq_hz)
    Y, y_offset, y_q, _ = sine_fit(t, y, freq_hz)

    if abs(U) < 1e-12:
        return None

    H = Y / U

    return {
        "freq_hz": freq_hz,
        "H_real": H.real,
        "H_imag": H.imag,
        "mag_A_per_V": abs(H),
        "mag_db": 20.0 * np.log10(abs(H)),
        "phase_deg_raw": np.angle(H, deg=True),
        "input_amp_V": abs(U),
        "output_amp_A": abs(Y),
        "input_offset_V": u_offset,
        "output_offset_A": y_offset,
        "input_fit_quality": u_q,
        "output_fit_quality": y_q,
        "cycles_total": cycles_total,
        "cycles_used": cycles_used,
        "n_samples": len(t),
        "t_start": t[0],
        "t_end": t[-1],
    }


def main() -> int:
    p = argparse.ArgumentParser(
        description="Stepped-sine Bode plot from STM/RPi current sysid CSV."
    )
    p.add_argument("csv", help="Input CSV file")

    p.add_argument("--time", default="host_time_s")
    p.add_argument("--freq", default="ic_mA", help="Frequency tag column; current hack uses ic_mA")
    p.add_argument("--input", default="vd_mV", help="Input voltage column")
    p.add_argument("--output", default="id_mA", help="Output current column")

    p.add_argument("--flag", default="0x0001",
                   help="Keep only this flags value. Use --flag any to disable.")
    p.add_argument("--start-time", type=float, default=1.5,
                   help="Drop rows with host_time_s <= this value.")
    p.add_argument("--skip-rows", type=int, default=0,
                   help="Drop first N rows after reading CSV, before filtering.")

    p.add_argument("--discard-cycles", type=float, default=3.0,
                   help="Discard this many cycles at start of each frequency segment.")
    p.add_argument("--min-cycles-total", type=float, default=6.0,
                   help="Require this many total cycles before trimming.")
    p.add_argument("--min-cycles-used", type=float, default=2.0,
                   help="Require this many cycles remain after trimming.")
    p.add_argument("--min-samples", type=int, default=30)

    p.add_argument("--R", type=float, default=1.55, help="Theory resistance, ohm")
    p.add_argument("--L", type=float, default=0.00204, help="Theory inductance, H")

    p.add_argument("--save", default="bode_plot.png", help="Output plot PNG")
    p.add_argument("--csv-out", default="bode_points.csv", help="Output analyzed points CSV")
    p.add_argument("--no-show", action="store_true", help="Do not display plot window")

    args = p.parse_args()

    path = Path(args.csv)
    if not path.exists():
        raise FileNotFoundError(path)

    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]

    for col in [args.time, args.freq, args.input, args.output]:
        require_col(df, col)

    if args.skip_rows > 0:
        df = df.iloc[args.skip_rows:].reset_index(drop=True)

    # Optional flag filtering
    if args.flag.lower() != "any":
        require_col(df, "flags")
        flags_norm = normalize_flags(df["flags"])
        want = args.flag.strip().lower()
        df = df[flags_norm == want].copy()

    # Optional global start time filtering
    if args.start_time is not None:
        t_global = as_float_array(df[args.time])
        df = df[t_global > args.start_time].copy()

    # Convert freq column to numeric but do not convert units based on name.
    df[args.freq] = pd.to_numeric(df[args.freq], errors="coerce")
    df = df[np.isfinite(df[args.freq])].copy()

    print()
    print(f"file:        {path}")
    print(f"rows kept:   {len(df)}")
    print(f"time col:    {args.time}")
    print(f"freq col:    {args.freq}  (treated as Hz)")
    print(f"input col:   {args.input}")
    print(f"output col:  {args.output}")
    print(f"flag filter: {args.flag}")
    print(f"start time:  > {args.start_time} s")
    print()

    if len(df) == 0:
        print("No rows after filtering.")
        return 2

    # Print sanity ranges
    def rng(col):
        x = pd.to_numeric(df[col], errors="coerce")
        return float(np.nanmin(x)), float(np.nanmax(x))

    for col in [args.freq, args.input, args.output]:
        lo, hi = rng(col)
        print(f"{col:>10s} range: {lo:g} to {hi:g}")

    # Estimate sample rate from kept rows.
    t_all = as_float_array(df[args.time])
    t_all = t_all[np.isfinite(t_all)]
    if len(t_all) > 2:
        t_all = np.sort(t_all)
        dts = np.diff(t_all)
        dts = dts[dts > 0]
        if len(dts):
            fs_est = 1.0 / np.median(dts)
            print(f"fs estimate: {fs_est:.1f} Hz from median dt")
    print()

    results = []
    skipped = 0

    # Exact grouping is okay because freq tag is normally integer Hz.
    for _, g in df.groupby(args.freq, sort=True):
        r = trim_and_fit_group(
            g=g,
            time_col=args.time,
            freq_col=args.freq,
            input_col=args.input,
            output_col=args.output,
            discard_cycles=args.discard_cycles,
            min_cycles_total=args.min_cycles_total,
            min_cycles_used=args.min_cycles_used,
            min_samples=args.min_samples,
        )
        if r is None:
            skipped += 1
        else:
            results.append(r)

    if not results:
        print("No valid frequency groups found.")
        print("Likely causes:")
        print("  - The input/output columns are not sinusoidal.")
        print("  - Too few cycles per frequency.")
        print("  - --discard-cycles is too large.")
        print("  - --start-time filtered out too much data.")
        print("  - flags are not 0x0001; try --flag any.")
        return 3

    res = pd.DataFrame(results).sort_values("freq_hz").reset_index(drop=True)

    # Unwrap phase in frequency order.
    res["phase_deg"] = np.rad2deg(np.unwrap(np.deg2rad(res["phase_deg_raw"].values)))

    # Theory
    f = res["freq_hz"].to_numpy(dtype=float)
    w = 2.0 * np.pi * f
    H_th = 1.0 / (args.R + 1j * w * args.L)
    mag_th_db = 20.0 * np.log10(np.abs(H_th))
    phase_th_deg = np.angle(H_th, deg=True)

    print(f"valid bode points: {len(res)}")
    print(f"skipped groups:    {skipped}")
    print()
    print(res[[
        "freq_hz",
        "mag_A_per_V",
        "mag_db",
        "phase_deg",
        "input_amp_V",
        "output_amp_A",
        "input_fit_quality",
        "output_fit_quality",
        "cycles_used",
        "n_samples",
    ]].to_string(index=False))

    if args.csv_out:
        res.to_csv(args.csv_out, index=False)
        print()
        print(f"wrote: {args.csv_out}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    ax1.semilogx(res["freq_hz"], res["mag_db"], "bo-", label="Measured sine-fit")
    ax1.semilogx(f, mag_th_db, "r--", label=f"Theory R={args.R:g}Ω L={args.L*1000:g}mH")
    ax1.set_ylabel("Magnitude (dB re A/V)")
    ax1.set_title(f"Current Plant Bode - {args.output} / {args.input}")
    ax1.grid(True, which="both")
    ax1.legend()

    ax2.semilogx(res["freq_hz"], res["phase_deg"], "bo-", label="Measured sine-fit")
    ax2.semilogx(f, phase_th_deg, "r--", label="Theory")
    ax2.set_ylabel("Phase (deg)")
    ax2.set_xlabel("Frequency (Hz)")
    ax2.grid(True, which="both")
    ax2.legend()

    fig.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f"wrote: {args.save}")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise
