#!/usr/bin/env python3
# py-script/plot_sysid.py
#
# Usage:
#   python3 py-script/plot_sysid.py drive_data/sysid_log.csv
#
# Saves plots into the same directory as the CSV.

import sys
from pathlib import Path

import pandas as pd

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt

import os

ADC_A_OFFSET = 2084
ADC_B_OFFSET = 2088
ADC_C_OFFSET = 2032


def require_columns(df, cols):
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise RuntimeError(f"CSV missing columns: {missing}")


def save_plot(path):
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()
    print(f"wrote: {path}")


def noise_stats(series, window=31):
    """
    Estimate high-frequency noise by subtracting a rolling average.

    rms_noise:
        RMS of signal - rolling_average(signal)

    p99_p1:
        Robust peak-to-peak estimate. Ignores rare extreme outliers better
        than max-min.
    """
    y = pd.to_numeric(series, errors="coerce").dropna()

    if y.empty:
        return {
            "mean": 0.0,
            "std": 0.0,
            "rms_noise": 0.0,
            "p99_p1": 0.0,
            "min": 0.0,
            "max": 0.0,
            "abs_max": 0.0,
        }

    if len(y) < window:
        noise = y - y.mean()
    else:
        trend = y.rolling(window, center=True, min_periods=1).mean()
        noise = y - trend

    return {
        "mean": float(y.mean()),
        "std": float(y.std()),
        "rms_noise": float((noise * noise).mean() ** 0.5),
        "p99_p1": float(noise.quantile(0.99) - noise.quantile(0.01)),
        "min": float(y.min()),
        "max": float(y.max()),
        "abs_max": float(y.abs().max()),
    }


def stats_text(name, stats, unit):
    return (
        f"{name}: mean={stats['mean']:.1f} {unit}, "
        f"rms_noise={stats['rms_noise']:.1f} {unit}, "
        f"p99-p1={stats['p99_p1']:.1f} {unit}"
    )


def annotate_stats(lines):
    text = "\n".join(lines)
    plt.gca().text(
        0.01,
        0.99,
        text,
        transform=plt.gca().transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round", alpha=0.15),
    )


def print_stats_block(title, stats_by_name, unit):
    print("")
    print(title)
    print("-" * len(title))

    for name, stats in stats_by_name.items():
        print(
            f"{name:8s} "
            f"mean={stats['mean']:9.2f} {unit}, "
            f"std={stats['std']:9.2f} {unit}, "
            f"rms_noise={stats['rms_noise']:9.2f} {unit}, "
            f"p99-p1={stats['p99_p1']:9.2f} {unit}, "
            f"min={stats['min']:9.2f}, "
            f"max={stats['max']:9.2f}"
        )


def main():
    if len(sys.argv) < 2:
        print("usage: plot_sysid.py <csv_path>")
        return 1

    csv_path = Path(sys.argv[1]).resolve()

    if not csv_path.exists():
        print(f"ERROR: CSV not found: {csv_path}")
        return 1

    out_dir = csv_path.parent

    print(f"reading: {csv_path}")
    print(f"output dir: {out_dir}")

    df = pd.read_csv(csv_path)

    if df.empty:
        print("ERROR: CSV is empty")
        return 1

    require_columns(
        df,
        [
            "host_time_s",
            "frame",
            "t",
            "dt",
            "ia_mA",
            "ib_mA",
            "ic_mA",
            "id_mA",
            "iq_mA",
            "vd_mV",
            "vq_mV",
            "theta_mrad",
            "adc_a",
            "adc_b",
            "adc_c",
        ],
    )

    x = df["host_time_s"]

    # Derived columns
    df["i_sum_mA"] = df["ia_mA"] + df["ib_mA"] + df["ic_mA"]

    df["adc_a_err"] = df["adc_a"] - ADC_A_OFFSET
    df["adc_b_err"] = df["adc_b"] - ADC_B_OFFSET
    df["adc_c_err"] = df["adc_c"] - ADC_C_OFFSET

    # Stats
    ia_stats = noise_stats(df["ia_mA"])
    ib_stats = noise_stats(df["ib_mA"])
    ic_stats = noise_stats(df["ic_mA"])

    id_stats = noise_stats(df["id_mA"])
    iq_stats = noise_stats(df["iq_mA"])

    isum_stats = noise_stats(df["i_sum_mA"])

    adc_a_stats = noise_stats(df["adc_a"])
    adc_b_stats = noise_stats(df["adc_b"])
    adc_c_stats = noise_stats(df["adc_c"])

    adc_a_err_stats = noise_stats(df["adc_a_err"])
    adc_b_err_stats = noise_stats(df["adc_b_err"])
    adc_c_err_stats = noise_stats(df["adc_c_err"])

    print("")
    print("first current rows")
    print("------------------")
    print(
        df[
            [
                "ia_mA",
                "ib_mA",
                "ic_mA",
                "i_sum_mA",
                "id_mA",
                "iq_mA",
                "adc_a",
                "adc_b",
                "adc_c",
            ]
        ].head(20)
    )

    print_stats_block(
        "phase current stats",
        {
            "ia": ia_stats,
            "ib": ib_stats,
            "ic": ic_stats,
            "sum": isum_stats,
        },
        "mA",
    )

    print_stats_block(
        "dq current stats",
        {
            "id": id_stats,
            "iq": iq_stats,
        },
        "mA",
    )

    print_stats_block(
        "raw ADC stats",
        {
            "adc_a": adc_a_stats,
            "adc_b": adc_b_stats,
            "adc_c": adc_c_stats,
        },
        "cnt",
    )

    print("")
    print("current sum check")
    print("-----------------")
    print(f"mean i_sum_mA    : {df['i_sum_mA'].mean():.2f}")
    print(f"min i_sum_mA     : {df['i_sum_mA'].min():.2f}")
    print(f"max i_sum_mA     : {df['i_sum_mA'].max():.2f}")
    print(f"abs max i_sum_mA : {df['i_sum_mA'].abs().max():.2f}")

    # -------------------------------------------------------------------------
    # Phase currents
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["ia_mA"], label="ia_mA")
    plt.plot(x, df["ib_mA"], label="ib_mA")
    plt.plot(x, df["ic_mA"], label="ic_mA")
    annotate_stats(
        [
            stats_text("ia", ia_stats, "mA"),
            stats_text("ib", ib_stats, "mA"),
            stats_text("ic", ic_stats, "mA"),
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("current (mA)")
    plt.title("SysID phase currents")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_phase_currents.png")

    # -------------------------------------------------------------------------
    # Phase-current sum
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["i_sum_mA"], label="ia + ib + ic")
    plt.axhline(0, linestyle="--", linewidth=1)
    annotate_stats(
        [
            stats_text("sum", isum_stats, "mA"),
            f"abs max={df['i_sum_mA'].abs().max():.1f} mA",
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("current sum (mA)")
    plt.title("SysID phase current sum")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_phase_current_sum.png")

    # -------------------------------------------------------------------------
    # Raw ADC counts
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["adc_a"], label="adc_a")
    plt.plot(x, df["adc_b"], label="adc_b")
    plt.plot(x, df["adc_c"], label="adc_c")
    annotate_stats(
        [
            stats_text("adc_a", adc_a_stats, "cnt"),
            stats_text("adc_b", adc_b_stats, "cnt"),
            stats_text("adc_c", adc_c_stats, "cnt"),
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("ADC counts")
    plt.title("SysID raw ADC current-sense counts")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_raw_adc_counts.png")

    # -------------------------------------------------------------------------
    # Raw ADC offset error
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["adc_a_err"], label="adc_a - offset")
    plt.plot(x, df["adc_b_err"], label="adc_b - offset")
    plt.plot(x, df["adc_c_err"], label="adc_c - offset")
    annotate_stats(
        [
            stats_text("adc_a_err", adc_a_err_stats, "cnt"),
            stats_text("adc_b_err", adc_b_err_stats, "cnt"),
            stats_text("adc_c_err", adc_c_err_stats, "cnt"),
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("ADC counts from offset")
    plt.title("SysID raw ADC offset error")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_raw_adc_offset_error.png")

    # -------------------------------------------------------------------------
    # D/Q currents
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["id_mA"], label="id_mA")
    plt.plot(x, df["iq_mA"], label="iq_mA")
    annotate_stats(
        [
            stats_text("id", id_stats, "mA"),
            stats_text("iq", iq_stats, "mA"),
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("current (mA)")
    plt.title("SysID d/q currents")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dq_current.png")

    # -------------------------------------------------------------------------
    # D/Q voltage command
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["vd_mV"], label="vd_mV")
    plt.plot(x, df["vq_mV"], label="vq_mV")
    plt.xlabel("host_time_s")
    plt.ylabel("voltage command (mV)")
    plt.title("SysID d/q voltage command")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dq_voltage_cmd.png")

    # -------------------------------------------------------------------------
    # Electrical angle
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["theta_mrad"], label="theta_mrad")
    plt.xlabel("host_time_s")
    plt.ylabel("theta (mrad)")
    plt.title("SysID electrical angle")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_theta.png")

    # -------------------------------------------------------------------------
    # dt
    # -------------------------------------------------------------------------
    dt_stats = noise_stats(df["dt"])

    plt.figure(figsize=(12, 6))
    plt.plot(x, df["dt"], label="dt")
    annotate_stats(
        [
            stats_text("dt", dt_stats, "ticks"),
            f"median={df['dt'].median():.1f} ticks",
        ]
    )
    plt.xlabel("host_time_s")
    plt.ylabel("STM sample delta")
    plt.title("SysID SPI sample delta")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dt.png")
    
    
    # Alignment phase analysis — R identification
    align = df[(df['vd_mV'] == 3000) & (df['vq_mV'] == 0)]
    if len(align) > 0:
        ia_mean = align['ia_mA'].mean()
        ib_mean = align['ib_mA'].mean()
        v_align = 3000.0  # mV
        # R = V/I, both in consistent units
        r_est = v_align / ia_mean if ia_mean != 0 else float('inf')
        print(f"\nAlignment phase R identification")
        print(f"---------------------------------")
        print(f"Samples in alignment     : {len(align)}")
        print(f"ia mean                  : {ia_mean:.1f} mA")
        print(f"ib mean                  : {ib_mean:.1f} mA")
        print(f"Estimated R (V/ia_mean)  : {r_est:.2f} ohm")
        
    # Zoomed step response plot — first 50ms for L identification
    align_early = df[df['host_time_s'] < 0.05]
    if len(align_early) > 0:
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(align_early['host_time_s'] * 1000, align_early['ia_mA'], label='ia_mA')
        ax.plot(align_early['host_time_s'] * 1000, align_early['ib_mA'], label='ib_mA')
        ax.set_xlabel('host_time_ms')
        ax.set_ylabel('current (mA)')
        ax.set_title('Step response — first 50ms (alignment phase)')
        ax.legend()
        ax.grid(True)
        out = os.path.join(out_dir, 'sysid_step_response.png')
        fig.savefig(out, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"wrote: {out}")
        
        # Debug: time domain plot of adc_a and vd during sysid
        sysid_df = df[df['host_time_s'] > 5.0].copy()
        sysid_df['t_rel'] = sysid_df['host_time_s'] - sysid_df['host_time_s'].min()
        mask = sysid_df['t_rel'] < 2.0

        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(sysid_df['t_rel'][mask], sysid_df['adc_a'][mask] - 2091, label='adc_a - offset')
        ax.plot(sysid_df['t_rel'][mask], sysid_df['adc_b'][mask] - 2091, label='adc_b - offset')
        ax.plot(sysid_df['t_rel'][mask], sysid_df['vd_mV'][mask] / 100, label='vd_mV / 100')
        ax2 = ax.twinx()
        ax2.plot(sysid_df['t_rel'][mask], sysid_df['vd_mV'][mask], 'k--', alpha=0.3, label='vd_mV')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('ADC counts / scaled vd')
        ax.set_title('Sysid time domain — first 2s')
        ax.legend()
        ax.grid(True)
        save_plot(out_dir / "sysid_time_domain.png")

    print("")
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())