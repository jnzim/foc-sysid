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


def require_columns(df, cols):
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise RuntimeError(f"CSV missing columns: {missing}")


def save_plot(path):
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()
    print(f"wrote: {path}")


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
        ],
    )

    x = df["host_time_s"]

    # -------------------------------------------------------------------------
    # Phase currents
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["ia_mA"], label="ia_mA")
    plt.plot(x, df["ib_mA"], label="ib_mA")
    plt.plot(x, df["ic_mA"], label="ic_mA")
    plt.xlabel("host_time_s")
    plt.ylabel("current (mA)")
    plt.title("SysID phase currents")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_phase_currents.png")

    # -------------------------------------------------------------------------
    # D/Q currents
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["id_mA"], label="id_mA")
    plt.plot(x, df["iq_mA"], label="iq_mA")
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
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["dt"], label="dt")
    plt.xlabel("host_time_s")
    plt.ylabel("STM sample delta")
    plt.title("SysID SPI sample delta")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dt.png")

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())