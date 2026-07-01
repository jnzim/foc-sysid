#!/usr/bin/env python3
# py-script/plot_sysid.py
#
# Usage:
#   python3 py-script/plot_sysid.py drive_data/sysid_log.csv
#
# CSV columns (current firmware):
#   host_time_s, frame, t, dt,
#   enc_hi, enc_lo, sysid_f,
#   id_mA, iq_mA, vd_mV, vq_mV, theta_mrad,
#   ia_mA, ib_mA, adc_c, flags, crc, pad

import sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence

# Known motor parameters
R_NOMINAL = 3.3       # ohm  line-to-line DMM measured
L_NOMINAL = 0.00204   # H    datasheet


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
    y = pd.to_numeric(series, errors="coerce").dropna()
    if y.empty:
        return {"mean": 0.0, "std": 0.0, "rms_noise": 0.0,
                "p99_p1": 0.0, "min": 0.0, "max": 0.0, "abs_max": 0.0}
    if len(y) < window:
        noise = y - y.mean()
    else:
        trend = y.rolling(window, center=True, min_periods=1).mean()
        noise = y - trend
    return {
        "mean":      float(y.mean()),
        "std":       float(y.std()),
        "rms_noise": float((noise * noise).mean() ** 0.5),
        "p99_p1":    float(noise.quantile(0.99) - noise.quantile(0.01)),
        "min":       float(y.min()),
        "max":       float(y.max()),
        "abs_max":   float(y.abs().max()),
    }


def stats_text(name, stats, unit):
    return (f"{name}: mean={stats['mean']:.1f} {unit}, "
            f"rms_noise={stats['rms_noise']:.1f} {unit}, "
            f"p99-p1={stats['p99_p1']:.1f} {unit}")


def annotate_stats(lines):
    text = "\n".join(lines)
    plt.gca().text(0.01, 0.99, text,
                   transform=plt.gca().transAxes,
                   va="top", ha="left", fontsize=9,
                   bbox=dict(boxstyle="round", alpha=0.15))


def print_stats_block(title, stats_by_name, unit):
    print(f"\n{title}")
    print("-" * len(title))
    for name, stats in stats_by_name.items():
        print(f"{name:8s} "
              f"mean={stats['mean']:9.2f} {unit}, "
              f"std={stats['std']:9.2f} {unit}, "
              f"rms_noise={stats['rms_noise']:9.2f} {unit}, "
              f"p99-p1={stats['p99_p1']:9.2f} {unit}, "
              f"min={stats['min']:9.2f}, max={stats['max']:9.2f}")


def plot_bode(df, out_dir):
    # Use only sysid samples — exclude alignment (vd=3000) and post-sysid
    d = df[df['flags'] == '0x0001'].copy()
    d = d[d['sysid_f'] > 0]
    d = d[d['dt'] > 0]
    d = d[d['vd_mV'].abs() < 1500]

    if len(d) < 100:
        print("WARNING: too few valid sysid samples for Bode")
        return

    t   = d['host_time_s'].values
    vd  = d['vd_mV'].values / 1000.0
    id_ = d['id_mA'].values / 1000.0

    fs = 1.0 / np.mean(np.diff(t))
    print(f"\nBode: {len(d)} samples, fs={fs:.1f} Hz")
    print(f"  vd  : {vd.min()*1000:.1f} to {vd.max()*1000:.1f} mV")
    print(f"  id  : {id_.min()*1000:.1f} to {id_.max()*1000:.1f} mA")
    print(f"  freq: {d['sysid_f'].min():.1f} to {d['sysid_f'].max():.1f} Hz")
    print(f"  corr: {np.corrcoef(vd, id_)[0,1]:.4f}")

    nperseg = int(fs * 2)

    f_csd, Piv = csd(vd, id_, fs=fs, nperseg=nperseg)
    f_csd, Pvv = welch(vd,      fs=fs, nperseg=nperseg)
    f_coh, Cxy = coherence(vd, id_, fs=fs, nperseg=nperseg)

    H   = Piv / (Pvv + 1e-10)
    mag = 20 * np.log10(np.abs(H) + 1e-12)
    phi = np.degrees(np.angle(H))

    band     = (f_csd >= 1.0) & (f_csd <= 1000.0)
    coh_good = band & (Cxy >= 0.5)

    # Extract R and L from magnitude
    mag_dc  = np.mean(mag[band][5:20])
    idx_3db = np.argmin(np.abs(mag[band] - (mag_dc - 3.0)))
    fc_meas = f_csd[band][idx_3db]
    L_meas  = R_NOMINAL / (2 * np.pi * fc_meas)
    R_meas  = 1.0 / (10 ** (mag_dc / 20))
    mag_3db = mag_dc - 3.0

    print(f"\nBode extraction:")
    print(f"  mag_dc  = {mag_dc:.2f} dB  (expect {20*np.log10(1/R_NOMINAL):.2f} dB for R={R_NOMINAL}Ω)")
    print(f"  R_meas  = {R_meas:.3f} ohm  (nominal {R_NOMINAL})")
    print(f"  fc_meas = {fc_meas:.1f} Hz")
    print(f"  L_meas  = {L_meas*1000:.3f} mH  (nominal {L_NOMINAL*1000:.2f})")

    # Theory overlay
    f_th  = np.logspace(np.log10(1.0), np.log10(1000.0), 500)
    w_th  = 2 * np.pi * f_th
    H_th  = 1.0 / (R_NOMINAL + 1j * w_th * L_NOMINAL)
    mag_th = 20 * np.log10(np.abs(H_th))
    phi_th = np.degrees(np.angle(H_th))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

    # Magnitude
    ax1.semilogx(f_csd[band], mag[band], 'b.-', label='Measured')
    ax1.semilogx(f_th, mag_th, 'r--',
                 label=f'Theory R={R_NOMINAL}Ω L={L_NOMINAL*1000:.2f}mH')
    ax1.axhline(mag_3db, color='gray', linestyle=':', alpha=0.7,
                label=f'-3dB = {mag_3db:.1f} dB')
    ax1.axvline(fc_meas, color='green', linestyle=':', alpha=0.7,
                label=f'fc={fc_meas:.0f}Hz → L={L_meas*1000:.2f}mH')
    ax1.plot(fc_meas, mag_3db, 'go', markersize=8)
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Current Plant Bode — id/Vd  (theta=0, rotor locked)')
    ax1.legend(fontsize=8)
    ax1.grid(True, which='both')

    # Phase
    ax2.semilogx(f_csd[coh_good], phi[coh_good], 'b.-', label='Measured (coh≥0.5)')
    ax2.semilogx(f_th, phi_th, 'r--', label='Theory')
    ax2.set_ylabel('Phase (deg)')
    ax2.set_ylim([-100, 10])
    ax2.legend()
    ax2.grid(True, which='both')

    # Coherence
    ax3.semilogx(f_coh[band], Cxy[band], 'g.-', label='Coherence')
    ax3.axhline(0.5, color='r', linestyle='--', label='0.5 threshold')
    ax3.axhline(0.8, color='orange', linestyle='--', label='0.8 threshold')
    ax3.set_ylabel('Coherence')
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_ylim([0, 1.1])
    ax3.legend()
    ax3.grid(True, which='both')

    save_plot(out_dir / "bode_plot.png")


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

    require_columns(df, [
        "host_time_s", "frame", "t", "dt",
        "enc_hi", "enc_lo", "sysid_f",
        "id_mA", "iq_mA", "vd_mV", "vq_mV", "theta_mrad",
        "ia_mA", "ib_mA", "adc_c", "flags",
    ])

    x = df["host_time_s"]

    # -------------------------------------------------------------------------
    # Alignment phase — scaling ground truth
    # Use raw unfiltered data — alignment has vd=3000mV
    # -------------------------------------------------------------------------
    align = df[(df['vd_mV'].abs() > 2900) & (df['vq_mV'] == 0) & (df['flags'] == '0x0001')]
    if len(align) > 0:
        ia_mean = align['ia_mA'].mean()
        ib_mean = align['ib_mA'].mean()
        id_mean = align['id_mA'].mean()
        v_align = align['vd_mV'].mean()
        expected_id = v_align / R_NOMINAL
        print(f"\nAlignment phase scaling check")
        print(f"-----------------------------")
        print(f"Samples               : {len(align)}")
        print(f"vd mean               : {v_align:.1f} mV")
        print(f"ia mean               : {ia_mean:.1f} mA")
        print(f"ib mean               : {ib_mean:.1f} mA")
        print(f"id mean               : {id_mean:.1f} mA")
        print(f"expected id (vd/R_ll) : {expected_id:.1f} mA  (R_ll={R_NOMINAL}Ω)")
        print(f"scaling error         : {id_mean/expected_id:.3f}x  (1.0 = perfect)")
    else:
        print("\nWARNING: no alignment samples found (vd_mV > 2900)")

    # -------------------------------------------------------------------------
    # Stats
    # -------------------------------------------------------------------------
    id_stats = noise_stats(df["id_mA"])
    iq_stats = noise_stats(df["iq_mA"])
    vd_stats = noise_stats(df["vd_mV"])
    ia_stats = noise_stats(df["ia_mA"])
    ib_stats = noise_stats(df["ib_mA"])

    print_stats_block("dq current stats", {"id": id_stats, "iq": iq_stats}, "mA")
    print_stats_block("phase current stats", {"ia": ia_stats, "ib": ib_stats}, "mA")
    print_stats_block("vd command stats", {"vd": vd_stats}, "mV")

    # -------------------------------------------------------------------------
    # D/Q currents
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["id_mA"], label="id_mA")
    plt.plot(x, df["iq_mA"], label="iq_mA")
    annotate_stats([stats_text("id", id_stats, "mA"),
                    stats_text("iq", iq_stats, "mA")])
    plt.xlabel("host_time_s")
    plt.ylabel("current (mA)")
    plt.title("SysID d/q currents")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dq_current.png")

    # -------------------------------------------------------------------------
    # Phase currents ia / ib
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(x, df["ia_mA"], label="ia_mA")
    plt.plot(x, df["ib_mA"], label="ib_mA")
    annotate_stats([stats_text("ia", ia_stats, "mA"),
                    stats_text("ib", ib_stats, "mA")])
    plt.xlabel("host_time_s")
    plt.ylabel("current (mA)")
    plt.title("SysID phase currents ia / ib")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_phase_currents.png")

    # -------------------------------------------------------------------------
    # Voltage command
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
    # Sweep frequency
    # -------------------------------------------------------------------------
    plt.figure(figsize=(12, 4))
    plt.plot(x, df["sysid_f"], label="sysid_f (Hz)")
    plt.xlabel("host_time_s")
    plt.ylabel("frequency (Hz)")
    plt.title("SysID chirp frequency vs time")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_freq.png")

    # -------------------------------------------------------------------------
    # Encoder position
    # -------------------------------------------------------------------------
    df['encoder'] = (df['enc_hi'].astype(np.int32) << 16) | \
                    (df['enc_lo'].astype(np.uint16).astype(np.int32))
    plt.figure(figsize=(12, 4))
    plt.plot(x, df["encoder"], label="encoder counts")
    plt.xlabel("host_time_s")
    plt.ylabel("counts")
    plt.title("SysID encoder position")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_encoder.png")

    # -------------------------------------------------------------------------
    # dt
    # -------------------------------------------------------------------------
    dt_stats = noise_stats(df["dt"])
    plt.figure(figsize=(12, 4))
    plt.plot(x, df["dt"], label="dt")
    annotate_stats([stats_text("dt", dt_stats, "ticks"),
                    f"median={df['dt'].median():.1f} ticks"])
    plt.xlabel("host_time_s")
    plt.ylabel("STM sample delta")
    plt.title("SysID SPI sample delta")
    plt.grid(True)
    plt.legend()
    save_plot(out_dir / "sysid_dt.png")

    # -------------------------------------------------------------------------
    # Bode plot
    # -------------------------------------------------------------------------
    plot_bode(df, out_dir)

    print("\ndone")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())