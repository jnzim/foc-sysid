import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df = pd.read_csv("drive_data/latest/sysid_log.csv")
df = df[df["flags"] != "0x1234"].copy().reset_index(drop=True)

df["pos_cmd_mrad"] = df["enc_hi_raw"]
df["pos_meas_mrad"] = df["enc_lo_raw"]
df["gen_vel_crs"] = df["sysid_f"]
df["vel_fbk_crs"] = df["id_mA"]
df["iq_cmd_mA"] = df["iq_mA"]
df["i_q_meas_mA"] = df["vd_mV"]
df["vq_mV"] = df["vq_mV"]
df["flags_num"] = df["flags"].astype(str).apply(lambda x: int(x, 0))

t = df["t"].to_numpy() / 20000.0  # 20kHz sample seq -> seconds

# Linear stage: 2mm screw pitch, 8192 counts/rev -> mm per rad = pitch / (2*pi)
SCREW_PITCH_MM = 2.0
MM_PER_RAD = SCREW_PITCH_MM / (2.0 * np.pi)

df["pos_cmd_mm"]  = (df.pos_cmd_mrad  / 1000.0) * MM_PER_RAD
df["pos_meas_mm"] = (df.pos_meas_mrad / 1000.0) * MM_PER_RAD
df["gen_vel_mm_s"] = (df.gen_vel_crs / 100.0) * MM_PER_RAD
df["vel_fbk_mm_s"] = (df.vel_fbk_crs / 100.0) * MM_PER_RAD

print("n =", len(df))
print("duration (s):", t[-1] - t[0])
print("pos_cmd range (mm):", df.pos_cmd_mm.min(), df.pos_cmd_mm.max())
print("pos_meas range (mm):", df.pos_meas_mm.min(), df.pos_meas_mm.max())
print("flags value counts:")
print(df["flags_num"].value_counts())

fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

axes[0].plot(t, df.pos_cmd_mm, label="pos_cmd (generator)", lw=1)
axes[0].plot(t, df.pos_meas_mm, label="pos_meas (encoder)", lw=1)
axes[0].set_ylabel("Position (mm)")
axes[0].legend(loc="lower right")
axes[0].grid(alpha=0.3)
axes[0].set_title("M2 trapezoidal generator bench test")

axes[1].plot(t, df.gen_vel_mm_s, label="generator vel_cmd", lw=1)
axes[1].plot(t, df.vel_fbk_mm_s, label="vel_fbk (encoder)", lw=1)
axes[1].set_ylabel("Velocity (mm/s)")
axes[1].legend(loc="lower right")
axes[1].grid(alpha=0.3)

axes[2].plot(t, df.iq_cmd_mA, label="iq_cmd", lw=1)
axes[2].plot(t, df.i_q_meas_mA, label="i_q_meas", lw=0.7, alpha=0.7)
axes[2].set_ylabel("Current (mA)")
axes[2].legend(loc="lower right")
axes[2].grid(alpha=0.3)

axes[3].plot(t, df.flags_num, drawstyle="steps-post", color="black", lw=1)
axes[3].set_ylabel("stage flag")
axes[3].set_xlabel("time (s)")
axes[3].set_yticks([0, 1, 2])
axes[3].set_yticklabels(["ALIGN", "RUN", "IDLE"])
axes[3].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("drive_data/latest/m2_trapgen_plot.png", dpi=130)
print("saved drive_data/latest/m2_trapgen_plot.png")
