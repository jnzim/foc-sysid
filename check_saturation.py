import pandas as pd
import numpy as np

df = pd.read_csv("drive_data/latest/sysid_log.csv")
flags = df["flags"].map(lambda v: int(str(v), 0) if isinstance(v, str) else int(v))
df = df[(flags == 1) & (df["dt"] > 0)].copy()
t = df["host_time_s"].to_numpy()
t = t - t[0]

F_START = 0.5
F_END = 200.0
DURATION = 60.0

df = df[t >= 2.0].copy()
t = df["host_time_s"].to_numpy()
t = t - (t[0] - 2.0)

f_inst = F_START * (F_END / F_START) ** (t / DURATION)

mask = (f_inst >= 10) & (f_inst <= 30)
sub = df[mask]

print("time range for 10-30Hz:", t[mask].min(), t[mask].max())
print("iq_mA max abs in window:", sub["iq_mA"].abs().max())
print("vq_mV max abs in window:", sub["vq_mV"].abs().max())
print("iq_mA max abs overall  :", df["iq_mA"].abs().max())
print("vq_mV max abs overall  :", df["vq_mV"].abs().max())
print()
print("--- iq_mA abs stats in dip window ---")
print(sub["iq_mA"].abs().describe())
print()
print("--- vq_mV abs stats in dip window ---")
print(sub["vq_mV"].abs().describe())
