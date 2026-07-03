#!/usr/bin/env python3
"""
step.py — CL step response plotter
Usage: python3 step.py <csv_file>
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if len(sys.argv) < 2:
    print("Usage: python3 step.py <csv_file>")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])

# Scale
df['iq_A']      = df['iq_mA'] / 1000.0
df['id_A']      = df['id_mA'] / 1000.0
df['vq_V']      = df['vq_mV'] / 1000.0
df['vd_V']      = df['vd_mV'] / 1000.0
df['ia_A']      = df['ia_mA'] / 1000.0
df['ib_A']      = df['ib_mA'] / 1000.0

# iq_cmd — present if sysid_f column was repurposed
if 'iq_cmd_mA' in df.columns:
    df['iq_cmd_A'] = df['iq_cmd_mA'] / 1000.0
elif 'sysid_f' in df.columns:
    df['iq_cmd_A'] = df['sysid_f'] / 1000.0
else:
    df['iq_cmd_A'] = float('nan')

# Time axis
if 'host_time_s' in df.columns:
    t = df['host_time_s'] - df['host_time_s'].iloc[0]
else:
    t = df.index / 10000.0

# Clip to 3s
PLOT_WINDOW = 1.0
mask = t <= PLOT_WINDOW
t   = t[mask]
df  = df[mask]

fig = plt.figure(figsize=(12, 9))
fig.suptitle('CL Step Response', fontsize=13)
gs = gridspec.GridSpec(3, 1, hspace=0.45)

# ── dq Currents ────────────────────────────────────────────────────────────────
ax0 = fig.add_subplot(gs[0])
ax0.plot(t, df['iq_cmd_A'], label='iq_cmd',  linewidth=1.0, linestyle='--', color='green')
ax0.plot(t, df['iq_A'],     label='iq_meas', linewidth=0.8, color='steelblue')
ax0.plot(t, df['id_A'],     label='id_meas', linewidth=0.8, alpha=0.6, color='orange')
ax0.set_ylabel('Current (A)')
ax0.set_title('dq Currents')
ax0.legend(loc='upper right')
ax0.grid(True, alpha=0.3)

# ── PI Output ──────────────────────────────────────────────────────────────────
ax1 = fig.add_subplot(gs[1])
ax1.plot(t, df['vq_V'], label='vq_cmd', linewidth=0.8, color='steelblue')
ax1.plot(t, df['vd_V'], label='vd_cmd', linewidth=0.8, alpha=0.6, color='orange')
ax1.set_ylabel('Voltage (V)')
ax1.set_title('PI Output')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# ── Phase Currents ─────────────────────────────────────────────────────────────
ax2 = fig.add_subplot(gs[2])
ax2.plot(t, df['ia_A'], label='ia', linewidth=0.8, color='steelblue')
ax2.plot(t, df['ib_A'], label='ib', linewidth=0.8, color='orange')
ax2.set_ylabel('Current (A)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Phase Currents')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

for ax in (ax0, ax1, ax2):
    ax.set_xlim(0, PLOT_WINDOW)

plt.savefig('step_response.png', dpi=150, bbox_inches='tight')
print("Saved step_response.png")
plt.show()