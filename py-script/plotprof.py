#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if len(sys.argv) < 2:
    print("usage: python3 plotprof.py run_000.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1],
    names=['t','pos_cmd','pos_fbk','vel_cmd','vel_fbk','pos_err','vel_err','iq_cmd','i_q_fbk','v_q_cmd','consumed'],
    header=0)

df = df.drop_duplicates(subset='consumed', keep='last')

fig = plt.figure(figsize=(14, 11))
fig.suptitle(sys.argv[1], fontsize=11)
gs = gridspec.GridSpec(4, 1, hspace=0.45)

# Position
ax0 = fig.add_subplot(gs[0])
ax0.plot(df.t, df.pos_cmd, label='pos_cmd', linewidth=1, color='blue')
ax0.plot(df.t, df.pos_fbk, label='pos_fbk', linewidth=1, color='orange')
ax0.set_ylabel('counts', fontweight='bold')
ax0.set_title('Position')
ax0.grid(True, alpha=0.3)

# Position Error — RIGHT AXIS (use CSV column, auto-scaled to show detail)
ax0_err = ax0.twinx()
ax0_err.plot(df.t, df.pos_err, label='pos_err', linewidth=2.5, color='red', alpha=1.0)
ax0_err.set_ylabel('pos_err (counts)', color='red', fontweight='bold')
ax0_err.tick_params(axis='y', labelcolor='red')

# Auto-scale error axis independently
pos_err_min, pos_err_max = df.pos_err.min(), df.pos_err.max()
margin = max(abs(pos_err_max - pos_err_min) * 0.15, 100)
ax0_err.set_ylim([pos_err_min - margin, pos_err_max + margin])

# Combined legend from both axes
lines0, labels0 = ax0.get_legend_handles_labels()
lines0_err, labels0_err = ax0_err.get_legend_handles_labels()
ax0.legend(lines0 + lines0_err, labels0 + labels0_err, fontsize=8, loc='upper left')

# Velocity
ax1 = fig.add_subplot(gs[1], sharex=ax0)
ax1.plot(df.t, df.vel_cmd, label='vel_cmd', linewidth=1, color='blue')
ax1.plot(df.t, df.vel_fbk, label='vel_fbk', linewidth=1, color='orange')
ax1.set_ylabel('counts/s', fontweight='bold')
ax1.set_title('Velocity')
ax1.grid(True, alpha=0.3)

# Velocity Error — RIGHT AXIS (auto-scaled to show detail)
ax1_err = ax1.twinx()
ax1_err.plot(df.t, df.vel_err, label='vel_err', linewidth=2.5, color='red', alpha=1.0)
ax1_err.set_ylabel('vel_err (counts/s)', color='red', fontweight='bold')
ax1_err.tick_params(axis='y', labelcolor='red')

# Auto-scale error axis independently
vel_err_min, vel_err_max = df.vel_err.min(), df.vel_err.max()
vel_err_range = vel_err_max - vel_err_min
if vel_err_range == 0:
    vel_err_range = 100
margin = vel_err_range * 0.15
ax1_err.set_ylim([vel_err_min - margin, vel_err_max + margin])

# Combined legend from both axes
lines1, labels1 = ax1.get_legend_handles_labels()
lines1_err, labels1_err = ax1_err.get_legend_handles_labels()
ax1.legend(lines1 + lines1_err, labels1 + labels1_err, fontsize=8, loc='upper left')

# Current (Q-axis)
ax2 = fig.add_subplot(gs[2], sharex=ax0)
ax2.plot(df.t, df.iq_cmd, color='blue', linewidth=1.5, label='iq_cmd (command)')
ax2.plot(df.t, df.i_q_fbk, color='red', linewidth=1.5, label='i_q_fbk (actual)')
ax2.set_ylabel('Current (mA)', fontweight='bold')
ax2.set_title('Q-axis Current')
ax2.legend(fontsize=8, loc='upper left')
ax2.grid(True, alpha=0.3)

# Auto-scale current
i_max = max(df.iq_cmd.max(), df.i_q_fbk.max())
i_min = min(df.iq_cmd.min(), df.i_q_fbk.min())
i_range = i_max - i_min
if i_range == 0:
    i_range = 1
i_margin = i_range * 0.15
ax2.set_ylim([i_min - i_margin, i_max + i_margin])
ax2.autoscale_view()

# Voltage (Q-axis)
ax3 = fig.add_subplot(gs[3], sharex=ax0)
ax3.plot(df.t, df.v_q_cmd, color='green', linewidth=1.5, label='v_q_cmd (voltage command)')
ax3.set_ylabel('Voltage (mV)', fontweight='bold')
ax3.set_xlabel('Time (ms)', fontweight='bold')
ax3.set_title('Q-axis Voltage')
ax3.legend(fontsize=8, loc='upper left')
ax3.grid(True, alpha=0.3)

# Auto-scale voltage
v_max = df.v_q_cmd.max()
v_min = df.v_q_cmd.min()
v_range = v_max - v_min
if v_range == 0:
    v_range = 1
v_margin = v_range * 0.15
ax3.set_ylim([v_min - v_margin, v_max + v_margin])
ax3.autoscale_view()

out = sys.argv[1].replace('.csv', '.png')
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"saved {out}")
plt.show()