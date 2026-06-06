#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if len(sys.argv) < 2:
    print("usage: python3 plotprof.py run_000.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1],
    names=['t','pos_cmd','pos_fbk','vel_cmd','vel_fbk','pos_err','iq_cmd','i_q_fbk','consumed'],
    header=0)
df = df.drop_duplicates(subset='consumed', keep='last')

fig = plt.figure(figsize=(14, 10))
fig.suptitle(sys.argv[1], fontsize=11)
gs = gridspec.GridSpec(3, 1, hspace=0.45)

# Position
ax0 = fig.add_subplot(gs[0])
ax0.plot(df.t, df.pos_cmd, label='pos_cmd', linewidth=1, color='blue')
ax0.plot(df.t, df.pos_fbk, label='pos_fbk', linewidth=1, color='orange')
ax0.set_ylabel('counts')
ax0.set_title('Position')
ax0.legend(fontsize=8, loc='upper left')
ax0.grid(True, alpha=0.3)

ax0_err = ax0.twinx()
ax0_err.plot(df.t, df.pos_err, label='pos_err', linewidth=2.5, color='red', alpha=1.0)
ax0_err.set_ylabel('pos_err (counts)', color='red', fontweight='bold')
ax0_err.tick_params(axis='y', labelcolor='red')
pos_err_min, pos_err_max = df.pos_err.min(), df.pos_err.max()
margin = (pos_err_max - pos_err_min) * 0.1 if pos_err_max > pos_err_min else 10
ax0_err.set_ylim([pos_err_min - margin, pos_err_max + margin])

# Velocity
ax1 = fig.add_subplot(gs[1], sharex=ax0)
ax1.plot(df.t, df.vel_cmd, label='vel_cmd', linewidth=1, color='blue')
ax1.plot(df.t, df.vel_fbk, label='vel_fbk', linewidth=1, color='orange')
ax1.set_ylabel('counts/s')
ax1.set_title('Velocity')
ax1.legend(fontsize=8, loc='upper left')
ax1.grid(True, alpha=0.3)

ax1_err = ax1.twinx()
vel_err = df.vel_cmd - df.vel_fbk
ax1_err.plot(df.t, vel_err, label='vel_err', linewidth=2.5, color='red', alpha=1.0)
ax1_err.set_ylabel('vel_err (counts/s)', color='red', fontweight='bold')
ax1_err.tick_params(axis='y', labelcolor='red')
vel_err_min, vel_err_max = vel_err.min(), vel_err.max()
margin = (vel_err_max - vel_err_min) * 0.1 if vel_err_max > vel_err_min else 100
ax1_err.set_ylim([vel_err_min - margin, vel_err_max + margin])

# Current
ax2 = fig.add_subplot(gs[2], sharex=ax0)
ax2.plot(df.t, df.iq_cmd, color='blue', linewidth=1, label='iq_cmd (velocity loop)')
ax2.plot(df.t, df.i_q_fbk, color='red', linewidth=1, label='i_q_fbk (actual)')
ax2.set_ylabel('mA')
ax2.set_xlabel('ms')
ax2.set_title('Q-axis Current')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)
ax2.axhline(y=5000, color='gray', linestyle=':', alpha=0.5)
ax2.axhline(y=-5000, color='gray', linestyle=':', alpha=0.5)

i_max = max(df.iq_cmd.max(), df.i_q_fbk.max())
i_min = min(df.iq_cmd.min(), df.i_q_fbk.min())
margin = max(abs(i_max - i_min) * 0.15, 100)
ax2.set_ylim([i_min - margin, i_max + margin])

out = sys.argv[1].replace('.csv', '.png')
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"saved {out}")
plt.show()