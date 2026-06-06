#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if len(sys.argv) < 2:
    print("usage: python3 plotprof.py run_000.csv")
    sys.exit(1)

df = pd.read_csv(sys.argv[1],
    names=['t','pos_cmd','pos_fbk','vel_cmd','vel_fbk',
           'iq_cmd','i_q_fbk','consumed'],
    header=0)
df = df.drop_duplicates(subset='consumed', keep='last')

fig = plt.figure(figsize=(14, 10))
fig.suptitle(sys.argv[1], fontsize=11)
gs = gridspec.GridSpec(3, 1, hspace=0.45)

# --- Position ---
ax0 = fig.add_subplot(gs[0])
ax0.plot(df.t, df.pos_cmd, label='pos_cmd', linewidth=1)
ax0.plot(df.t, df.pos_fbk, label='pos_fbk', linewidth=1)
ax0.set_ylabel('counts')
ax0.set_title('Position')
ax0.legend(fontsize=8)
ax0.grid(True, alpha=0.3)

# --- Velocity ---
ax1 = fig.add_subplot(gs[1], sharex=ax0)
ax1.plot(df.t, df.vel_cmd, label='vel_cmd', linewidth=1)
ax1.plot(df.t, df.vel_fbk, label='vel_fbk', linewidth=1)
ax1.set_ylabel('counts/s')
ax1.set_title('Velocity')
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

# --- Current (commanded vs actual) ---
ax2 = fig.add_subplot(gs[2], sharex=ax0)
ax2.plot(df.t, df.iq_cmd, color='blue', linewidth=1, label='iq_cmd (velocity loop)')
ax2.plot(df.t, df.i_q_fbk, color='red', linewidth=1, label='i_q_fbk (actual)')
ax2.set_ylabel('mA')
ax2.set_xlabel('ms')
ax2.set_title('Q-axis Current')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)
ax2.axhline(y=5000, color='gray', linestyle=':', alpha=0.5, label='±5A limit')
ax2.axhline(y=-5000, color='gray', linestyle=':', alpha=0.5)

out = sys.argv[1].replace('.csv', '.png')
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"saved {out}")
plt.show()