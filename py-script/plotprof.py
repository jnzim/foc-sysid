import csv
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

profile_file = sys.argv[1] if len(sys.argv) > 1 else "/home/jz/trajectory-streamer/docs/profile.csv"
telem_file   = sys.argv[2] if len(sys.argv) > 2 else "/home/jz/trajectory-streamer/docs/telem.csv"

def load_csv(path, cols):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({c: float(row[c]) for c in cols})
    return rows

profile = load_csv(profile_file, ['sample', 't', 'pos', 'vel'])
telem   = load_csv(telem_file,   ['t', 'pos_cmd', 'pos_fbk', 'vel_fbk',
                                   'pos_err', 'i_q_fbk', 'v_q_cmd', 'samples_consumed'])

# ── Telem on STM time base (timestamp_ms - t0, converted to seconds) ─────────
telem_t       = [s['t'] / 1000.0 for s in telem]
telem_pos_cmd = [s['pos_cmd']    for s in telem]
telem_pos_fbk = [s['pos_fbk']   for s in telem]
telem_vel_fbk = [s['vel_fbk']   for s in telem]
telem_pos_err = [s['pos_err']   for s in telem]
telem_i_q_fbk = [s['i_q_fbk'] / 1000.0 for s in telem]   # mA → A
telem_v_q_cmd = [s['v_q_cmd']   for s in telem]

# ── Profile normalized to telem time base — shape reference only ──────────────
profile_t_raw = [s['t'] for s in profile]
profile_pos   = [s['pos'] for s in profile]
profile_vel   = [s['vel'] for s in profile]

if telem_t and profile_t_raw[-1] > 0:
    scale = (telem_t[-1] - telem_t[0]) / profile_t_raw[-1]
    profile_t = [telem_t[0] + t * scale for t in profile_t_raw]
else:
    profile_t = profile_t_raw

fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
ax1, ax2, ax3, ax4 = axes

# ── Position ──────────────────────────────────────────────────────────────────
ax1.plot(profile_t, profile_pos,   color='steelblue', label='profile cmd (shape)',  linewidth=2, alpha=0.5)
ax1.plot(telem_t,   telem_pos_cmd, color='tomato',    label='telem cmd (STM ticks)', linewidth=1, linestyle='--', alpha=0.9)
ax1.plot(telem_t,   telem_pos_fbk, color='green',     label='plant pos fbk',         linewidth=1, alpha=0.9)
ax1.set_ylabel('position (counts)')
ax1.set_title('trajectory vs telem — STM time base')
ax1.legend()

# ── Velocity ──────────────────────────────────────────────────────────────────
ax2.plot(profile_t, profile_vel,   color='steelblue', label='profile vel (shape)',  linewidth=2, alpha=0.5)
ax2.plot(telem_t,   telem_vel_fbk, color='green',     label='plant vel fbk',         linewidth=1, alpha=0.9)
ax2.set_ylabel('velocity (counts/s)')
ax2.legend()

# ── Position error + current ──────────────────────────────────────────────────
ax3.plot(telem_t, telem_pos_err, color='orange', label='pos_err (counts)', linewidth=1)
ax3.set_ylabel('pos error (counts)')
ax3.legend(loc='upper left')
ax3b = ax3.twinx()
ax3b.plot(telem_t, telem_i_q_fbk, color='purple', label='i_q_fbk (A)', linewidth=1, alpha=0.7)
ax3b.set_ylabel('i_q (A)')
ax3b.legend(loc='upper right')

# ── v_q_cmd ───────────────────────────────────────────────────────────────────
ax4.plot(telem_t, telem_v_q_cmd, color='red', label='v_q_cmd (V)', linewidth=1)
ax4.set_ylabel('v_q (V)')
ax4.set_xlabel('time (s) — STM tick_ms')
ax4.legend()

fig.tight_layout()
plt.savefig('/home/jz/trajectory-streamer/docs/profile_plot.png', dpi=150)
print("Done — profile_plot.png")