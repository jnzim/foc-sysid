import csv
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

profile_file = sys.argv[1] if len(sys.argv) > 1 else "/home/jz/trajectory-streamer/build/profile.csv"
telem_file   = sys.argv[2] if len(sys.argv) > 2 else "/home/jz/trajectory-streamer/build/telem.csv"

def load_csv(path, cols):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({c: float(row[c]) for c in cols})
    return rows

profile = load_csv(profile_file, ['sample', 't', 'pos', 'vel'])
telem   = load_csv(telem_file,   ['t', 'pos_cmd', 'pos_fbk', 'vel_fbk', 'samples_consumed'])

profile_t   = [s['t']   for s in profile]
profile_pos = [s['pos'] for s in profile]
profile_vel = [s['vel'] for s in profile]

telem_t       = [s['t'] / 1000.0 for s in telem]
telem_pos_cmd = [s['pos_cmd']    for s in telem]
telem_pos_fbk = [s['pos_fbk']   for s in telem]
telem_vel_fbk = [s['vel_fbk']   for s in telem]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

ax1.plot(profile_t, profile_pos,   color='steelblue', label='profile cmd',   linewidth=2)
ax1.plot(telem_t,   telem_pos_cmd, color='tomato',    label='telem cmd',     linewidth=1, linestyle='--', alpha=0.8)
ax1.plot(telem_t,   telem_pos_fbk, color='green',     label='plant pos fbk', linewidth=1, alpha=0.8)
ax1.set_ylabel('position (counts)')
ax1.set_title('trajectory vs telem')
ax1.legend()

ax2.plot(profile_t, profile_vel,   color='steelblue', label='profile vel',   linewidth=2)
ax2.plot(telem_t,   telem_vel_fbk, color='green',     label='plant vel fbk', linewidth=1, alpha=0.8)
ax2.set_xlabel('time (s)')
ax2.set_ylabel('velocity (counts/s)')
ax2.legend()

fig.tight_layout()
plt.savefig('/home/jz/trajectory-streamer/docs/profile_plot.png', dpi=150)
print("Done — profile_plot.png")