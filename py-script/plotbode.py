import csv
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

telem_file = sys.argv[1] if len(sys.argv) > 1 else "/home/jz/trajectory-streamer/docs/chirp_telem.csv"

# ── Load telem ────────────────────────────────────────────────────────────────
rows = []
with open(telem_file) as f:
    for row in csv.DictReader(f):
        rows.append({c: float(row[c]) for c in ['t', 'pos_cmd', 'pos_fbk']})

t       = np.array([r['t'] / 1000.0 for r in rows])   # ms → s
pos_cmd = np.array([r['pos_cmd']     for r in rows])
pos_fbk = np.array([r['pos_fbk']     for r in rows])

# ── Resample to uniform 1kHz grid ─────────────────────────────────────────────
dt   = 0.001
t_u  = np.arange(t[0], t[-1], dt)
cmd_u = np.interp(t_u, t, pos_cmd)
fbk_u = np.interp(t_u, t, pos_fbk)

# ── FFT ───────────────────────────────────────────────────────────────────────
n    = len(t_u)
fs   = 1.0 / dt                          # 1000 Hz
freq = np.fft.rfftfreq(n, d=dt)

CMD  = np.fft.rfft(cmd_u)
FBK  = np.fft.rfft(fbk_u)

# Avoid divide by zero
eps = 1e-12
H   = FBK / (CMD + eps)

magnitude_db = 20 * np.log10(np.abs(H) + eps)
phase_deg    = np.angle(H, deg=True)

# ── Frequency range of interest ───────────────────────────────────────────────
mask = (freq >= 0.1) & (freq <= 250.0)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

ax1.semilogx(freq[mask], magnitude_db[mask], color='steelblue', linewidth=1)
ax1.axhline(-3, color='red', linestyle='--', alpha=0.5, label='-3 dB')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title('Bode Plot — pos_fbk / pos_cmd')
ax1.legend()
ax1.grid(True, which='both', alpha=0.3)
ax1.set_ylim(-60, 10)

ax2.semilogx(freq[mask], phase_deg[mask], color='green', linewidth=1)
ax2.axhline(-180, color='red', linestyle='--', alpha=0.5, label='-180°')
ax2.set_ylabel('Phase (deg)')
ax2.set_xlabel('Frequency (Hz)')
ax2.legend()
ax2.grid(True, which='both', alpha=0.3)
ax2.set_ylim(-360, 90)

fig.tight_layout()
out = telem_file.replace('chirp_telem.csv', 'bode.png')
plt.savefig(out, dpi=150)
print(f"Done — {out}")