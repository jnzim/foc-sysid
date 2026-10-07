#!/usr/bin/env python3
"""Observer vs encoder boxcar, from one CL_VEL_CHIRP capture.

    python3 py-script/plot_observer_vs_boxcar.py [csv]

Requires firmware with OBSERVER_ENABLE=1 (drive repo), which logs both
estimates in the same frame for SYSID_TEST_CL_VEL_CHIRP:

    sysid_f      vel_cmd                      [mrad/s]
    iq_cmd_mA    boxcar  velocity  <- what the loop currently uses
    enc_lo_raw   observer velocity <- same units, so they difference directly

Both columns are counts/s divided by VEL_TELEM_DIV, hence SCALE below.
"""
import csv, sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence

BLUE, ORANGE, INK, MUTED, GRID, SURF = "#3B6FB6", "#E8833A", "#1a1a1a", "#6b6b6b", "#e4e4e1", "#fcfcfb"
SCALE = 8.0 / (8192.0 / (2*np.pi))      # VEL_TELEM_DIV / COUNTS_PER_RAD

rows=[r for r in csv.DictReader(open(sys.argv[1] if len(sys.argv)>1 else "/home/jz/foc-sysid/drive_data/latest/sysid_log.csv")) if r["flags"]=="0x0001"]
t=np.array([float(r["t"]) for r in rows])*50e-6
k=np.concatenate([[True],np.diff(t)>0]); t=t[k]-t[k][0]
cmd=np.array([float(r["sysid_f"]) for r in rows])[k]/1000.0
box=np.array([float(r["iq_cmd_mA"]) for r in rows])[k]*SCALE
obs=np.array([float(r["enc_lo_raw"]) for r in rows])[k]*SCALE
fs=1.0/np.median(np.diff(t)); g=np.arange(t[0],t[-1],1/fs)
cmd_u,box_u,obs_u = (np.interp(g,t,v) for v in (cmd,box,obs))

def frf(x,y,n=16384):
    f,Pxy=csd(x-x.mean(),y-y.mean(),fs=fs,nperseg=n)
    f,Pxx=welch(x-x.mean(),fs=fs,nperseg=n)
    f,C=coherence(x-x.mean(),y-y.mean(),fs=fs,nperseg=n)
    H=Pxy/Pxx
    return f,20*np.log10(np.abs(H)),np.degrees(np.unwrap(np.angle(H))),C
f,mb,pb,cb = frf(cmd_u,box_u)
_,mo,po,_  = frf(cmd_u,obs_u)
sel=(f>=1)&(f<=200)&(cb>=0.8)

fig=plt.figure(figsize=(13,8.6),facecolor=SURF)
gs=fig.add_gridspec(2,2,height_ratios=[1,1.25],hspace=0.42,wspace=0.22)
fig.suptitle("Velocity estimate: 3-state observer vs 20-tap moving average",
             x=0.015,ha="left",fontsize=14,color=INK,weight="bold")
fig.text(0.015,0.925,"CL_VEL_CHIRP 0.5-200Hz, stage loaded.  Observer runs in parallel; "
         "the velocity loop still uses the boxcar.",ha="left",fontsize=10,color=MUTED)

def style(a):
    a.set_facecolor(SURF)
    for s in ("top","right"): a.spines[s].set_visible(False)
    for s in ("left","bottom"): a.spines[s].set_color(GRID)
    a.tick_params(colors=MUTED,labelsize=9); a.grid(color=GRID,lw=.8); a.set_axisbelow(True)

# time zoom, late in the sweep = high frequency
i0=int(len(g)*0.80); w=int(fs*0.035)
ax=fig.add_subplot(gs[0,:]); style(ax)
ax.plot(g[i0:i0+w]-g[i0],cmd_u[i0:i0+w],lw=2.4,color=MUTED,alpha=.45)
ax.plot(g[i0:i0+w]-g[i0],box_u[i0:i0+w],lw=1.8,color=BLUE)
ax.plot(g[i0:i0+w]-g[i0],obs_u[i0:i0+w],lw=1.8,color=ORANGE)
ax.set_xlabel("time (s)",color=MUTED,fontsize=9.5); ax.set_ylabel("velocity (rad/s)",color=MUTED,fontsize=9.5)
ax.set_title("35 ms late in the sweep — the observer leads",loc="left",fontsize=11,color=INK)
for lbl,c,yo in (("command",MUTED,0),("boxcar",BLUE,-1),("observer",ORANGE,-2)):
    ax.text(0.985,0.93+yo*0.115,lbl,transform=ax.transAxes,ha="right",color=c,fontsize=10,weight="bold")

ax=fig.add_subplot(gs[1,0]); style(ax)
ax.semilogx(f[sel],mb[sel],lw=1.8,color=BLUE); ax.semilogx(f[sel],mo[sel],lw=1.8,color=ORANGE)
ax.axhline(-3,color=GRID,lw=1,ls=(0,(4,3)))
ax.set_xlabel("frequency (Hz)",color=MUTED,fontsize=9.5); ax.set_ylabel("magnitude (dB)",color=MUTED,fontsize=9.5)
ax.set_title("Closed-loop magnitude",loc="left",fontsize=11,color=INK); ax.set_ylim(-8,2)

ax=fig.add_subplot(gs[1,1]); style(ax)
ax.semilogx(f[sel],pb[sel],lw=1.8,color=BLUE,label="boxcar")
ax.semilogx(f[sel],po[sel],lw=1.8,color=ORANGE,label="observer")
d=po[sel]-pb[sel]
ax.set_xlabel("frequency (Hz)",color=MUTED,fontsize=9.5); ax.set_ylabel("phase (deg)",color=MUTED,fontsize=9.5)
ax.set_title("Closed-loop phase — the whole point",loc="left",fontsize=11,color=INK)
for ff in (40,95):
    i=np.argmin(abs(f[sel]-ff))
    ax.annotate(f"{d[i]:+.0f}° at {ff}Hz",(f[sel][i],po[sel][i]),textcoords="offset points",
                xytext=(6,10),fontsize=9.5,color=ORANGE,weight="bold")
for lbl,c,yo in (("boxcar",BLUE,0),("observer",ORANGE,-1)):
    ax.text(0.02,0.14+yo*0.11,lbl,transform=ax.transAxes,color=c,fontsize=10,weight="bold")

fig.tight_layout(rect=[0,0,1,0.90])
fig.savefig("/home/jz/foc-sysid/drive_data/latest/observer_vs_boxcar.png",dpi=150,facecolor=SURF)
print("phase advantage: %+.1f deg @40Hz, %+.1f deg @95Hz"%(d[np.argmin(abs(f[sel]-40))],d[np.argmin(abs(f[sel]-95))]))
print("boxcar  -3dB: %.1f Hz"%f[sel][np.argmin(abs(mb[sel]+3))])
print("observer -3dB: %.1f Hz"%f[sel][np.argmin(abs(mo[sel]+3))])
