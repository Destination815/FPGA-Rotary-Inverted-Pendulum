#!/usr/bin/env python3
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--out');args=p.parse_args()
rows=list(csv.DictReader(open(args.csv)))
if not rows:raise SystemExit('Empty log')
a=lambda key:np.array([float(r[key]) for r in rows])
fig,axes=plt.subplots(4,1,figsize=(12,9),sharex=True)
if 'ms' in rows[0]:
 t=a('ms')/1000;th=a('theta');ph=a('phi');ref=a('reference');vel=a('dphi');rv=a('ref_velocity');u=a('u');state=a('state')
else:
 t=a('time_s');th=(a('theta_rad')+np.pi)%(2*np.pi)-np.pi;ph=a('phi_rad');ref=a('ref_rad');vel=a('dphi_rad_s');rv=a('ref_velocity_rad_s');u=a('u_permille');state=a('state')
axes[0].plot(t,np.rad2deg(th));axes[0].set_ylabel('Pendulum (deg)')
axes[1].plot(t,np.rad2deg(ph),label='actual');axes[1].plot(t,np.rad2deg(ref),label='reference');axes[1].set_ylabel('Arm (deg)');axes[1].legend()
axes[2].plot(t,vel,label='actual');axes[2].plot(t,rv,label='reference');axes[2].set_ylabel('Arm speed (rad/s)');axes[2].legend()
axes[3].plot(t,u,label='PWM permille');axes[3].set_ylabel('Command');axes[3].set_xlabel('Time (s)')
for ax in axes:ax.grid(alpha=.3)
fig.suptitle(Path(args.csv).stem+' — simulation' if 'time_s' in rows[0] else Path(args.csv).stem+' — board telemetry')
fig.tight_layout();out=args.out or str(Path(args.csv).with_suffix('.png'));fig.savefig(out,dpi=140);print(out)
