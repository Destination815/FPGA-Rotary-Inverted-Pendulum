"""Scientific comparison of complete RTL traces; no interpolation or data filtering."""
from pathlib import Path
import argparse,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--start',type=float,default=6);a=p.parse_args()
fig,axs=plt.subplots(4,1,figsize=(11,9),sharex=True)
for label,path,color in [('R41T2',a.baseline,'#888888'),('R43 selected',a.candidate,'#087eaa')]:
    f=path/'trace.npz'
    if not f.exists():f=path/'case/trace.npz'
    z=np.load(f)['data'];z=z[z[:,0]>=a.start];t=z[:,0]
    axs[0].plot(t,np.degrees(z[:,3]),label=label,color=color,lw=.9)
    if label=='R43 selected':axs[0].plot(t,np.degrees(z[:,11]),label='Reference',color='#ca6817',ls='--',lw=1)
    axs[1].plot(t,np.degrees(z[:,4]),color=color,lw=.8)
    axs[2].plot(t,np.degrees(np.arctan2(np.sin(z[:,1]),np.cos(z[:,1]))),color=color,lw=.8)
    axs[3].plot(t,z[:,9]*100,color=color,lw=.7)
for ax,label in zip(axs,['Arm position (deg)','Arm velocity (deg/s)','Pendulum (deg)','PWM (%)']):ax.set_ylabel(label);ax.grid(alpha=.2)
axs[0].legend(ncol=3);axs[0].set_title('Complete RTL with declared model; physical hardware performance remains separate')
axs[-1].set_xlabel('Session time (s)');fig.tight_layout();fig.savefig(a.out,dpi=140);plt.close(fig)
