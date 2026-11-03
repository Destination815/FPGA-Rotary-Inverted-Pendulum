from model_common import *
import json,numpy as np,time
from scipy.signal import butter,sosfiltfilt
lib=library(A/"sine/probe.cpp",A/"sine");dp=C.POINTER(C.c_double);ip=C.POINTER(C.c_longlong);lib.simulate_sine.argtypes=[dp,dp,ip,C.c_int,C.c_int,dp,dp]
settings=json.loads((A/'inputs/SETTINGS.json').read_text());p0=np.array(list(json.loads((A/'inputs/models.json').read_text()).values())[0]);p0[9]*=.5;rows=[];beg=time.monotonic();sos=butter(2,[2,30],fs=1000,btype='bandpass',output='sos');rms=lambda x:float(np.sqrt(np.mean(x*x)))
for pg in [1,1.5,2]:
 for si,st in enumerate(settings):
  ref=np.loadtxt(A/'inputs'/f'ref{si}.dat',dtype=np.int64)
  for scale in [.3,.45]:
   for cap in [150,350]:
    p=p0.copy();cfg=np.array([0,.04,pg,1,1,cap,np.radians(-.084),.04,1,2,.04,4,6,1,1,1.2,scale],float);tr=np.zeros((len(ref),12));out=np.zeros(2)
    lib.simulate_sine(p.ctypes.data_as(dp),cfg.ctypes.data_as(dp),ref.ctypes.data_as(ip),len(ref),st['exit'],out.ctypes.data_as(dp),tr.ctypes.data_as(dp));n=int(out[0]);z=tr[6000:min(n,st['exit'])];m={}
    if len(z)>500:
     deg=180/np.pi;v=z[:,2]*deg;phi=z[:,1]*deg;theta=z[:,9]*deg;vrip=sosfiltfilt(sos,v);times=np.arange(len(z))*.001;H=np.stack([np.ones(len(z)),np.sin(2*np.pi*times/st['period']),np.cos(2*np.pi*times/st['period'])],axis=1);coef=np.linalg.lstsq(H,phi,rcond=None)[0];exitdata=tr[st['exit']:n]
     m=dict(theta_rms=rms(theta),theta_true_rms=rms(z[:,0]*deg),velocity_error_rms=rms((z[:,2]-z[:,4])*deg),velocity_2to30=rms(vrip),position_error_rms=rms((z[:,1]-z[:,3])*deg),pwm_rms=rms(z[:,5]),amplitude=float(np.linalg.norm(coef[1:])),center=float(coef[0]),extrema=[float(min(phi)),float(max(phi))],exit_theta_peak=float(max(abs(exitdata[:,9]*deg)))if len(exitdata)else None,exit_position_peak=float(max(abs(exitdata[:,1]*deg)))if len(exitdata)else None)
    rows.append(dict(pg=pg,speed=[15,30,45,60][si],scale=scale,cap=cap,complete=bool(out[1]),n=n,metrics=m))
    if si==3 and cap==350:np.savez_compressed(D/f'pg{pg}_scale{scale}.npz',data=tr[:n:10])
 rr=[r for r in rows if r['pg']==pg];print(pg,'complete',sum(r['complete']for r in rr),len(rr),'theta',np.mean([r['metrics'].get('theta_rms',999)for r in rr]),flush=True)
 (D/'RESULT.json').write_text(json.dumps(dict(rows=rows,simulations=len(rows),elapsed_s=time.monotonic()-beg,scope='Finite position-gain-only sweep on ordinary-feature-calibrated M03 family with real estimator; this mapping is not independently validated for sine. Fixed-cap150/350 engineering stress; exact255 A observer variant with all other gains/shape/rate unchanged. Curves10ms preview, metrics1ms.'),indent=2))
