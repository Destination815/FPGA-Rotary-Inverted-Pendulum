from model_common import *
import hashlib,json,time,numpy as np
from scipy.signal import periodogram,butter,sosfiltfilt
lib=library(A/"slew/probe.cpp",A/"slew");dp=C.POINTER(C.c_double);lib.simulate.argtypes=[dp,dp,C.c_int,dp,dp]
conditions=json.loads((A/"inputs/bang_conditions.json").read_text())
rows=[];beg=time.monotonic();rms=lambda x:float(np.sqrt(np.mean(x*x)));sos=butter(2,[2,15],fs=1000,btype='bandpass',output='sos')
for v in [12,16]:
 for ci,c in enumerate(conditions):
  p=np.array(c['parameters']);cfg=np.array(c['config']);cfg[11]=v;N=15000;tr=np.zeros((N,14));out=np.zeros(2);lib.simulate(p.ctypes.data_as(dp),cfg.ctypes.data_as(dp),N,out.ctypes.data_as(dp),tr.ctypes.data_as(dp));n=int(out[1]);z=tr[:n];late=z[max(0,n-5000):];m=(z[:,0]>=3)&(z[:,0]<=4);post=(z[:,0]>=4)&(z[:,0]<7);row=dict(variant=v,condition=ci,model=c['model'],case=c['case'],fault=int(out[0]),n=n,theta_rms=rms(late[:,4]),pwm_rms=rms(late[:,5]),velocity_rms=rms(late[:,7]),velocity_2to15=rms(sosfiltfilt(sos,late[:,7])),endpoint_3to4_max=float(max(abs(z[m,2])))if m.any()else 999,position_final_rms=rms(late[:,2]),postkick_peak=float(max(abs(z[post,4])))if post.any()else 999)
  rows.append(row)
  if ci in [0,3,12,18]and v in [12,16]:np.savez_compressed(D/f'v{v}_c{ci}.npz',data=z)
 print('variant',v,'faults',sum(x['fault']>0 for x in rows if x['variant']==v),flush=True)
base=rows[:len(conditions)];summary=[]
for v in [12,16]:
 a=[r for r in rows if r['variant']==v];same=[(b,r)for b,r in zip(base,a)if not b['fault']and not r['fault']];entry=dict(variant=v,newfault=sum(not b['fault']and r['fault']for b,r in zip(base,a)),faults=sum(r['fault']>0 for r in a))
 for k in ['theta_rms','pwm_rms','velocity_rms','velocity_2to15','endpoint_3to4_max','position_final_rms','postkick_peak']:entry[k]=dict(mean=[float(np.mean([b[k]for b,r in same])),float(np.mean([r[k]for b,r in same]))],worse=sum(r[k]>b[k]*1.02 for b,r in same))
 summary.append(entry)
(D/'SLEW_RESULT.json').write_text(json.dumps(dict(rows=rows,conditions=conditions,summary=summary,simulations=len(rows),elapsed_s=time.monotonic()-beg,scope='Exploratory finite architecture/gain screening on calibrated oscillation model plus uncalibrated cross-models, not physical validation; initial state/perturbations prescribed. No production source changes.'),indent=2)+'\n');print(json.dumps(summary,indent=2))
