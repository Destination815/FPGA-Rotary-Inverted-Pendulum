from model_common import *
import json,numpy as np,time,hashlib
lib=library(A/"fixtures/position_probe.cpp",A/"fixtures");ptr=C.POINTER(C.c_double);lib.simulate.argtypes=[ptr,ptr,C.c_int,C.c_int,C.c_int,ptr,ptr]
plan=json.loads((A/"inputs/positions.json").read_text());conditions=plan["conditions"];(D/"PLAN.json").write_text(json.dumps(plan,indent=2))
rms=lambda x:float(np.sqrt(np.mean(x*x)));rows=[];beg=time.monotonic()
for policy,variant,anchor in [('255_old_origin',0,0),('255_reanchor',0,1),('R43_reanchor_continuous',12,1)]:
 for c in conditions:
  cfg=np.array(c['cfg']);cfg[0]=variant;cfg[19]=anchor;p=np.array(c['parameters']);N=42000;tr=np.zeros((N,20));out=np.zeros(2);lib.simulate(p.ctypes.data_as(ptr),cfg.ctypes.data_as(ptr),c['speed'],1,N,out.ctypes.data_as(ptr),tr.ctypes.data_as(ptr));z=tr[:int(out[1])];moving=np.flatnonzero(z[:,4]!=0);end=moving[-1]+1 if len(moving)else 0;arr=z[end+3000:end+4000];tail=z[-5000:];target=c['target'];initial=np.degrees(cfg[5]);sg=np.sign(target-initial);e=z[:,2]-target;after=z[end:];settle=np.where(np.maximum.accumulate(abs(after[::-1,2]-target))[::-1]<=5)[0]if len(after)else[]
  r=dict(policy=policy,case=c['case'],target=target,speed=c['speed'],scale=c['scale'],initial_velocity=c['initial_velocity'],fault=int(out[0]),n=len(z),reference_done_s=end*.001,endpoint_3to4_max=float(max(abs(arr[:,2]-target)))if len(arr)==1000 else None,endpoint_3to4_mean=float(np.mean(arr[:,2]-target))if len(arr)else None,tail_phi_mean=float(np.mean(tail[:,2]-target)),tail_phi_pp=float(np.ptp(tail[:,2])),tail_phi_rms=rms(tail[:,2]-target),tail_theta_rms=rms(tail[:,14]),tail_theta_true_rms=rms(tail[:,1]),tail_pwm_rms=rms(tail[:,5]),tail_I=float(np.mean(tail[:,6])),first1s_wrongway_peak=float(max(0,max(-sg*(z[:1000,2]-initial)))),first1s_ref_wrongway_peak=float(max(0,max(-sg*(z[:1000,3]-initial)))),settle5_s=float(settle[0]*.001)if len(settle)else None)
  rows.append(r)
  if c['case']in[0,1,2,12,18]:np.savez_compressed(D/f'{policy}_c{c["case"]}.npz',data=z)
  (D/'RESULT.json').write_text(json.dumps(dict(rows=rows,simulations=len(rows),elapsed_s=time.monotonic()-beg),indent=2))
 rr=rows[-len(conditions):];print(policy,'faults',sum(x['fault']>0 for x in rr),'arrival_fail',sum((x['endpoint_3to4_max']or 999)>5 for x in rr),'arrival',np.mean([x['endpoint_3to4_max']or 999 for x in rr]),'tailDC',np.mean([abs(x['tail_phi_mean'])for x in rr]),'theta',np.mean([x['tail_theta_rms']for x in rr]),flush=True)
