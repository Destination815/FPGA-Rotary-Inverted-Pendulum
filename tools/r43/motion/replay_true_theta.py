from model_common import *
import json,numpy as np,time,hashlib
old=json.loads((P/"evidence/r43/motion/bang_screen.json").read_text());lib=library(A/"fixtures/ordinary_probe.cpp",A/"fixtures");ptr=C.POINTER(C.c_double);lib.simulate.argtypes=[ptr,ptr,C.c_int,ptr,ptr];rms=lambda x:float(np.sqrt(np.mean(x*x)));rows=[];beg=time.monotonic()
for v in [0,12]:
 for ci,c in enumerate(old['conditions']):
  p=np.array(c['parameters']);cfg=np.array(c['config']);cfg[11]=v;N=15000;tr=np.zeros((N,14));out=np.zeros(2);lib.simulate(p.ctypes.data_as(ptr),cfg.ctypes.data_as(ptr),N,out.ctypes.data_as(ptr),tr.ctypes.data_as(ptr));n=int(out[1]);z=tr[:n];late=z[max(0,n-5000):];post=(z[:,0]>=4)&(z[:,0]<7)
  previous=next(x for x in old['rows']if x['variant']==v and x['condition']==ci);measured=rms(late[:,4]);assert measured==previous['theta_rms'] and n==previous['n'] and int(out[0])==previous['fault']
  rows.append(dict(variant=v,condition=ci,model=c['model'],case=c['case'],fault=int(out[0]),n=n,true_theta_rms_deg=rms(late[:,1]),measured_theta_rms_deg=measured,true_theta_postkick_peak_deg=float(max(abs(z[post,1]))),measured_theta_postkick_peak_deg=float(max(abs(z[post,4])))))
summary=[]
for family in [None,0,1,2,3]:
 base=[r for r in rows if r['variant']==0 and(family is None or r['model']==family)];chosen=[r for r in rows if r['variant']==12 and(family is None or r['model']==family)];q=dict(family=family,n=len(base))
 for k in ['true_theta_rms_deg','measured_theta_rms_deg','true_theta_postkick_peak_deg','measured_theta_postkick_peak_deg']:
  q[k]=dict(mean=[float(np.mean([r[k]for r in a]))for a in [base,chosen]],worse_over2pct=sum(y[k]>x[k]*1.02 for x,y in zip(base,chosen)),worse_cases=[y['condition']for x,y in zip(base,chosen)if y[k]>x[k]*1.02])
 summary.append(q)
result=dict(status="PASS_SAME48_RUNS_REANALYZED",rows=rows,summary=summary,elapsed_s=time.monotonic()-beg,scope="Same48 old/selected model conditions, additional model-state analysis, not new validation. Model rigid theta and measured output are distinct; neither is a new physical measurement.")
(D/"TRUE_THETA_ALL.json").write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(summary,indent=2))
