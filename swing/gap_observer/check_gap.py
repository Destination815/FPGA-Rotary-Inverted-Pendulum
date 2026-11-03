from pathlib import Path
import sys,json,math,time,random
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent));sys.path.insert(0,str(HERE.parents[2]/'simulation_workbench_20261002') if (HERE.parents[2]/'simulation_workbench_20261002').exists() else str(HERE.parents[1]/'models'))
from swing_fixed import IntegerSwing,q
from observer import Observer,sensor,ENC,wrap
from plant import step,nominal
import torque_crosscheck as tq

def run(model,sg,behavior,p=None):
 p=p or nominal();plant=step if model=='effective' else tq.step;o=Observer();c=IntegerSwing(sg);x=np.array([sg*(math.pi-.02),0.,0.,0.]);cap=None;reason='';last=None;mxerr=0.;trace=[]
 for j in range(15000):
  raw=sensor(wrap(x[0]),behavior=behavior,last=last);last=raw;enc=round(x[2]/ENC)
  th,w,ph,v,filtered,ok=o.update(raw,enc,cap is None)
  u=c.update(q(th),q(ph),q(w),q(v))/1000
  if c.mode==2 and cap is None:cap=j*.001
  if o.gap_age:mxerr=max(mxerr,abs(wrap(filtered-x[0])))
  xn=plant(x,u,p,.001,4)
  if not ok:reason=o.fault
  elif abs(xn[2])>math.radians(60):reason='arm60'
  elif cap is not None and abs(wrap(xn[0]))>math.radians(15):reason='fall15'
  elif cap is None and j>=12000:reason='timeout12'
  trace.append([j*.001,wrap(x[0]),th,w,ph,v,u,o.gap_age,raw]);x=xn
  if reason or (cap is not None and j*.001-cap>=2):break
 return dict(model=model,sign=sg,behavior=behavior,passed=bool(cap is not None and not reason),capture_s=cap,reason=reason or 'completed_2s_hold',max_gap_ms=o.max_gap_age,gap_samples=o.bridge_samples,max_gap_angle_error_deg=math.degrees(mxerr)),trace
if __name__=='__main__':
 rows=[]
 for model in('effective','mass_matrix'):
  for sg in(-1,1):
   for b in('zero','max','hold','sweep'):
    r,tr=run(model,sg,b);rows.append(r);print(json.dumps(r),flush=True);np.savetxt(HERE/f'{model}_{sg:+d}_{b}.csv',tr,delimiter=',',header='time_s,truth,theta,omega,phi,velocity,duty,gap_ms,adc',comments='')
 (HERE/'nominal_results.json').write_text(json.dumps(rows,indent=2)+'\n')
