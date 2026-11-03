"""Updated and historical M05 causal local plant, physical public coordinates.
Valid for measured-history seeded <=200ms prediction; autonomous3s not validated.
"""
from pathlib import Path
import sys,json
import numpy as np
from scipy.signal import savgol_filter
P=Path(__file__).resolve().parent
from local_backend.model import matrices,friction
from model_api import K
OLD=json.loads((P/'local_backend/frozen.json').read_text())['selected']['q'];OLDP=json.loads((P/'local_backend/plateau_frozen.json').read_text())['q'];NEW=json.loads((P/'local_frozen.json').read_text())
def config(history,variant):
 h=np.array(history,dtype=float,copy=True);h[:,1]*=K
 if variant not in ('baseline','updated'):raise ValueError('variant baseline/updated required')
 flat=np.ptp(h[-9:,1])<np.pi/1040
 return np.array((NEW['qp']if flat else NEW['q'])if variant=='updated'else(OLDP if flat else OLD))
def seed_from_history(history,past_pwm,*,variant):
 """Exactly31 physical[theta,phi]samples/5ms,150 priorlogicalPWM fractions.
 No future observations, no estimated-zero placeholders for absent C831 fields.
 """
 h=np.array(history,dtype=float,copy=True);u=np.asarray(past_pwm,dtype=float)
 if h.shape!=(31,2)or u.shape!=(150,)or not np.isfinite(h).all()or not np.isfinite(u).all():raise ValueError('31x2 past angles and150 priorPWM required')
 q=config(h,variant);h[:,1]*=K;a,b,c,f=matrices(q,.001);force=np.zeros(7);power=np.eye(7);obs=[];res=[];vel=savgol_filter(h[:,1],9,2,deriv=1,delta=.005);drive=np.sign(u)*np.maximum(abs(u)-q[9],0)*np.where(u>=0,1,q[8])
 for t in range(151):
  if t%5==0:obs.append(c@power);res.append(h[t//5]-c@force)
  if t<150:
   j=t//5;frac=t%5/5;v=(1-frac)*vel[j]+frac*vel[j+1];force=a@force+drive[t]*b+friction(q,v)*f;power=a@power
 O=np.vstack(obs);target=np.concatenate(res)
 if len(q)>14 and q[14]>0:
  past_v=savgol_filter(h[-7:],7,2,deriv=1,delta=.005,axis=0)[-1];dc=(c@a-c)/.001;du=c@b/.001;df=c@f/.001;rhs=past_v-dc@force-drive[149]*du-friction(q,vel[-1])*df
  O=np.vstack([O,q[14]*dc@power]);target=np.r_[target,q[14]*rhs]
 x=force+power@np.linalg.pinv(O,rcond=1e-6)@target;x+=np.linalg.pinv(c)@(h[-1]-c@x)
 if len(q)>15 and q[15]>0 and np.ptp(h[-9:,1])<np.pi/1040:x[3]=0
 return {'q':q.tolist(),'state':x.tolist(),'variant':variant,'state_coordinates':'effective latent legacy','public_coordinates':'theta,physical_phi radians','horizon_limit_ms':200}
class Plant:
 def __init__(self,seed):
  self.q=np.array(seed['q']);self.x=np.array(seed['state']);self.a,self.b,self.c,self.f=matrices(self.q,.001)
 def observe(self):
  y=self.c@self.x;y[1]/=K;return y
 def step(self,logical_pwm_fraction):
  u=float(logical_pwm_fraction)
  if not np.isfinite(u)or abs(u)>1:raise ValueError('finite PWM fraction [-1,1] required')
  q=self.q;drive=np.sign(u)*max(abs(u)-q[9],0)*(q[8]if u<0 else 1);fric=friction(q,self.x[3])
  if len(q)>15 and q[15]>0 and abs(self.x[3])<.08:
   r=-q[1]*self.x[3]+self.x[4]-q[6]*q[7]*self.x[0]+q[6]*q[2]*self.x[1];fric=np.clip(self.x[3]/.001+r,-q[11]*(1-q[13]),q[11]*(1+q[13]))
  self.x=self.a@self.x+self.b*drive+self.f*fric;return self.observe()
