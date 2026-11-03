"""R10 bit-exact integer direct feedback, Q18 inputs. No controller integrator."""
from pathlib import Path
import json, math
Q=1<<18;CQ=1<<30
p=json.loads(Path(__file__).with_name('direct_parameters.json').read_text())['k']
K=dict(kp=round(p[0]*Q),kd=round(p[1]*Q),op=round(p[2]*Q),od=round(p[3]*Q),comp=round(p[4]*Q),band=round(p[6]*Q),feed=round(p[7]*Q),ff=round((1.27/94.5076)*Q))
TANH=tuple(round(math.tanh((i/256)/p[5])*Q) for i in range(129))
LIM_IN=(4*Q,32*Q,8*Q,32*Q,4*Q,8*Q,8*Q,128*Q,1024*Q)
def clip(x,lo,hi):return min(hi,max(lo,x))
def sat(x):return clip(x,-(1<<31),(1<<31)-1)
def mul(a,b,shift=18):return sat((int(a)*int(b))>>shift)
def tanh_q(x):
 z=abs(x)
 if z>=Q//2:return Q if x>=0 else -Q
 idx=z>>10;part=z&1023;y=TANH[idx]+((TANH[idx+1]-TANH[idx])*part>>10)
 return -y if x<0 else y
def permille(x):
 v=abs(x)*1000;n=v>>18;r=v&(Q-1)
 if r>Q//2 or (r==Q//2 and n&1):n+=1
 return -n if x<0 else n
class FixedController:
 def __init__(self,parameters=None):self.reset()
 def reset(self,theta=None,vreq=0.,integral=0.,previous_duty=0.):
  self.prevu=clip(round(previous_duty*1000),-150,150);self.pending_init=theta is not None;self.fault=False;self.diagnostics={}
 def clear(self):self.reset()
 def tick_q(self,theta,omega,phi,vel,angle_filtered,ref_pos=0,ref_vel=0,ref_acc=0,ref_jerk=0,*,init_state=False,prior_duty_permille=0,duty_limit=150):
  vals=(theta,omega,phi,vel,angle_filtered,ref_pos,ref_vel,ref_acc,ref_jerk)
  if self.fault or any(abs(int(v))>bound for v,bound in zip(vals,LIM_IN)):
   self.fault=True;self.prevu=0;return 0
  if init_state:self.prevu=clip(prior_duty_permille,-200,200)
  ff=mul(ref_acc,K['ff']);e=sat(angle_filtered-ff);e=max(e-K['band'],0) if e>=0 else min(e+K['band'],0)
  a=sat(mul(e,K['kp'])+mul(omega,K['kd']));b=sat(mul(sat(phi-ref_pos),K['op'])+mul(sat(vel-ref_vel),K['od']))
  linear=sat(sat(a+b)+mul(ref_vel,K['feed']));raw=sat(linear+mul(tanh_q(linear),K['comp']))
  cap=clip(duty_limit,150,200);self.prevu=clip(clip(permille(raw),-cap,cap),self.prevu-20,self.prevu+20);self.diagnostics=dict(lean=ff,acceleration_request=linear,vreq=0,vi=0,raw=raw,angle_filtered=angle_filtered)
  return self.prevu
 def tick(self,measured,reference=(0.,0.,0.,0.),*,angle_filtered=None):
  if angle_filtered is None:raise ValueError('Supply filtered angle')
  vals=[round(float(v)*Q) for v in (*measured,angle_filtered,*reference)];first=self.pending_init;self.pending_init=False
  return self.tick_q(*vals,init_state=first,prior_duty_permille=self.prevu)/1000.
 tick_float=tick
