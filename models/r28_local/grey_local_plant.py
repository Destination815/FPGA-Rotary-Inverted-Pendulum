"""Unchanged S16/S20 local seven-state plant as optional closed-loop PRESSURE model.
State is x7; only observations [theta,phi] may enter the controller. Internal omega
is not a measured input. Continuous matrices as in frozen measured_grey.py;
exact linear exponential and ZOH nonlinear friction at each actual PWM segment.
No replacement of the nonlinear full-angle swing model is intended.
"""
from pathlib import Path
import json,numpy as np
from scipy.linalg import expm
A=Path(__file__).resolve().parent
class GreyPlant:
 def __init__(self,model='s20_joint',x=None):
  self.q=np.array(json.loads((A/'grey_model_bank.json').read_text())['models'][model]);self.model=model;self.cache={};self.x=np.zeros(7) if x is None else np.array(x,dtype=float)
  q=self.q;gain,bm,bp,kappa,tau,lag,mu,g=q[:8];a=np.zeros((7,7));b=np.zeros((7,2));c=np.zeros((2,7));c[0,0]=1;c[1,2]=1
  a[0,1]=1;a[2,3]=1;a[1,0]=g;a[1,1]=-bp;a[1,6]=-kappa;a[3,3]=-bm;a[3,4]=1;a[3,0]=-mu*g;a[3,1]=mu*bp;a[4,4]=-1/tau;b[4,0]=gain/tau;a[5,:]=a[3,:]/lag;a[5,5]-=1/lag;a[6,5]=1/lag;a[6,6]=-1/lag;c[0,:]+=q[10]*a[3,:];b[3,1]=-1;b[5,1]=-1/lag
  self.M=np.zeros((9,9));self.M[:7,:7]=a;self.M[:7,7:]=b;self.C=c
 def observe(self):return self.C@self.x
 def step(self,u,dt=.001):
  if dt<=0:return
  key=round(float(dt),12)
  if key not in self.cache:
   e=expm(self.M*dt);self.cache[key]=(e[:7,:7],e[:7,7],e[:7,8])
  a,b,f=self.cache[key];q=self.q;drive=np.sign(u)*max(abs(u)-q[9],0)*(1 if u>=0 else q[8]);self.x=a@self.x+b*drive+f*q[11]*np.tanh(self.x[3]/.1)
 def set_observed(self,theta=0,phi=0,omega=0,velocity=0):
  # Defined synthetic equilibrium/perturbation fixture, not measured latent state.
  self.x=np.zeros(7);self.x[1]=omega;self.x[2]=phi;self.x[3]=velocity;self.x[0]=(theta-self.C[0,1:]@self.x[1:])/self.C[0,0]
 def kick(self,omega):self.x[1]+=omega
if __name__=='__main__':
 import importlib.util
 src=A/'historical_measured_grey.py';sp=importlib.util.spec_from_file_location('frozen',src);g=importlib.util.module_from_spec(sp);sp.loader.exec_module(g);g.DT=.001;out=[]
 for name in ['s16_v7','s20_joint']:
  p=GreyPlant(name);p.set_observed(.02,.10,.03,.2);a,b,c,f=g.matrices(p.q);x=p.x.copy();maxerr=0.
  for j in range(4000):
   u=.12*np.sin(j*.013);drive=np.sign(u)*max(abs(u)-p.q[9],0)*(1 if u>=0 else p.q[8]);x=a@x+b*drive+f*p.q[11]*np.tanh(x[3]/.1);p.step(u);maxerr=max(maxerr,float(np.max(abs(x-p.x))));assert np.allclose(c@x,p.observe(),atol=1e-8)
  assert maxerr<1e-9;out.append(dict(model=name,samples=4000,max_state_error=maxerr))
 (A/'grey_plant_equivalence.json').write_text(json.dumps(dict(status='PASS',scope='New wrapper vs frozen model exact discrete 1ms equations; no fitted coefficients changed',cases=out),indent=2));print(out)
