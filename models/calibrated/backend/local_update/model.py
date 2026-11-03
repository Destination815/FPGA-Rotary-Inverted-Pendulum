"""Measured-coordinate grey model with per-ms PWM nonlinearity, causal150ms initialization."""
from pathlib import Path
import sys,numpy as np
from scipy.signal import savgol_filter
try:from . import grey_base as G
except ImportError:import grey_base as G

def matrices(q,dt=.001):
 old=G.DT;G.DT=dt
 try:return G.matrices(q)
 finally:G.DT=old

def friction(q,v):
 width=q[12] if len(q)>12 else .1
 asym=q[13] if len(q)>13 else 0.
 z=np.tanh(v/width)
 return q[11]*(z+asym*z*z)

def forecast(q,H,U,substeps=1):
 assert substeps in (1,2,4)
 dt=.001/substeps
 n=len(H);a,b,c,f=matrices(q,dt);drive=np.sign(U)*np.maximum(abs(U)-q[9],0)*np.where(U>=0,1,q[8]);force=np.zeros((n,7));power=np.eye(7);O=[];res=[]
 vel=savgol_filter(H[:,:,1],9,2,deriv=1,delta=.005,axis=1)
 for t in range(150*substeps+1):
  if t%(5*substeps)==0:O.append(c@power);res.append(H[:,t//(5*substeps)]-force@c.T)
  if t<150*substeps:
   j=t//(5*substeps);frac=t%(5*substeps)/(5*substeps);v=(1-frac)*vel[:,j]+frac*vel[:,j+1];force=force@a.T+drive[:,t//substeps,None]*b+friction(q,v[:,None])*f;power=a@power
 obs=np.vstack(O);target=np.stack(res,axis=1).reshape(n,-1)
 if len(q)>14 and q[14]>0:
  # Only past endpoint velocity from a quadratic fit over the last 35ms.
  past_v=savgol_filter(H[:,-7:],7,2,deriv=1,delta=.005,axis=1)[:,-1]
  dc=(c@a-c)/dt;du=c@b/dt;df=c@f/dt
  rhs=past_v-force@dc.T-drive[:,149,None]*du-friction(q,vel[:,-1,None])*df
  obs=np.vstack([obs,q[14]*dc@power]);target=np.c_[target,q[14]*rhs]
 x=force+target@np.linalg.pinv(obs,rcond=1e-6).T@power.T;x+=(H[:,-1]-x@c.T)@np.linalg.pinv(c).T
 if len(q)>15 and q[15]>0:
  # Encoder plateaus: one count = 2*pi/1040rad. A past 40ms plateau bounds
  # average velocity; initialize zero only when no complete count occurred.
  flat=np.ptp(H[:,-9:,1],axis=1)<2*np.pi/1040*.5
  x[flat,3]=0.
 out=[]
 for t in range(150*substeps,U.shape[1]*substeps+1):
  if (t-150*substeps)%(5*substeps)==0:out.append(x@c.T)
  if t<U.shape[1]*substeps:
   force_value=friction(q,x[:,3,None])
   if len(q)>15 and q[15]>0:
    # Set-valued dry friction near rest prevents low motor force creating
    # fictitious creep. Direction bounds inherited from moving friction.
    r=-q[1]*x[:,3]+x[:,4]-q[6]*q[7]*x[:,0]+q[6]*q[2]*x[:,1]
    dry=np.clip(x[:,3]/dt+r,-q[11]*(1-q[13]),q[11]*(1+q[13]))
    force_value=np.where((abs(x[:,3])<.08)[:,None],dry[:,None],force_value)
   x=x@a.T+drive[:,t//substeps,None]*b+force_value*f
 return np.array(out).transpose(1,0,2)
