"""Measured-coordinate grey model with per-ms PWM nonlinearity, causal150ms initialization."""
from pathlib import Path
import sys,numpy as np
from scipy.signal import savgol_filter
sys.path.insert(0,str(Path(__file__).resolve().parent/'src'));import grey_original as G

def matrices(q):
 old=G.DT;G.DT=.001
 try:return G.matrices(q)
 finally:G.DT=old

def forecast(q,H,U):
 n=len(H);a,b,c,f=matrices(q);drive=np.sign(U)*np.maximum(abs(U)-q[9],0)*np.where(U>=0,1,q[8]);force=np.zeros((n,7));power=np.eye(7);O=[];res=[]
 vel=savgol_filter(H[:,:,1],9,2,deriv=1,delta=.005,axis=1)
 for t in range(151):
  if t%5==0:O.append(c@power);res.append(H[:,t//5]-force@c.T)
  if t<150:
   j=t//5;frac=t%5/5;v=(1-frac)*vel[:,j]+frac*vel[:,j+1];force=force@a.T+drive[:,t,None]*b+q[11]*np.tanh(v[:,None]/.1)*f;power=a@power
 x=force+np.stack(res,axis=1).reshape(n,-1)@np.linalg.pinv(np.vstack(O),rcond=1e-6).T@power.T;x+=(H[:,-1]-x@c.T)@np.linalg.pinv(c).T;out=[]
 for t in range(150,U.shape[1]+1):
  if (t-150)%5==0:out.append(x@c.T)
  if t<U.shape[1]:x=x@a.T+drive[:,t,None]*b+q[11]*np.tanh(x[:,3,None]/.1)*f
 return np.array(out).transpose(1,0,2)
