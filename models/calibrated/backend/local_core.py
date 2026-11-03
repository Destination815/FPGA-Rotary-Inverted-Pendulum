from pathlib import Path
import sys,numpy as np
from scipy.signal import savgol_filter
sys.path.insert(0,str(Path(__file__).resolve().parent/"src"))
import grey_original as G
G.DT=.005;G.HIST=30
def initialize(q,y,u,tau):
 a,b,c,f=G.matrices(q);power=np.eye(7);force=np.zeros((len(y),7));O=[];res=[]
 vel=savgol_filter(y[:,:,1],9,2,deriv=1,delta=.005,axis=1);drive=np.sign(u)*np.maximum(abs(u)-q[9],0)*np.where(u>=0,1,q[8])
 for j in range(31):
  weight=np.exp((j-30)*.005/tau)if tau else 1.;O.append(weight*c@power);res.append(weight*(y[:,j]-force@c.T))
  if j<30:force=force@a.T+drive[:,j,None]*b+q[11]*np.tanh(vel[:,j,None]/.1)*f;power=a@power
 x=force+np.stack(res,axis=1).reshape(len(y),-1)@np.linalg.pinv(np.vstack(O),rcond=1e-6).T@power.T
 # Exact present observation projection; derivative/hidden states still estimated from past only.
 return x+(y[:,-1]-x@c.T)@np.linalg.pinv(c).T

def forecast(q,Y,U,tau):
 a,b,c,f=G.matrices(q);x=initialize(q,Y[:,:31],U[:,:30],tau);out=[];drive=np.sign(U)*np.maximum(abs(U)-q[9],0)*np.where(U>=0,1,q[8])
 for j in range(30,71):
  out.append(x@c.T)
  if j<70:x=x@a.T+drive[:,j,None]*b+q[11]*np.tanh(x[:,3,None]/.1)*f
 return np.array(out).transpose(1,0,2)
