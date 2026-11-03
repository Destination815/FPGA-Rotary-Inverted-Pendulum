# R31C1: private legacy-coordinate backend. Physical callers use coordinate_adapters.py.
"""Frozen S16 v7 effective grey model; fitted history, failed R09_2 extrapolation.
Used for paired engineering checks, not a validated ranking of hardware gains.
"""
import numpy as np
from scipy.linalg import expm
from scipy.signal import savgol_filter
DT=.005;HIST=30
def matrices(q):
 gain,bm,bp,kappa,tau,lag,mu,g,ratio,dead=q[:10];A=np.zeros((7,7));B=np.zeros((7,2));C=np.zeros((2,7));C[0,0]=1;C[1,2]=1
 A[0,1]=1;A[2,3]=1;A[1,0]=g;A[1,1]=-bp;A[1,6]=-kappa;A[3,3]=-bm;A[3,4]=1;A[3,0]=-mu*g;A[3,1]=mu*bp;A[4,4]=-1/tau;B[4,0]=gain/tau;A[5,:]=A[3,:]/lag;A[5,5]-=1/lag;A[6,5]=1/lag;A[6,6]=-1/lag
 C[0,:]+=q[10]*A[3,:];B[3,1]=-1;B[5,1]=-1/lag
 M=np.zeros((9,9));M[:7,:7]=A;M[:7,7:]=B;E=expm(M*DT);return E[:7,:7],E[:7,7],C,E[:7,8]
def init_batch(q,y,u):
 A,B,C,F=matrices(q);u=np.sign(u)*np.maximum(abs(u)-q[9],0)*np.where(u>=0,1,q[8]);force=np.zeros((len(y),7));O=[];res=[];power=np.eye(7)
 # Derivative uses only the supplied past window, never future validation samples.
 vel=savgol_filter(y[:,:,1],9,2,deriv=1,delta=.005,axis=1)
 for i in range(HIST):
  O.append(C@power);res.append(y[:,i]-force@C.T);force=force@A.T+u[:,i,None]*B+q[11]*np.tanh(vel[:,i,None]/.1)*F;power=A@power
 x=force+np.stack(res,axis=1).reshape(len(y),-1)@np.linalg.pinv(np.vstack(O),rcond=1e-6).T@power.T
 return x

