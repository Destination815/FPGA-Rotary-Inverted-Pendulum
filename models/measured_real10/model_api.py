"""Causal nonlinear measured-input model. Public theta/phi rad, PWM logical fraction.
Uses frozen M05 equations; new effective parameters supplied explicitly.
No serial, controller edits, or future measured-state correction.
"""
from pathlib import Path
import ctypes as C,json,hashlib,subprocess
import numpy as np
P=Path(__file__).resolve().parent
W=P.parents[2]
K=101376/97763
PTR=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def native():
 src=P/'fullplant.cpp';lib=P/('fullplant_'+digest(src)[:12]+'.so')
 if not lib.exists():subprocess.run(['c++','-O3','-std=c++17','-shared','-fPIC',str(src),'-o',str(lib)],check=True)
 l=C.CDLL(str(lib));l.rollout.argtypes=[PTR,PTR,PTR,C.c_int,C.c_int,PTR];l.initialize.argtypes=[PTR,PTR,PTR,PTR,C.c_int,PTR];l.batch_replay.argtypes=[PTR,PTR,PTR,C.c_int,C.c_int,C.c_int,PTR];return l
LIB=None
def getlib():
 global LIB
 if LIB is None:LIB=native()
 return LIB
def baseline(name='swing'):
 if (P/'baseline_parameters.json').is_file():return np.array(json.loads((P/'baseline_parameters.json').read_text())[name])
 j=json.loads((W/'experiments/r33_revision1_20261024/evidence/r33_shared_plan/cases.json').read_text())
 return np.array(next(c['plant'] for c in j['scopes']['28'] if c['case']==f'M05_{name}_domain_constrained_-1'))
def parameters(base,log_scales):
 p=np.array(base,copy=True);z=np.exp(log_scales);p[6]*=z[0];p[9]*=z[1];p[8]*=z[2];p[10]*=z[3];return p
def measured_rod_angle(true_angle,scale):
 true_angle=np.asarray(true_angle);a=(true_angle+np.pi)%(2*np.pi)-np.pi;a=np.where(a>1.7/scale,a-2*np.pi,a);m=a*scale
 return m+2*np.pi*np.round((true_angle-m)/(2*np.pi))
def rollout(state,logical_pwm,p,substeps=2):
 x=np.ascontiguousarray(state,dtype=float);u=np.ascontiguousarray(logical_pwm,dtype=float);p=np.ascontiguousarray(p,dtype=float)
 if x.shape!=(7,)or p.shape!=(18,)or u.ndim!=1 or not all(np.isfinite(v).all()for v in[x,u,p])or np.any(abs(u)>1):raise ValueError('7 finite latent states,18 parameters and bounded logical PWM fractions required')
 out=np.empty((len(u)+1,7));failure=getlib().rollout(x,u,p,len(u),substeps,out)
 if failure>=0:out[failure:]=np.nan
 return out
def initialize(history,past_pwm,p):
 h=np.array(history,dtype=float,copy=True)
 if h.ndim!=3 or h.shape[1:]!=(31,2)or np.asarray(past_pwm).shape!=(len(h),150)or np.asarray(p).shape!=(18,):raise ValueError('batchx31x2 physical angles, batchx150 priorPWM,18 parameters required')
 if not all(np.isfinite(v).all()for v in[h,past_pwm,p])or np.any(abs(np.asarray(past_pwm))>1):raise ValueError('finite histories/parameters and bounded PWM fraction required')
 h[:,:,1]*=K
 # Exact frozen M05 chart handling. Whole measured turns must not be divided by sensor scale.
 h[:,:,0]=np.unwrap(h[:,:,0],axis=1);wrapped=np.arctan2(np.sin(h[:,0,0]),np.cos(h[:,0,0]));wrapped=np.where(wrapped>1.7,wrapped-2*np.pi,wrapped);shift=h[:,0,0]-wrapped;h[:,:,0]-=shift[:,None]
 h=np.ascontiguousarray(h);u=np.ascontiguousarray(past_pwm,dtype=float);p=np.ascontiguousarray(p,dtype=float)
 v=np.ascontiguousarray([np.polynomial.polynomial.polyfit(np.arange(9)*.005,y[:9],2)[1]for y in h]);v=np.clip(v,[-35,-20],[35,20]);out=np.empty((len(h),7))
 fail=getlib().initialize(h,u,v,p,len(h),out)
 if fail:raise FloatingPointError(f'{fail} past-only initializations diverged')
 # True-state revolutions restore physics; observe() applies the periodic sensor chart.
 out[:,0]+=shift
 return out
def forecast(history,past_pwm,future_pwm,p):
 x=initialize(history,past_pwm,p);u=np.ascontiguousarray(future_pwm,dtype=float)
 if u.ndim!=2 or u.shape[0]!=len(x)or u.shape[1]not in(20,50,100,200)or not np.isfinite(u).all()or np.any(abs(u)>1):raise ValueError('20/50/100/200ms bounded future logical PWM fractions required; no global3s accuracy claim')
 out=np.empty((len(x),u.shape[1]//5+1,2));getlib().batch_replay(x,u,np.ascontiguousarray(p),len(x),u.shape[1],5,out);out[:,:,0]=measured_rod_angle(out[:,:,0]/p[16],p[16]);out[:,:,1]/=K;return out
class Plant:
 """Physical-view stepper for batched control screening. State is explicitly legacy latent.
 Public observe gives measured theta and physical encoder phi; true load angle is hidden.
 """
 def __init__(self,p,initial,*,state_coordinates):
  if state_coordinates!='legacy_latent':raise ValueError('Use initialize() or explicit legacy_latent states')
  self.p=np.asarray(p,dtype=float);self.x=np.asarray(initial,dtype=float).copy()
 def observe(self):return np.array([float(measured_rod_angle(self.x[0],self.p[16])),self.x[2]/K])
 def step(self,pwm_fraction):
  self.x=rollout(self.x,np.array([pwm_fraction]),self.p)[-1];return self.observe()
