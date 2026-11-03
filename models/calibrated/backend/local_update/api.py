"""M05 local measured-coordinate predictor. Offline only; does not access hardware."""
from pathlib import Path
import json
import numpy as np
try:from .model import forecast
except ImportError:from model import forecast
P=Path(__file__).resolve().parent

def predict(history,past_pwm,future_pwm,*,refine=1,variant='m05_domain'):
 """150ms past at 5ms, per-ms logical PWM fractions, predicts 200ms every5ms.
 This is input-conditioned prediction when actual future PWM is replayed; it is
 not an autonomous controller or a globally valid all-angle pendulum model.
 """
 h=np.asarray(history,float);up=np.asarray(past_pwm,float);uf=np.asarray(future_pwm,float);n=len(h)
 if h.shape!=(n,31,2)or up.shape!=(n,150)or uf.shape!=(n,200):raise ValueError('Expected batch×31×2 angles, batch×150 and batch×200 PWM')
 if not all(np.isfinite(a).all()for a in [h,up,uf])or np.max(abs(np.r_[up.ravel(),uf.ravel()]))>1:raise ValueError('Invalid angles or logical PWM fraction')
 if np.max(abs(h[:,:,0]))>=.30:raise ValueError('Local model requires |theta|<0.30rad throughout past history')
 if refine not in[1,2,4]:raise ValueError('refine must be1/2/4')
 bank=json.loads((P/'frozen.json').read_text(encoding="utf-8"));c=[x for x in bank['candidates']if x['name']==('m05_past_velocity' if variant=='m05_domain' else variant)]
 if len(c)!=1:raise ValueError('Unknown local model')
 u=np.c_[up,uf];out=forecast(np.array(c[0]['q']),h,u,substeps=refine)
 if variant=='m05_domain':
  mask=np.ptp(h[:,-9:,1],axis=1)<np.pi/1040
  if mask.any():out[mask]=forecast(np.array(json.loads((P/'plateau_frozen.json').read_text(encoding="utf-8"))['q']),h[mask],u[mask],substeps=refine)
 return out
