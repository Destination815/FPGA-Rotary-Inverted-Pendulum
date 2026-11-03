"""Measured-coordinate,5ms local prediction; input perturbations are hypotheses.

theta and physical phi in radians; PWM is logical duty fraction. This model
does not consume arm legacy coordinates and does not establish motor torque.
"""
from pathlib import Path
import json
import numpy as np

def features(y,u):
    th=y[-1,0];s=np.sin(th);c=np.cos(th)
    d=np.array([(y[-1]-y[-1-k])/(k*.005)for k in [1,2,4,8,12]])
    w,v=d[1];z=d.ravel()
    f=[1,s,c,s*c,v*v*s*c,w*w*s,w*v*s]
    f.extend(z);f.extend(c*z);f.extend(s*z)
    f.extend(np.tanh(d[:,0]/.3));f.extend(np.tanh(d[:,1]/.3))
    us=np.array([np.mean(u[-k:])for k in [1,2,4,8,12]])
    f.extend(us);f.extend(c*us);f.extend(s*us)
    return np.asarray(f)

class LocalPlant:
    def __init__(self,angles_history,duty_history,variant='alpha0.01',pressure=False):
        y=np.asarray(angles_history,dtype=float);u=np.asarray(duty_history,dtype=float)
        if y.shape!=(13,2)or u.shape!=(12,)or not np.isfinite(y).all()or not np.isfinite(u).all():
            raise ValueError('Need13 exact5ms angle pairs and12 preceding5ms command means')
        if np.max(abs(u))>.35+1e-9:raise ValueError('Duty fraction outside recorded350permille bound')
        self.y=list(y.copy());self.u=list(u.copy());self.steps=0;self.maximum=40 if pressure else 20
        p=json.loads(Path(__file__).with_name('parameters.json').read_text())
        self.coef=np.asarray(p['variants'][variant]['coef'],dtype=float)
    def observe(self):return np.asarray(self.y[-1]).copy()
    def step5(self,logical_duty_fraction):
        if self.steps>=self.maximum:raise ValueError('Local horizon exhausted; new measured-history seed required')
        u=float(logical_duty_fraction)
        if not np.isfinite(u)or abs(u)>.35+1e-9:raise ValueError('Duty fraction outside350permille')
        self.u.append(u)
        delta=features(np.asarray(self.y[-13:]),np.asarray(self.u[-13:]))@self.coef
        if not np.isfinite(delta).all()or max(abs(delta))>.6:raise ValueError('Model left finite local domain')
        self.y.append(self.y[-1]+delta);self.steps+=1
        return self.observe()

def forecast(angles_history,duty_history,future_known_duty,variant='alpha0.01',pressure=False):
    p=LocalPlant(angles_history,duty_history,variant,pressure)
    return np.asarray([p.observe()]+[p.step5(u)for u in future_known_duty])
