"""Vectorizable reduced nonlinear Furuta model with effective speed actuator.
Legacy-coordinate model: theta=0 upright; phi/v use historical internal units.
Use physical_plant.py or calibrated/calibrated_models.py for physical angles.
Angles: theta=0 upright, phi=internal arm; positive phi acceleration counteracts positive theta.
This combines a rigid pendulum equation with a locally fitted *effective* actuator;
it is not a fully identified torque/electrical or CAD multibody model.
"""
import json
from pathlib import Path
import numpy as np
GRAVITY=9.81
ADC_RAD=386/65536
LEGACY_INTERNAL_PER_COUNT=396/65536
ENC_RAD=LEGACY_INTERNAL_PER_COUNT  # historical internal-coordinate runner; NOT physical rad/count

def pendulum_accel(theta,w,v,arm_accel,p):
    return (p['alpha']*np.sin(theta)-p['kappa']*np.cos(theta)*arm_accel
            + np.sin(theta)*np.cos(theta)*v*v-p['damping']*w
            -p['coulomb']*np.tanh(w/.05))

def step(x,duty,p,dt=.001,substeps=1):
    """Exact first-order actuator plus midpoint pendulum update. x shape (...,4).
    p may hold scalars or arrays matching x[...,0]. Zero-input tau is independent.
    Returns a new array; no clipping of physical theta, no hidden upright clamp.
    """
    y=np.array(x,dtype=float,copy=True);h=dt/substeps
    eff=np.sign(duty)*np.maximum(np.abs(duty)-p['deadzone'],0.)
    gain=np.where(eff>=0,p['gain_pos'],p['gain_neg'])
    target=gain*eff
    tau=np.where(np.abs(eff)>1e-12,p['tau_on'],p['tau_off'])
    eh=np.exp(-h/tau);em=np.exp(-h/(2*tau))
    for _ in range(substeps):
        t,w,f,v=np.moveaxis(y,-1,0)
        a0=(target-v)/tau;vm=target+(v-target)*em;am=(target-vm)/tau
        wd0=pendulum_accel(t,w,v,a0,p)
        tm=t+.5*h*w;wm=w+.5*h*wd0
        wdmid=pendulum_accel(tm,wm,vm,am,p)
        vn=target+(v-target)*eh
        fn=f+target*h+(v-target)*tau*(1-eh)
        y=np.stack((t+h*wm,w+h*wdmid,fn,vn),axis=-1)
    return y

def nominal(config_path=None):
    """Read the same nominal JSON as Simulink; return a fresh independent dict.
    config_path is for isolated tests/custom copies, never an implicit global edit.
    Campaign scenario ranges are separate: see scenario_ranges.json.
    """
    source=Path(config_path) if config_path is not None else Path(__file__).with_name('model_config.json')
    values=json.loads(source.read_text(encoding='utf-8'))['nominal']
    keys=('alpha','kappa','damping','coulomb','gain_pos','gain_neg','tau_on','tau_off','deadzone')
    out={key:float(values[key]) for key in keys}
    if not all(np.isfinite(v) for v in out.values()):
        raise ValueError('Nominal model parameters must be finite')
    if any(out[k]<=0 for k in ('alpha','kappa','gain_pos','gain_neg','tau_on','tau_off')):
        raise ValueError('Nominal gravity/coupling/actuator gains and time constants must be positive')
    if any(out[k]<0 for k in ('damping','coulomb','deadzone')):
        raise ValueError('Nominal damping, Coulomb friction and dead zone must be nonnegative')
    return out
