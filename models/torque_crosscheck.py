# R31C1: private legacy-coordinate backend. Physical callers use coordinate_adapters.py.
"""Alternate coupled mass-matrix plant, not an independently identified plant.
Use measured alpha and STEP-derived candidate r via kappa; illustrative m=.06kg,
l=.075m. Motor damping/stall gain chosen to reproduce effective local speed tau.
Unlike the speed-actuated plant, pendulum reaction torque loads the arm.
"""
import numpy as np
from plant import step as effective_step

def physical(p):
    m=.06;l=.075;gravity=9.81
    J=m*gravity*l/p['alpha'];r=p['kappa']*gravity/p['alpha'];b=m*r*l
    gain=(p['gain_pos']+p['gain_neg'])/2
    torque_gain=.647 # official stall-reference conversion, not dynamic measurement
    damping=torque_gain/gain
    Jr=p['tau_on']*damping-m*r*r+b*b/J
    assert np.all(J>=m*l*l), "negative centre-of-mass inertia"
    assert np.all(Jr>0), "negative arm inertia"
    return m,l,J,r,Jr,torque_gain,damping

def derivative(x,u,p):
    theta,w,phi,v=np.moveaxis(x,-1,0);m,l,J,r,Jr,K,B=physical(p)
    s,c=np.sin(theta),np.cos(theta);a=Jr+m*r*r+J*s*s;b=m*r*l*c
    # Directional motor effectiveness retained as uncertainty, not compensation.
    gain_dir=np.where(u>=0,p['gain_pos'],p['gain_neg'])
    eff=np.sign(u)*np.maximum(np.abs(u)-p['deadzone'],0.)
    B_eff=np.where(np.abs(eff)>1e-12,B,(Jr+m*r*r-b*b/J)/p['tau_off'])
    rhs_arm=K*(gain_dir/((p['gain_pos']+p['gain_neg'])/2))*eff-B_eff*v-2*J*s*c*v*w+m*r*l*s*w*w
    rhs_pend=m*9.81*l*s+J*s*c*v*v-J*p['damping']*w-J*p['coulomb']*np.tanh(w/.05)
    det=a*J-b*b
    return np.stack((w,(a*rhs_pend-b*rhs_arm)/det,v,(J*rhs_arm-b*rhs_pend)/det),axis=-1)

def step(x,u,p,dt=.001,substeps=4):
    y=np.array(x,dtype=float,copy=True);h=dt/substeps
    for _ in range(substeps):
        k1=derivative(y,u,p);k2=derivative(y+h*k1/2,u,p);k3=derivative(y+h*k2/2,u,p);k4=derivative(y+h*k3,u,p)
        y+=h*(k1+2*k2+2*k3+k4)/6
    return y
