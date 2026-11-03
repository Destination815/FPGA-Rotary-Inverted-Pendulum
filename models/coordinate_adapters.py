"""Physical coordinate APIs for the retained S16 and legacy stress model families.
Backend parameters stay in their original fit units, explicitly called legacy_q /
legacy_parameters. These functions change coordinates, not physics assumptions.
"""
from pathlib import Path
import sys,numpy as np
P=Path(__file__).resolve().parent;sys.path[:0]=[str(P.parent/'host'),str(P)]
from coordinates import SCALE,KAPPA,RPC_Q24
S4=np.array([1.,1.,SCALE,SCALE]);S7=np.array([1.,1.,SCALE,SCALE,SCALE,SCALE,SCALE])
def finite(x,last):
 y=np.asarray(x,dtype=float)
 if y.ndim==0 or y.shape[-1]!=last or not np.isfinite(y).all():raise ValueError('Finite state of length '+str(last)+' required')
 return y

def native_step(state_physical,pwm,legacy_parameters,model,dt=.001,substeps=4):
 if model not in ('effective','mass'):raise ValueError('Choose effective or mass')
 if not np.isfinite(pwm) or abs(pwm)>1 or not 0<dt<=.02 or not 1<=substeps<=100:raise ValueError('PWM/dt/substeps outside domain')
 from nonlinear_native import advance
 x=finite(state_physical,4)
 if x.shape!=(4,):raise ValueError('Native backend requires one four-state vector')
 return advance(x/S4,pwm,legacy_parameters,model,dt,substeps)*S4

def mass_step(state_physical,pwm,legacy_parameters,dt=.001,substeps=4):
 if not np.isfinite(pwm) or abs(pwm)>1 or not 0<dt<=.02 or not isinstance(substeps,int) or not 1<=substeps<=100:raise ValueError('PWM/dt/substeps outside domain')
 from torque_crosscheck import step
 return step(finite(state_physical,4)/S4,pwm,legacy_parameters,dt,substeps)*S4

def local_matrices(legacy_q):
 from measured_grey import matrices
 A,B,C,F=matrices(legacy_q);S=np.diag(S7);Si=np.diag(1/S7);T=np.diag([1.,SCALE])
 return dict(A=S@A@Si,B=S@B,C=T@C@Si,F_legacy_friction_input=S@F,state_units='theta,omega,physical_phi,physical_velocity,physical_acceleration_and_two_filtered_acceleration_states',dt_s=.005)

def local_initialize(legacy_q,history_physical,past_pwm):
 from measured_grey import init_batch,HIST
 h=finite(history_physical,2).copy();u=np.asarray(past_pwm,float)
 if h.ndim!=3 or h.shape[1]<HIST or u.ndim!=2 or u.shape[0]!=len(h) or u.shape[1]<HIST or not np.isfinite(u).all():raise ValueError('Matching past history/PWM batches required')
 h[:,:,1]*=KAPPA
 return init_batch(legacy_q,h,u)*S7
