"""Physical view of the existing reduced stress model, not a new physical fit.
Legacy parameters remain in plant.py. PWM is unchanged; phi/v are converted at
both boundaries. Use calibrated/calibrated_models.py for the M05 model ensemble.
"""
from pathlib import Path
import sys,numpy as np
P=Path(__file__).resolve().parent
sys.path.insert(0,str(P.parent/'host'))
from coordinates import SCALE,RPC_Q24
import plant as legacy
S=np.array([1.,1.,SCALE,SCALE])
def step(state,pwm,parameters,dt=.001,substeps=1):
 return legacy.step(np.asarray(state)/S,pwm,parameters,dt,substeps)*S
def encoder_counts(phi_physical,*,zero=0):
 return np.rint(np.asarray(phi_physical)/(RPC_Q24/2**24)).astype(np.int64)+zero
def nominal():return legacy.nominal()
