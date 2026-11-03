# R31C1: private legacy-coordinate backend. Physical callers use coordinate_adapters.py.
from pathlib import Path
import ctypes as ct
import sys,subprocess
import numpy as np
P=Path(__file__).resolve().parents[1];binary=P/'build'/('nonlinear.dylib' if sys.platform=='darwin' else 'nonlinear.so')
if not binary.exists():
 binary.parent.mkdir(exist_ok=True,parents=True)
 subprocess.run(['c++','-shared','-O2','-fPIC',str(P/'models/nonlinear_native.cpp'),'-o',str(binary)],check=True)
lib=ct.CDLL(str(binary));ptr=ct.POINTER(ct.c_double)
for n in ['effective_step','mass_step']:getattr(lib,n).argtypes=[ptr,ct.c_double,ptr,ct.c_double,ct.c_int]
keys=['alpha','kappa','damping','coulomb','gain_pos','gain_neg','tau_on','tau_off','deadzone']
def advance(x,u,p,model,dt=.001,substeps=4):
 v=np.ascontiguousarray(x,dtype=float).copy();q=np.array([p[k]for k in keys]);getattr(lib,'effective_step'if model=='effective'else 'mass_step')(v.ctypes.data_as(ptr),u,q.ctypes.data_as(ptr),dt,substeps);return v
