from pathlib import Path
import ctypes as C,json,numpy as np,sys
P=Path(__file__).resolve().parent.parent;D=P/'build/reproduce/hybrid';bankfile=json.loads((P/'hybrid/model_bank.json').read_text());models=bankfile['models'];KEYS=bankfile['keys'];bank=np.array([[m['p'][k]for k in KEYS]for m in models]);ptr=C.POINTER(C.c_double);lib=C.CDLL(str(D/('hybrid_fixed.dylib'if sys.platform=='darwin'else'hybrid_fixed.so')));lib.simulate.argtypes=[ptr,ptr,C.c_int,ptr,C.c_int,C.c_int,C.c_int,ptr,ptr];unused=np.zeros(16)
for name,a,b in [('stress',.05,.1),('holdout_recheck',.02,.03)]:
 rows=[]
 for mi in range(4):
  for sign in [-1,1]:
   for variation in ([-1,1]if name=='stress'else[sign]):
    p=bank[mi].copy();p[0]*=1+variation*a;p[1]*=1-variation*a;p[4:6]*=1+variation*b;p[6:8]*=1-variation*b;o=np.zeros(11)
    lib.simulate(bank.ctypes.data_as(ptr),p.ctypes.data_as(ptr),int(mi>=2),unused.ctypes.data_as(ptr),sign,36000,variation*2,o.ctypes.data_as(ptr),None)
    rows.append(dict(model=mi,sign=sign,variation=variation,fault=int(o[0]),mode=int(o[1]),capture=o[2],duration=o[3],theta_rms=o[4],phi_rms=o[5],passed=bool(o[0]==0 and o[1]==2 and o[4]<3 and o[5]<5)))
 (P/'build/reproduce'/(name+'.json')).write_text(json.dumps(dict(cases=rows),indent=2));print(name,sum(r['passed']for r in rows),len(rows),flush=True)
