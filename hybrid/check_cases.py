from pathlib import Path
import sys,json,ctypes as C,time,numpy as np
D=Path(sys.argv[1]);R=D.parents[1];bank_json=json.loads((D/'model_bank.json').read_text());models=bank_json['models'];KEYS=bank_json['keys'];ext='.dylib' if sys.platform=='darwin' else '.so';lib=C.CDLL(str(D/('hybrid_fixed'+ext)));ptr=C.POINTER(C.c_double);lib.simulate.argtypes=[ptr,ptr,C.c_int,ptr,C.c_int,C.c_int,C.c_int,ptr,ptr];bank=np.array([[m['p'][k]for k in KEYS]for m in models]);unused=np.zeros(16);unused[2]=2;rows=[];st=time.monotonic()
for mi in range(4):
 for sign in [-1,1]:
  for event in [-1,0,1]:
   p=bank[mi].copy();o=np.zeros(11);lib.simulate(bank.ctypes.data_as(ptr),p.ctypes.data_as(ptr),int(mi>=2),unused.ctypes.data_as(ptr),sign,22000 if event else 18000,event,o.ctypes.data_as(ptr),None)
   r=dict(model=mi,sign=sign,event=event,fault=int(o[0]),mode=int(o[1]),capture_s=o[2],duration_s=o[3],theta_rms=o[4],phi_rms=o[5],restarts=int(o[7]),pass_case=bool(o[0]==0 and o[1]==2 and o[4]<3 and o[5]<5));rows.append(r)
(R/'common_cases.json').write_text(json.dumps(dict(cases=rows,elapsed_seconds=time.monotonic()-st,criteria='End-of-run 1s theta RMS <3deg, origin error RMS <5deg, captured, no fault. NOT final4 3.5-second event score.'),indent=2))
assert all(x['pass_case']for x in rows),[x for x in rows if not x['pass_case']];print('PASS 24 causal integer / real-estimator cases')
