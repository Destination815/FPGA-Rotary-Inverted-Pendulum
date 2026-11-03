"""Same-condition short sensor stress. No fitting; no hardware access."""
from pathlib import Path
import ctypes as C,json,subprocess,sys,numpy as np,time
P=Path(__file__).resolve().parents[1];B=P/'build/reproduce';D=B/'hybrid';O=P/'build/sensor_pressure';O.mkdir(parents=True,exist_ok=True)
a=__import__('argparse').ArgumentParser();a.add_argument('--baseline',type=Path);a.add_argument('--delays',type=int,nargs='+',default=[8,16]);args=a.parse_args()
ext='.dylib'if sys.platform=='darwin'else'.so'
if args.baseline:
 base_est=args.baseline/'build/reproduce/hybrid'/('estimator'+ext)
else:
 # Build the exact frozen R19R2 estimator included with the package. No cache
 # from another checkout is necessary for an independent comparison.
 S=P/'evidence/r19r2_estimator_golden';E=O/'baseline_estimator';V=Path(subprocess.check_output(['verilator','--getenv','VERILATOR_ROOT'],text=True).strip())
 with (O/'baseline_build.log').open('w')as f:
  subprocess.run(['verilator','--cc','--build','-j','2','-Wno-fatal','-CFLAGS','-O2 -fPIC','--top-module','state_estimator','--Mdir',str(E),'-I'+str(S),str(S/'state_estimator.v'),str(S/'r01_gap_bridge.v')],stdout=f,stderr=subprocess.STDOUT,check=True)
 base_est=O/('estimator'+ext)
 subprocess.run(['c++','-shared','-O3','-fPIC','-std=c++17','-I'+str(E),'-I'+str(V/'include'),'-I'+str(V/'include/vltstd'),str(P/'hybrid/estimator_api.cpp'),str(E/'Vstate_estimator__ALL.a'),str(V/'include/verilated.cpp'),str(V/'include/verilated_threads.cpp'),'-o',str(base_est)],check=True)
base_lib=O/('baseline'+ext)
subprocess.run(['c++','-shared','-O3','-fPIC','-std=c++17',str(D/'hybrid_fixed.cpp'),str(base_est),'-o',str(base_lib)],check=True)
bankdata=json.loads((D/'model_bank.json').read_text());keys=bankdata['keys'];bank=np.array([[m['p'][k]for k in keys]for m in bankdata['models']]);ptr=C.POINTER(C.c_double);results=[];start=time.monotonic()
for name,path in [('R19R2',base_lib),('R20',D/('hybrid_fixed'+ext))]:
 lib=C.CDLL(str(path));lib.simulate.argtypes=[ptr,ptr,C.c_int,ptr,C.c_int,C.c_int,C.c_int,ptr,ptr]
 for delay in args.delays:
  for mi in range(4):
   for sign in [-1,1]:
    cfg=np.zeros(16);cfg[1]=delay;p=bank[mi].copy();o=np.zeros(11);lib.simulate(bank.ctypes.data_as(ptr),p.ctypes.data_as(ptr),int(mi>=2),cfg.ctypes.data_as(ptr),sign,18000,0,o.ctypes.data_as(ptr),None)
    r=dict(version=name,release_delay_ms=delay,model=mi,sign=sign,fault=int(o[0]),mode=int(o[1]),capture_s=o[2],duration_s=o[3],theta_rms=o[4],position_rms=o[5],passed=bool(o[0]==0 and o[1]==2 and o[4]<3 and o[5]<5));results.append(r)
summary={n:dict(passed=sum(r['passed']for r in results if r['version']==n),count=sum(r['version']==n for r in results))for n in ['R19R2','R20']}
import hashlib
hashes={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest()for f in [P/'fpga/src/r01_gap_bridge.v',P/'fpga/src/state_estimator.v',P/'hybrid/hybrid_fixed.cpp',P/'hybrid/model_bank.json']}
(O/'results.json').write_text(json.dumps(dict(scope='Causal integer control + real estimator RTL; delayed rail exit observational stress, not identified sensor physics or full controller RTL. Historical plant parameters unchanged.',cases=results,summary=summary,source_sha256=hashes,elapsed_seconds=time.monotonic()-start),indent=2));print(summary)
