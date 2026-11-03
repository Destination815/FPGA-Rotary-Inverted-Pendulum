from pathlib import Path
import argparse,subprocess,ctypes as C
A=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,default=A.parents[2]);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();P=args.project.resolve();D=args.out.resolve();D.mkdir(parents=True,exist_ok=False)
vr=Path(subprocess.check_output(['verilator','--getenv','VERILATOR_ROOT'],text=True).strip());est=D/'estimator'
def library(source,include):
 with(D/'build.log').open('w')as f:
  subprocess.run(['verilator','--cc','--build','-j','2','-Wno-fatal','-CFLAGS','-O3 -fPIC','--Mdir',str(est),'--top-module','state_estimator','-I'+str(P/'fpga/src'),str(P/'fpga/src/state_estimator.v')],stdout=f,stderr=subprocess.STDOUT,check=True)
  subprocess.run(['c++','-O3','-shared','-fPIC','-std=c++17','-I'+str(include),'-I'+str(A/'fixtures'),'-I'+str(est),'-I'+str(vr/'include'),'-I'+str(vr/'include/vltstd'),str(A/'fixtures/estimator_api.cpp'),str(est/'Vstate_estimator__ALL.a'),str(vr/'include/verilated.cpp'),str(vr/'include/verilated_threads.cpp'),str(source),'-o',str(D/'probe.dylib')],stdout=f,stderr=subprocess.STDOUT,check=True)
 return C.CDLL(str(D/'probe.dylib'))
