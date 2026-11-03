#!/usr/bin/env python3
"""Build actual R01 RTL-in-loop library and run reproducible local verification.
Never opens a serial port or programs hardware. Python3+NumPy+Verilator+C++ needed.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
EX=ROOT/'experiments/r01_full_control_20261002'
if not EX.exists():EX=ROOT
BUILD=ROOT/'build/r01'
def run(args,name):
 with (BUILD/name).open('w') as f:
  p=subprocess.run([str(x) for x in args],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:raise RuntimeError(f'{name}:\n'+(BUILD/name).read_text()[-5000:])
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--build-only',action='store_true');a=ap.parse_args();BUILD.mkdir(exist_ok=True,parents=True)
 inc='-I'+str(EX/'fpga/src');src=list((EX/'fpga/src').glob('*.v'))+[EX/'sim/dpb_sim_model.v']
 run(['verilator','--lint-only','--language','1800-2012','-Wno-fatal','-Wall','-Wno-DECLFILENAME','-Wno-UNUSEDSIGNAL','-Wno-UNUSEDPARAM','-Wno-SYNCASYNCNET','--top-module','j280_top',inc,*src],'strict_lint.log')
 vr=subprocess.check_output(['verilator','--getenv','VERILATOR_ROOT'],text=True).strip()
 obj=BUILD/'control_obj';sources=[EX/'sim/r01_control_harness.v',EX/'fpga/src/controller.v',EX/'fpga/src/state_estimator.v',*sorted((EX/'fpga/src').glob('r01*.v')),*sorted((EX/'fpga/src').glob('r10*.v')),*sorted((EX/'fpga/src').glob('r29*.v')),*sorted((EX/'fpga/src').glob('s23*.v')),EX/'sim/dpb_sim_model.v']
 run(['verilator','--cc','--build','-j','3','--Mdir',obj,'--top-module','r01_control_harness','-Wno-fatal','-CFLAGS','-O2 -fPIC',inc,*sources],'control_build.log')
 # Linux and macOS desktop development. Windows C uses pre-generated Verilog/GUI.
 library=BUILD/('libr01_control.dylib' if sys.platform=='darwin' else 'libr01_control.so')
 run([os.environ.get('CXX','c++'),'-shared','-O2','-fPIC','-std=c++17','-I'+str(obj),'-I'+vr+'/include','-I'+vr+'/include/vltstd',EX/'sim/control_api.cpp',obj/'Vr01_control_harness__ALL.a',vr+'/include/verilated.cpp',vr+'/include/verilated_threads.cpp','-o',library],'ffi_build.log')
 hashes={str(p.relative_to(EX)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [*src,*sorted((EX/'fpga/src').glob('*.vh')),EX/'sim/control_api.cpp',EX/'sim/r01_control_harness.v']}
 (BUILD/'rtl_library_provenance.json').write_text(json.dumps(dict(source_sha256=hashes,library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),lint='PASS; warnings retained',hardware_STA='PENDING'),indent=2)+'\n')
 print('PASS integrated RTL lint (warnings logged); actual RTL library built',flush=True)
 if not a.build_only:
  run([sys.executable,EX/'tools/check_manual_model.py','--out',EX/'evidence/manual_model_local'],'manual_model.log')
  print('Completed four legacy-model manual-hold scenarios. Read their summary; no physical performance claim.')
if __name__=='__main__':main()
