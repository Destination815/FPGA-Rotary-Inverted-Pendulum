"""Bounded R31C1 verification. No serial, programming, refit, or historical search.
Default: host/data/model checks and fresh top/controller build. --rtl adds actual
UART/FPGA tests. --gui requires GUI dependencies. --baseline compares existing
R31 RTL; --resources is a local estimate, never real Gowin STA.
"""
from pathlib import Path
import subprocess,sys,os,json,time,hashlib,argparse
ROOT=Path(__file__).resolve().parents[1]
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--rtl',action='store_true');ap.add_argument('--gui',action='store_true');ap.add_argument('--baseline',type=Path);ap.add_argument('--resources',action='store_true');ap.add_argument('--manifest',action='store_true');a=ap.parse_args()
 out=ROOT/'build/calibration_verification';out.mkdir(parents=True,exist_ok=True);checks=[];started=time.monotonic()
 def run(name,script,*extra):
  cmd=[sys.executable,str(ROOT/script),*map(str,extra)]
  with (out/(name+'.log')).open('w') as log:r=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
  checks.append(dict(name=name,status='PASS' if r.returncode==0 else 'FAIL',command=cmd,script_sha256=hashlib.sha256((ROOT/script).read_bytes()).hexdigest()))
  (out/'receipt.json').write_text(json.dumps(dict(status='RUNNING' if not r.returncode else 'FAIL',checks=checks,seconds=time.monotonic()-started),indent=2)+'\n')
  if r.returncode:raise RuntimeError(name+': '+(out/(name+'.log')).read_text()[-5000:])
  print('PASS',name,flush=True)
 run('coordinate_constants','tools/generate_coordinates.py','--check')
 run('host_contract','tools/test_calibrated_host.py')
 run('passive_and_scoring','tools/test_calibrated_tools.py')
 run('physical_models','tools/test_calibrated_models.py')
 run('build','tools/verify_r01.py','--build-only')
 if a.rtl:run('actual_uart_and_arithmetic','tools/test_calibrated_rtl.py')
 if a.gui:run('actual_default_gui','tools/test_calibrated_gui.py')
 if a.baseline:run('old_new_rtl_equivalence','tools/test_calibrated_equivalence.py','--baseline',a.baseline.resolve())
 if a.resources:run('local_resource_estimate','tools/synthesize_resources.py')
 if a.manifest:run('source_manifest','tools/check_project.py')
 result=dict(status='PASS',checks=checks,seconds=time.monotonic()-started,hardware_STA='PENDING',scope='Specified calibration integration checks only; unchanged R31 validation reused via bound sources. This tool never authorizes motion.')
 (out/'receipt.json').write_text(json.dumps(result,indent=2)+'\n');print('PASS R31C1 specified checks; real Gowin STA still required')
if __name__=='__main__':main()
