#!/usr/bin/env python3
"""Actual nonlinear plant integration, integer tracking only; float swing/reference retained.
Uses the exact existing scenarios and independent plant evolution, not CSV replay.
"""
from pathlib import Path
import importlib.util,json,hashlib,argparse
import numpy as np
from fixed_tracking import FixedController
HERE=Path(__file__).resolve().parent;EXP=HERE.parents[1] if (HERE.parents[1]/'robust_tracking_20261002').exists() else HERE.parent/'models'
p=EXP/'robust_validation_20261002/run_integrated.py'
s=importlib.util.spec_from_file_location('tracking_fixed_closed_loop',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
parser=argparse.ArgumentParser();parser.add_argument('--rtl',action='store_true');args=parser.parse_args()
if args.rtl:
 from rtl_tracking import RtlController
 m.track.Controller=RtlController
else:m.track.Controller=FixedController
out=HERE/('rtl_closed_loop_results' if args.rtl else 'closed_loop_results');out.mkdir(exist_ok=True)
results=[]
for model in ('effective','mass_matrix'):
 for sign in (-1,1):
  r,a=m.run(sign,model,duration=60.,substeps=4,swing_path=EXP/'robust_swingup_20261002/selected_candidate_v2.json')
  if args.rtl:
   baseline=np.genfromtxt(HERE/'closed_loop_results'/f'{model}_{sign:+d}.csv',delimiter=',',skip_header=1)
   r['integer_reference_equal']=bool(np.array_equal(a,baseline))
   assert r['integer_reference_equal']
  results.append(r)
  np.savetxt(out/f'{model}_{sign:+d}.csv',a,delimiter=',',comments='',header='time_s,theta_rad,omega_rad_s,phi_rad,velocity_rad_s,duty,reference_rad,ref_velocity_rad_s,ref_acceleration,ref_jerk,theta_measured,omega_estimated,phi_measured,velocity_estimated,mode')
  print(json.dumps(r),flush=True)
report=dict(status='PASS' if all(x['passed'] for x in results) else 'FAIL',results=results,
 source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'fixed_tracking.py',HERE/'closed_loop.py',EXP/'robust_validation_20261002/run_integrated.py']},
 engine='Verilator actual RTL' if args.rtl else 'Python integer',
 scope='Four nominal 60-second nonlinear full flows. Integer tracking; swing, reference, and observers still floating. This is a component substitution check; full RTL closed loop is separate.')
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
