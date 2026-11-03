from pathlib import Path
import argparse,json,importlib.util,math,hashlib
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();P=args.project.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
s=importlib.util.spec_from_file_location('ordinary43',P/'observer/fixed_model.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);rows=[];K=97763/101376
for physical_error in [-8,-5.5,-5.01,-4.99,-3,-1,-.5,-.25,.25,.5,1,3,4.99,5.01,5.5,8]:
 p=round(math.radians(physical_error)/K*65536);a=m.Observer();a.ready=1;a.age=1000;a.samples=200;a.blend=2**24;a.motion_latched=True;a.motion_phase=3;a.drift_latched=True;a.z[2]=p<<8;a.lp=p<<8;u=old=0;first=None
 for j in range(2000):
  a.mature_static_hold=True;before=a.integral;d,_,fault=a.tick(0,p,u,350,0,0,True,True,0,False,old,True,0,True);old,u=u,d
  if first is None and a.integral!=before:first=j
  assert not fault
 rows.append(dict(physical_error_deg=physical_error,p_q16=p,first_integral_update_ms=first,integral_at_2000ms=a.integral/2**24,request_pwm=u))
assert all(r['first_integral_update_ms']is not None and r['integral_at_2000ms']*r['physical_error_deg']>0 for r in rows)
(out/'NO_POSITION_DEADBAND.json').write_text(json.dumps(dict(status='PASS',transactions=32000,source_sha256=hashlib.sha256((P/'observer/fixed_model.py').read_bytes()).hexdigest(),scope='Synthetic fixed encoder test establishes continuing correction below5deg, not physical control efficacy. Explicit endpoint is mature/motion_latched/phase3; positionP and gatedI remain active.',cases=rows),indent=2)+'\n');print('PASS32,000 synthetic no-deadband transactions')
