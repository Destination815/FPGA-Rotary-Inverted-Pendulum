from pathlib import Path
import sys,json,math,time,random,subprocess,hashlib
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE.parent),str(HERE.parents[2]/'simulation_workbench_20261002') if (HERE.parents[2]/'simulation_workbench_20261002').exists() else str(HERE.parents[1]/'models')]
from swing_fixed import IntegerSwing,q
from observer import sensor,ENC,wrap
from gap_fixed import FixedObserver
from plant import step,nominal
import torque_crosscheck as tq
FIELDS=('theta','dtheta','phi','dphi','filtered_theta_q18','measurement_ok','gap_active','gap_ms')
BUILD=HERE/'build';EXE=str(BUILD/'Vstate_estimator')
def run(model,sg,behavior,p=None,use_rtl=False):
 p=p or nominal();plant=step if model=='effective' else tq.step;o=FixedObserver();c=IntegerSwing(sg);x=np.array([sg*(math.pi-.02),0.,0.,0.]);cap=None;reason='';last=None;mxerr=0.;maxage=0;trace=[];clocks=0
 rtl=subprocess.Popen([EXE],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) if use_rtl else None
 for j in range(15000):
  raw=sensor(wrap(x[0]),behavior=behavior,last=last);last=raw;enc=round(x[2]/ENC)
  out=o.update(raw,enc,cap is None)
  if rtl:
   rtl.stdin.write(f'U {raw} {enc} {int(cap is None)} 1\n');rtl.stdin.flush();actual=list(map(int,rtl.stdout.readline().split()))
   expected=[int(out[k]) for k in FIELDS]
   if actual[:8]!=expected:raise AssertionError((j,raw,enc,actual,expected))
   out=dict(zip(FIELDS,actual));clocks=max(clocks,actual[-1])
  th=out['theta']/65536;w=out['dtheta']/65536;ph=out['phi']/65536;v=out['dphi']/65536;filtered=out['filtered_theta_q18']/262144
  u=c.update(out['theta']<<2,out['phi']<<2,out['dtheta']<<2,out['dphi']<<2)/1000
  if c.mode==2 and cap is None:cap=j*.001
  if out['gap_active']:mxerr=max(mxerr,abs(wrap(filtered-x[0])))
  maxage=max(maxage,out['gap_ms']);xn=plant(x,u,p,.001,4)
  if j>0 and not out['measurement_ok']:reason='measurement_failed'
  elif abs(xn[2])>math.radians(60):reason='arm60'
  elif cap is not None and abs(wrap(xn[0]))>math.radians(15):reason='fall15'
  elif cap is None and j>=12000:reason='timeout12'
  trace.append([j*.001,wrap(x[0]),th,w,ph,v,u,out['gap_ms'],raw]);x=xn
  if reason or (cap is not None and j*.001-cap>=2):break
 if rtl:rtl.stdin.close();assert rtl.wait()==0
 return dict(model=model,sign=sg,behavior=behavior,rtl=use_rtl,passed=bool(cap is not None and not reason),capture_s=cap,reason=reason or 'completed_2s_hold',max_gap_ms=maxage,max_gap_angle_error_deg=math.degrees(mxerr),clocks=clocks,samples=j+1),trace

def random_equivalence():
 r=random.Random(2026100251);commands=[];expects=[];o=FixedObserver()
 for j in range(10000):
  # free-spin+wrap plusnearblindrail,calibrationabortandfreshnessfault conditions
  angle=math.sin(j*.001)*2.9;adc=sensor(angle,behavior=('zero','max','hold','sweep')[(j//1000)%4],last=500)
  enc=round(math.sin(j*.007)*50);allow=int(j%937<920);ready=int(j%829!=800)
  e=o.update(adc,enc,allow,ready);commands.append(f'U {adc} {enc} {allow} {ready}');expects.append([int(e[k]) for k in FIELDS])
  if j%991==990:
   o.calibrate(encoder=0);commands.append('C 0');expects.append(None)
 proc=subprocess.run([EXE],input='\n'.join(commands)+'\n',text=True,capture_output=True,check=True)
 rows=proc.stdout.splitlines();assert len(rows)==len(expects)
 for j,(a,b) in enumerate(zip(rows,expects)):
  if b is not None and list(map(int,a.split()))[:8]!=b:raise AssertionError((j,commands[j],a,b))
 return dict(samples=10000,mismatches=0)

if __name__=='__main__':
 BUILD.mkdir(exist_ok=True)
 with (HERE/'build.log').open('w') as log:subprocess.run(['verilator','--cc','--exe','--build','-j','4','--language','1364-2005','--top-module','state_estimator','--Mdir',str(BUILD),'-I'+str(HERE.parents[1]/'fpga/src'),str(HERE/'state_estimator_gap.v'),str(HERE/'r01_gap_bridge.v'),str(HERE/'estimator_runner.cpp')],stdout=log,stderr=subprocess.STDOUT,check=True)
 t=time.time();rows=[];random_result=random_equivalence()
 for model in('effective','mass_matrix'):
  for sg in(-1,1):
   for b in('zero','max','hold','sweep'):
    r,tr=run(model,sg,b,use_rtl=True);rows.append(r);print(json.dumps(r),flush=True);np.savetxt(HERE/f'fixed_{model}_{sg:+d}_{b}.csv',tr,delimiter=',',header='time_s,truth,theta,omega,phi,velocity,duty,gap_ms,adc',comments='')
 rng=np.random.default_rng(2026100253);cases=[];uncertain=[]
 for i in range(20):
  p=nominal();p['alpha']*=rng.uniform(.92,1.08);radius=rng.uniform(.115,.145);p['kappa']=p['alpha']*radius/9.81
  p.update(gain_pos=rng.uniform(4.05,6.13),gain_neg=rng.uniform(4.05,6.13),tau_on=rng.uniform(.0024,.02),deadzone=rng.uniform(0,.025),damping=rng.uniform(.10,.24),coulomb=rng.uniform(0,.05))
  sg=(-1,1)[i%2];model=('effective','mass_matrix')[(i//2)%2];behavior=('zero','max','hold','sweep')[i%4]
  cases.append(dict(id=i,plant=p,sign=sg,model=model,behavior=behavior))
 (HERE/'fresh20_plan.json').write_text(json.dumps(dict(seed=2026100253,cases=cases),indent=2)+'\n')
 for cs in cases:
  r,_=run(cs['model'],cs['sign'],cs['behavior'],cs['plant']);r['case']=cs;uncertain.append(r);print(json.dumps(r),flush=True)
 result=dict(random_equivalence=random_result,nominal=rows,nominal_passed=sum(r['passed'] for r in rows),fresh20=uncertain,fresh20_passed=sum(r['passed'] for r in uncertain),elapsed_s=time.time()-t)
 result['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*') if p.suffix in('.py','.v','.cpp')}
 (HERE/'fixed_verification.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k not in('nominal','fresh20','source_sha256')},indent=2))
