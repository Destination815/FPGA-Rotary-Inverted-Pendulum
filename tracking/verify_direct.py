from pathlib import Path
import json,subprocess,random,math
from fixed_tracking import FixedController,Q,LIM_IN,tanh_q,p
H=Path(__file__).resolve().parent;B=H/'build';B.mkdir(exist_ok=True);rng=random.Random(44107);c=FixedController();rows=0
with (B/'direct_vectors.txt').open('w') as f:
 def put(v,reset=False,init=False,prior=0):
  global rows
  if reset:c.clear()
  u=c.tick_q(*v,init_state=init,prior_duty_permille=prior);d=c.diagnostics
  expected=[u,d.get('lean',0),d.get('acceleration_request',0),0,0,int(c.fault)]
  f.write(' '.join(map(str,[int(reset),*v,int(init),prior,*expected]))+'\n');rows+=1
 for i in range(18000):put([rng.randint(-b,b) for b in LIM_IN],i%17==0,i%11==0,rng.randint(-150,150))
 for col,b in enumerate(LIM_IN):
  for x in [-2147483648,-b-1,-b,-1,0,1,b,b+1,2147483647]:
   v=[0]*9;v[col]=x;put(v,True);put([0]*9);put([0]*9,True)
 for sign in [-1,1]:
  for i in range(5000):put([0,0,0,0,round(sign*.1*math.pi/180*Q),0,0,0,0],i==0)
log=B/'direct_verification.log'
with log.open('w') as f:
 subprocess.run(['verilator','--cc','--exe','--build','-j','3','--top-module','r01_tracking_core','--Mdir',str(B/'obj'),str(H/'r01_tracking_core.v'),str(H/'test_driver.cpp')],stdout=f,stderr=subprocess.STDOUT,check=True)
 subprocess.run([str(B/'obj/Vr01_tracking_core'),str(B/'direct_vectors.txt')],stdout=f,stderr=subprocess.STDOUT,check=True)
report=dict(vectors=rows,result=log.read_text().splitlines()[-1],lookup_max_error=max(abs(tanh_q(x)/Q-math.tanh(x/Q/p[5])) for x in range(-Q,Q+1)),scope='exact integer/RTL matching including limits, sticky faults, handoff, fixed small offset')
(H/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
