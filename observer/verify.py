from pathlib import Path
import sys,subprocess,json,random,math,re,hashlib
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'observer'));from fixed_model import Observer
B=P/'build/observer';B.mkdir(parents=True,exist_ok=True);rng=random.Random(6100520);m=Observer();rows=[];previous=0
for automatic in [False,True]:
 m=Observer();previous=0;prediction_previous=0
 for j in range(6000):
  reset=int(j%1997==0)
  if reset:m.clear();previous=0;prediction_previous=0
  theta=round((4000 if j%1997>1000 else 800)*math.sin(j*.007));ref=round(3000*math.sin(j*.003)) if j>2500 else 0
  phi=ref+round((10000 if j%1997>1000 else 2000)*math.sin(j*.002));cap=150+(j//150%21)*10;rv=round(4000*math.cos(j*.003));hold=not(2500<j<3800);fallback=max(-cap,min(cap,previous+rng.randint(-20,20)))
  brake=automatic and j%2000<700
  permission=automatic and (j%1997>=500)
  u,ready,fault=m.tick(theta,phi,previous,cap,ref,rv,automatic,hold,fallback,brake,prediction_previous,permission)
  rows.append([int(permission),prediction_previous,int(brake),int(automatic),int(hold),fallback,reset,theta,phi,previous,cap,ref,rv,u,ready,fault,*m.z,m.samples,m.integral,m.lp,m.age]);prediction_previous=previous;previous=u
(B/'vectors.txt').write_text('\n'.join(' '.join(map(str,r))for r in rows)+'\n')
with (B/'build.log').open('w')as log:subprocess.run(['verilator','--cc','--exe','--build','-j','2','--Mdir',str(B/'obj'),'-Wno-fatal','--top-module','r10_observer_core',str(P/'fpga/src/r10_observer_core.v'),str(P/'observer/test_driver.cpp')],stdout=log,stderr=subprocess.STDOUT,check=True)
r=subprocess.check_output([str(B/'obj/Vr10_observer_core'),str(B/'vectors.txt')],text=True);print(r);(P/'evidence/observer_arithmetic.json').write_text(json.dumps(dict(status='PASS',sources={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [P/'fpga/src/r10_observer_core.v',P/'observer/fixed_model.py',P/'observer/test_driver.cpp',P/'observer/verify.py']},transactions=len(rows),clear_at_every_cycle=int(re.search(r'every_cycle_clear=(\d+)',r).group(1)),output=r),indent=2))
