from pathlib import Path
import importlib.util as U,hashlib,json,subprocess,math,time
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path);ap.add_argument('--out',type=Path);args=ap.parse_args()
A=Path(__file__).resolve().parent;P=(args.project or A.parents[2]).resolve();TOOLS=A;B=(args.out or A/'contract').resolve();B.mkdir(parents=True,exist_ok=True)
spec=U.spec_from_file_location('r42ordinary',P/'observer/fixed_model.py');m=U.module_from_spec(spec);spec.loader.exec_module(m);vectors=[];cover=[]
profiles=['mature_static','explicit_forward','explicit_reverse','theta_boundary','omega_boundary','current_gap','manual','brake','fresh_capture','motion_then_hold','postmove_wave','hard_cap','negative_rounding','reference_change','prediction_difference','omega_range']
for name in profiles:
 ob=m.Observer();prev=pred=0;act=0
 for j in range(2400):
  theta=round(1800*math.sin(j*.027));phi=round(1300*math.sin(j*.009));omega=round(70000*math.cos(j*.027));mature=j>=200;permission=False;rv=0;ref=0;auto=True;brake=False;cur=True;cap=150 if j%500<250 else 350
  if name.startswith('explicit'):permission=True;rv=34000 if name=='explicit_forward'else -34000
  if name=='theta_boundary':theta=[-9152,-9151,-9150,9150,9151,9152][j%6]
  if name=='omega_boundary':omega=[-163841,-163840,-163839,163839,163840,163841][j%6]
  if name=='current_gap':cur=j%4!=0;permission=True;rv=6554
  if name=='manual':auto=False
  if name=='brake':brake=True;permission=True
  if name=='fresh_capture':mature=False;permission=False
  if name=='motion_then_hold':permission=True;rv=6554 if j<1200 else 0
  if name=='postmove_wave':permission=True;mature=True;theta=round(2300*math.sin(2*math.pi*j/220));omega=round(2300*(2*math.pi/.22)*math.cos(2*math.pi*j/220))
  if name=='hard_cap':permission=True;omega=1000000 if j%100<50 else -1000000
  if name=='negative_rounding':theta=-100;phi=-3000;omega=-j
  if name=='reference_change':permission=True;ref=4000 if j>=1200 else 0
  if name=='prediction_difference':permission=True;pred=-prev
  if name=='omega_range':permission=True;omega=[-1966081,-1966080,-1966079,1966079,1966080,1966081][j%6]
  reset=j in(0,1600)
  if reset:ob.clear();prev=pred=0
  ob.mature_static_hold=mature;u,ready,fault=ob.tick(theta,phi,prev,cap,ref,rv,auto,True,0,brake,pred,permission,omega,cur)
  assert not fault and abs(u)<=cap and abs(ob.rate_replacement_delta)<=243012
  scope=auto and not brake and cur and abs(omega)<=1966080 and(permission or(mature and rv==0 and abs(theta)<=9151 and abs(omega)<163840))
  act+=ob.rate_replacement_delta!=0;g=ob.damping_gate
  vectors.append([int(mature),int(permission),pred,int(brake),int(auto),1,0,int(reset),theta,phi,prev,cap,ref,rv,omega,int(cur),u,ready,fault,*ob.z,ob.samples,ob.integral,ob.lp,ob.age,ob.fast_good,ob.fast_correction,int(ob.motion_latched),ob.motion_phase,ob.motion_ticks,int(ob.rate_good),g.quiet,g.sign,g.elapsed,g.halves,int(g.active),g.weight,g.ramp,int(g.mode),ob.rate_replacement_delta]);pred=prev;prev=u
 cover.append(dict(profile=name,rows=2400,active=act))
(B/'vectors.dat').write_text('\n'.join(' '.join(map(str,r))for r in vectors)+'\n')
cmd=['verilator','--cc','--exe','--build','-j','2','--public-flat-rw','--Mdir',str(B/'obj'),'-Wno-fatal','--top-module','r10_observer_core',str(P/'fpga/src/r10_observer_core.v'),str(TOOLS/'verify_postmove_damping_rtl.cpp')]
with (B/'build.log').open('w')as f:
 subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
 subprocess.run(['c++','-O2','-std=c++17','-I'+str(P/'hybrid'),str(TOOLS/'verify_postmove_damping_cpp.cpp'),'-o',str(B/'check_cpp')],stdout=f,stderr=subprocess.STDOUT,check=True)
cpp=subprocess.check_output([str(B/'check_cpp'),str(B/'vectors.dat')],text=True);print(cpp,flush=True)
rtl=subprocess.check_output([str(B/'obj/Vr10_observer_core'),str(B/'vectors.dat')],text=True);print(rtl,flush=True)
(B/'RECEIPT.json').write_text(json.dumps(dict(status='PASS',transactions=len(vectors),coverage=cover,cpp=cpp,rtl=rtl,source_sha256={f:hashlib.sha256((P/f).read_bytes()).hexdigest()for f in ['observer/fixed_model.py','hybrid/hold_observer.hpp','fpga/src/r10_observer_core.v']}),indent=2))
