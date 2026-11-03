from pathlib import Path
import argparse,hashlib,json,importlib.util,math,subprocess,time
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();P=a.project.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
spec=importlib.util.spec_from_file_location('r42_final_observer',P/'observer/fixed_model.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
files=['fpga/src/r10_observer_core.v','observer/fixed_model.py','observer/r39_ext2_model.py'];before={f:hashlib.sha256((P/f).read_bytes()).hexdigest()for f in files}
rows=[];profiles=['continuous','measurement_gap','measured_bound','hard_cap','sine_switch','warm_only','negative_rounding','fast_theta','mode_switch','sine_phase','independent_prediction','extreme_rates']
for name in profiles:
 ob=m.Observer();previous=prediction=0
 for i in range(2200):
  reset=i==0 or i==1400
  if reset:ob.clear();previous=prediction=0
  t=round(2300*math.sin(i*.027));ref=round(95000*math.sin(i*.0013));p=ref+round(7000*math.sin(i*.011));rv=round(110000*math.cos(i*.0013));omega=round(61000*math.cos(i*.027));cur=True;warm=enable=motion=True;cap=350;sr=ref//4
  if name=='measurement_gap':cur=i%5!=0
  if name=='measured_bound':omega=[-1966081,-1966080,-1966079,1966079,1966080,1966081][i%6]
  if name=='hard_cap':cap=150 if i%127<64 else 350
  if name=='sine_switch':enable=i%1000>=500
  if name=='warm_only':enable=motion=False
  if name=='negative_rounding':omega=-i;t=-100;p=-3000;ref=rv=sr=0
  if name=='fast_theta':t=round(12000*math.sin(i*.017));omega=round(450000*math.cos(i*.017))
  if name=='mode_switch':warm=enable=motion=i%1000>=500;ref=sr=0;rv=0
  if name=='sine_phase':sr=round(80000*math.cos(i*.0041))
  if name=='independent_prediction':prediction=-previous
  if name=='extreme_rates':omega=1966081 if i%2 else -1966081
  u,ready,fault=ob.tick(t,p,previous,cap,ref,rv,True,True,0,False,prediction,motion_assist_permission=False,measurement_current=cur,measured_omega=omega,ext2_warm=warm,ext2_enable=enable,ext2_motion=motion,ext2_reference=sr)
  assert not fault and abs(u)<=cap,(name,i,u,fault)
  b=ob.ext2
  extra=[b.z[7],b.feedback_lp,b.shape_fast,b.shape_slow,int(b.shape_valid)+2*int(b.sine_valid)+4*int(b.smooth_valid)+8*int(b.smooth_sine_active)+16*int(b.bias_clipped)+32,b.sine_bias,b.sine_position_ff,b.sine_reference,b.smooth_position,b.smooth_velocity_step,b.smooth_correction,b.smooth_velocity_normalized,b.smooth_previous_feedback,b.smooth_bound,b.smooth_innovation,ob.transfer_ticks+256*int(ob.r40_unwind_active)]if warm else [0]*16
  rows.append([int(reset),t,p,previous,prediction,cap,ref,rv,omega,int(cur),int(warm),int(enable),int(motion),sr,u,ready,fault,*ob.z,ob.integral,ob.lp,ob.age,ob.samples,*extra]);prediction=previous;previous=u
(O/'vectors.dat').write_text('\n'.join(' '.join(map(str,r))for r in rows)+'\n');assert len(rows[0])==44
cpp=r'''#include "Vr10_observer_core.h"
#include "Vr10_observer_core___024root.h"
#include "verilated.h"
#include <fstream>
#include <iostream>
#include <cstdint>
#define X(n) d.rootp->r10_observer_core__DOT__##n
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);Vr10_observer_core d;auto ck=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};d.rst_n=0;d.clear=0;d.start=0;ck();d.rst_n=1;ck();d.automatic_policy=1;d.automatic_hold=1;d.automatic_brake=0;d.motion_assist_permission=d.handoff_reblend=d.mature_static_hold=d.rate_hold=0;d.fallback_duty=0;std::ifstream f(argv[1]);int64_t x[44];int n=0,maxcycles=0;while(f>>x[0]){for(int j=1;j<44;j++)f>>x[j];if(x[0]){d.clear=1;ck();d.clear=0;ck();}d.theta=x[1];d.phi=x[2];d.previous_duty=x[3];d.prediction_duty=x[4];d.limit_permille=x[5];d.reference_offset=x[6];d.reference_velocity=x[7];d.measured_omega=x[8];d.measurement_current=x[9];d.ext2_warm=x[10];d.ext2_enable=x[11];d.ext2_motion=x[12];d.ext2_reference=x[13];d.start=1;ck();d.start=0;int k=0;while(!d.done&&k++<2000)ck();maxcycles=std::max(maxcycles,k);bool ok=d.done&&(int32_t)d.duty==x[14]&&d.ready==x[15]&&d.fault==x[16];for(int j=0;j<7;j++)ok&=(int32_t)d.debug_state[j]==x[17+j];ok&=(int32_t)d.debug_integral==x[24]&&(int32_t)d.debug_lp==x[25]&&d.debug_age==x[26]&&d.debug_samples==x[27];if(x[10])for(int j=0;j<16;j++){int32_t actual=d.debug_ext2[j];if(j==4)actual&=63;if(actual!=x[28+j]){std::cerr<<"Astate "<<j<<" "<<actual<<"/"<<x[28+j]<<"\n";ok=false;}}if(!ok){std::cerr<<"mismatch "<<n<<" u="<<(int32_t)d.duty<<"/"<<x[14]<<" I="<<(int32_t)d.debug_integral<<"/"<<x[24]<<" lp="<<(int32_t)d.debug_lp<<"/"<<x[25]<<" rate="<<(int32_t)X(tf_old)<<" n1="<<(int32_t)X(n1)<<" ma21="<<(int32_t)X(r39t7_ma21_prepared)<<"\n";return 1;}ck();n++;}
for(int cut=0;cut<=maxcycles;cut++){d.clear=1;ck();d.clear=0;d.ext2_warm=d.ext2_enable=d.ext2_motion=1;d.theta=100;d.phi=500;d.measured_omega=30000;d.measurement_current=1;d.start=1;ck();d.start=0;for(int j=0;j<cut&&!d.done;j++)ck();d.clear=1;ck();d.clear=0;ck();if(d.busy||d.fault||d.duty||d.debug_integral||d.debug_lp||d.debug_samples){std::cerr<<"clearfailed";return 2;}for(int j=0;j<16;j++)if(d.debug_ext2[j])return 3;}
std::cout<<"{\"status\":\"PASS\",\"transactions\":"<<n<<",\"max_cycles\":"<<maxcycles<<",\"abort_cuts\":"<<maxcycles+1<<"}\n";}
'''
(O/'check.cpp').write_text(cpp)
cmd=['verilator','--cc','--exe','--build','--assert','--public-flat-rw','-j','2','-Wno-fatal','--top-module','r10_observer_core','--Mdir',str(O/'obj'),str(P/files[0]),str(O/'check.cpp'),'-o','check']
with (O/'build.log').open('w')as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
r=subprocess.run([str(O/'obj/check'),str(O/'vectors.dat')],capture_output=True,text=True);(O/'run.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);r.check_returncode()
after={f:hashlib.sha256((P/f).read_bytes()).hexdigest()for f in files};receipt={'result':json.loads(r.stdout),'profiles':profiles,'rows':len(rows),'before':before,'after':after,'changed_during_test':before!=after,'scope':'Actual final observer RTL, Python arbitrary-precision oracle and clear at every reachable completion clock. No plant or actualGowin.'};(O/'RECEIPT.json').write_text(json.dumps(receipt,indent=2))
