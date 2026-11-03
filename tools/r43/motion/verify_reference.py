from pathlib import Path
import argparse,importlib.util,hashlib,json,math,subprocess
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();P=a.project.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
spec=importlib.util.spec_from_file_location('ref43',P/'reference/reference_model.py');rm=importlib.util.module_from_spec(spec);spec.loader.exec_module(rm)
Q=1<<18;K=97763/101376;qr=lambda deg:round(math.radians(deg)/K*Q);m=rm.Reference();rows=0;cases=[];errors=[0.]*4
f=(O/'vectors.dat').open('w')
def push(kw={},age=-1,phase=-1):
 global rows
 if age>=0:m.age=age
 if phase>=0:m.phase=phase
 names=['reseed','seed','target_valid','target','shuttle_valid','shuttle_on','amplitude','speed_level','period_seconds','target_bound','speed_limit','ext2_centered','ext2_phase_step','anchor_from_seed']
 cfg=dict(amplitude=qr(90),speed_level=3,period_seconds=35,target_bound=qr(360),anchor_from_seed=1);cfg.update(kw)
 out=m.tick(**cfg)
 row=[age,phase]+[int(cfg.get(k,0))for k in names]+out+[int(m.mode==0),int(m.mode==2),int(m.reject),int(m.center)]
 f.write(' '.join(map(str,row))+'\n');rows+=1
 for i in range(4):errors[i]=max(errors[i],abs(out[i]/Q-m.floatout[i]))
 return out

for speed in [15,30,45,60]:
 for old,actual,target in [(101.830,99.493,90),(-98,-93,-90),(-90,-93,0),(0,3,0),(90,95,90),(-170,-166,90)]:
  cfg=dict(speed_limit=qr(speed));push(dict(reseed=1,seed=qr(old),**cfg));r=push(dict(seed=qr(actual),target_valid=1,target=qr(target),**cfg));assert r[0]==qr(actual)and r[1]==r[2]==0
  T=m.duration
  for tick in sorted(set([1,2,50,250,500,T//4,T//2,T-2,T-1,T])):
   push(cfg,age=tick)
  assert m.out==[qr(target),0,0,0]
  # Same target is still a real measured-state correction, never a hidden deadband.
  r=push(dict(seed=qr(actual),target_valid=1,target=qr(target),**cfg));assert r[0]==qr(actual)
  push(dict(seed=qr(-actual),target_valid=1,target=0,**cfg));assert m.reject
  push(cfg,age=m.duration)
  cases.append(dict(kind='position',speed=speed,prior=old,measured=actual,target=target,duration_ms=T))

for old,center,amplitude,period in [(6.677,0,90,18.85),(-25,30,90,9.42),(101,90,30,12.57),(-40,-40,90,37.70),(3,-90,120,18.85),(10,0,180,120)]:
 cfg=dict(seed=qr(center),ext2_centered=1,ext2_phase_step=round(2**32/(1000*period)),amplitude=qr(amplitude))
 push(dict(reseed=1,seed=qr(old)))
 r=push(dict(shuttle_valid=1,shuttle_on=1,**cfg));assert r[:3]==[qr(old),0,0]and m.center==qr(center)
 # Complete entry proves smooth derivatives, then multiple phase points.
 for _ in range(2002):push(cfg)
 for phase in range(0,2**32,2**26):push(cfg,age=2000,phase=phase)
 before=m.out.copy();push(dict(shuttle_valid=1,shuttle_on=0,**cfg));assert m.origin==qr(center)
 for _ in range(2002):push(cfg)
 assert m.out==[qr(center),0,0,0]
 cases.append(dict(kind='centered_sine',prior=old,center=center,amplitude=amplitude,period=period))

# Centered geometry must be checked at desired center, not the abandoned catch.
push(dict(reseed=1,seed=qr(170)))
push(dict(seed=0,shuttle_valid=1,shuttle_on=1,ext2_centered=1,ext2_phase_step=227850,amplitude=qr(90),target_bound=qr(180)));assert not m.reject
push(dict(reseed=1,seed=0))
push(dict(seed=qr(170),shuttle_valid=1,shuttle_on=1,ext2_centered=1,ext2_phase_step=227850,amplitude=qr(90),target_bound=qr(180)));assert m.reject
f.close()
cpp=r'''#include "Vr01_reference.h"
#include "Vr01_reference___024root.h"
#include "verilated.h"
#include <fstream>
#include <iostream>
#include <cstdint>
#include <algorithm>
#define X(n) d.rootp->r01_reference__DOT__##n
int main(int argc,char**argv){Vr01_reference d;auto tick=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};d.rst_n=0;tick();d.rst_n=1;d.clear=0;tick();std::ifstream f(argv[1]);int64_t a[24];int n=0,maximum=0;
while(f>>a[0]){for(int i=1;i<24;i++)f>>a[i];if(a[0]>=0)X(age)=a[0];if(a[1]>=0)X(phase)=a[1];d.reseed=a[2];d.seed_phi=a[3];d.target_valid=a[4];d.target=a[5];d.shuttle_valid=a[6];d.shuttle_on=a[7];d.amplitude=a[8];d.speed_level=a[9];d.period_seconds=a[10];d.target_bound=a[11];d.speed_limit=a[12];d.ext2_centered=a[13];d.ext2_phase_step=a[14];d.anchor_from_seed=a[15];d.start=1;tick();d.start=0;int c=0;
while(!d.done&&c<20000){d.seed_phi=0x7fffffff;d.target=0x80000000;d.anchor_from_seed=0;d.target_valid=d.shuttle_valid=0;tick();c++;}
int32_t out[8]={int32_t(d.__SYM__reference),int32_t(d.ref_velocity),int32_t(d.ref_acceleration),int32_t(d.ref_jerk),int(d.stationary),int(d.trajectory_running),int(d.command_rejected),int32_t(d.ext2_center)};
if(!d.done){std::cerr<<"timeout "<<n;return 2;}for(int i=0;i<8;i++)if(out[i]!=a[16+i]){std::cerr<<"mismatch "<<n<<" field "<<i<<" got "<<out[i]<<" expected "<<a[16+i];return 1;}maximum=std::max(maximum,c);tick();n++;}
std::cout<<"{\"transactions\":"<<n<<",\"max_cycles\":"<<maximum<<"}\n";}
'''
(O/'main.cpp').write_text(cpp)
cmd=['verilator','--cc','--exe','--build','-j','2','--public-flat-rw','-Wno-fatal','--top-module','r01_reference','--Mdir',str(O/'obj'),str(P/'fpga/src/r01_reference.v'),str(O/'main.cpp'),'-o','check']
with(O/'build.log').open('w')as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
r=subprocess.run([str(O/'obj/check'),str(O/'vectors.dat')],capture_output=True,text=True);(O/'run.log').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
result=dict(status='PASS',rtl=json.loads(r.stdout),cases=cases,model_float_max_error=errors,sources={str(p.relative_to(P)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [P/'fpga/src/r01_reference.v',P/'reference/reference_model.py']},scope='Actual reference RTL and independent integer oracle; internal age/phase jumped only in declared sparse duration/phase rows. Full centered entry/exit executed. Not physical motion validation.')
(O/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['rtl']))
