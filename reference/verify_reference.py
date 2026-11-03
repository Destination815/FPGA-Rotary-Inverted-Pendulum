#!/usr/bin/env python3
from pathlib import Path
import json,sys,math,subprocess,hashlib
from reference_model import Reference,Q,qr
P=Path(__file__).resolve().parent;B=P/'build';B.mkdir(exist_ok=True)
m=Reference();n=0;maxerror=[0.]*4;maxv=maxa=0.;durations=set();entry_peaks={};steady_v=0.
def push(f,args={}):
 global n,maxv,maxa,steady_v
 inp=[int(args.get(k,0)) for k in ('reseed','seed','target_valid','target','shuttle_valid','shuttle_on')]
 inp += [int(args.get('amplitude',qr(math.radians(15)))),int(args.get('speed_level',0)),int(args.get('period_seconds',20)),int(args.get('target_bound',qr(math.pi)))];out=m.tick(*inp);f.write(' '.join(map(str,inp+out+[int(m.mode==0),int(m.mode==2),int(m.reject)]))+'\n');n+=1
 for i in range(4):maxerror[i]=max(maxerror[i],abs(out[i]/Q-m.floatout[i]))
 if m.mode==2:
  key=f'{math.degrees(m.origin/Q):+.3f}'
  if m.age<2000:
   e=entry_peaks.setdefault(key,[0.,0.,0.])
   for k in range(3):e[k]=max(e[k],abs(out[k+1]/Q))
  else:steady_v=max(steady_v,abs(out[1]/Q))
 if m.mode==1:maxv=max(maxv,abs(out[1]/Q));maxa=max(maxa,abs(out[2]/Q));durations.add(m.duration)
with (B/'vectors.txt').open('w') as f:
 # Full-distance trajectories and configurable shuttle at every supported period.
 for seed,target,speed in [(-180,180,0),(180,-180,2),(0,90,1),(0,-30,0),(0,0,0)]:
  cfg=dict(speed_level=speed)
  push(f,dict(reseed=1,seed=qr(math.radians(seed)),**cfg));push(f,dict(target_valid=1,target=qr(math.radians(target)),**cfg))
  for _ in range(96010):
   push(f,cfg)
   if m.mode==0:break
 for period in (8,10,12,16,20,30,40,60,90,120):
  cfg=dict(amplitude=qr(math.radians(min(180,period*30/9))),speed_level=2,period_seconds=period)
  push(f,dict(reseed=1,**cfg));push(f,dict(shuttle_valid=1,shuttle_on=1,**cfg))
  for _ in range(3200):push(f,cfg)
  push(f,dict(shuttle_valid=1,shuttle_on=0,**cfg))
  for _ in range(2010):push(f,cfg)
 push(f,dict(reseed=1));push(f,dict(target_valid=1,target=qr(math.pi)+1));assert m.reject
 push(f,dict(target_valid=1,target=-qr(math.pi)-1));assert m.reject
assert maxv<math.radians(30)+.001,(maxv,maxa)
assert maxa<math.radians(30)+.001
assert maxerror[0]<.002,maxerror
(B/'main.cpp').write_text(r'''
#include "Vr01_reference.h"
#include "verilated.h"
#include <fstream>
#include <iostream>
#include <cstdint>
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);Vr01_reference d;
 auto tick=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};d.rst_n=0;tick();d.rst_n=1;d.clear=0;tick();
 std::ifstream f(argv[1]);long long a[17];int n=0,maxcycles=0;
 while(f>>a[0]){for(int i=1;i<17;i++)f>>a[i];d.start=1;d.reseed=a[0];d.seed_phi=a[1];d.target_valid=a[2];d.target=a[3];d.shuttle_valid=a[4];d.shuttle_on=a[5];d.amplitude=a[6];d.speed_level=a[7];d.period_seconds=a[8];d.target_bound=a[9];tick();d.start=0;
 int c=0;while(!d.done&&c<10000){d.seed_phi=0x7fffffff;d.target=0x80000000;d.target_bound=0;d.amplitude=123456;d.speed_level=3;d.period_seconds=255;d.reseed=0;d.target_valid=0;d.shuttle_valid=0;tick();c++;}if(c>maxcycles)maxcycles=c;
 int32_t o[7]={(int32_t)d.__SYM__reference,(int32_t)d.ref_velocity,(int32_t)d.ref_acceleration,(int32_t)d.ref_jerk,(int)d.stationary,(int)d.trajectory_running,(int)d.command_rejected};
 for(int i=0;i<7;i++)if(o[i]!=a[i+10]){std::cerr<<"mismatch row "<<n<<" field "<<i<<" got "<<o[i]<<" wanted "<<a[i+10]<<"\n";return 1;}
 if(!d.done){std::cerr<<"timeout\n";return 2;}tick();n++;}
 // Abort every clock of the longest motion transaction, then verify no stale output.
 for(int at=0;at<=maxcycles;at++){
  d.clear=1;tick();d.clear=0;d.reseed=1;d.seed_phi=0;d.target_valid=0;d.shuttle_valid=0;d.start=1;tick();d.start=0;
  while(!d.done)tick();tick();d.reseed=0;d.target_valid=1;d.target=13726;d.start=1;tick();d.start=0;
  for(int k=0;k<at;k++)tick();d.clear=1;tick();d.clear=0;d.target_valid=0;
  for(int k=0;k<16;k++){tick();if(d.busy||d.done||d.__SYM__reference||d.ref_velocity||!d.stationary)return 3;}
 }
 std::cout<<"{\"samples\":"<<n<<",\"clear_clocks\":"<<maxcycles+1<<",\"max_clocks\":"<<maxcycles<<",\"matched\":true}\n";
}
''')
cmd=['verilator','--cc','--exe','--build','-Wno-fatal','-Wno-SYMRSVDWORD','--top-module','r01_reference','--Mdir',str(B/'obj'),str(P.parent/'fpga/src/r01_reference.v'),str(B/'main.cpp'),'-o','verify_reference']
r=subprocess.run(cmd,capture_output=True,text=True,errors="replace");(B/'build.log').write_text(r.stdout+r.stderr);r.check_returncode()
r=subprocess.run([str(B/'obj/verify_reference'),str(B/'vectors.txt')],capture_output=True,text=True,errors="replace");(B/'run.log').write_text(r.stdout+r.stderr);r.check_returncode()
result=dict(rtl=json.loads(r.stdout),max_float_error_rad_units=maxerror,max_position_velocity_rad_s=maxv,max_position_acceleration_rad_s2=maxa,duration_bins_tested_ms=sorted(durations),sine_entry_peak_v_a_j_rad_units=entry_peaks,steady_sine_peak_velocity_rad_s=steady_v,sha256={str(p.relative_to(P.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (P.parent/'fpga/src/r01_reference.v',P/'reference_model.py',P/'verify_reference.py')})
(P/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
