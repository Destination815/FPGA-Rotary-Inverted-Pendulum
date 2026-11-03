#include "Vr01_tracking_core.h"
#include "verilated.h"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <array>
#include <cstdlib>
static uint64_t cycles=0;
static void tick(Vr01_tracking_core &d){d.clk=0;d.eval();d.clk=1;d.eval();cycles++;}
static int32_t s(uint32_t x){return static_cast<int32_t>(x);}
int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv); if(argc!=2)return 2;
 Vr01_tracking_core d;d.rst_n=0;d.start=0;d.clear_state=0;tick(d);d.rst_n=1;tick(d);
 std::ifstream f(argv[1]);std::array<int64_t,18>a;uint64_t n=0;unsigned maxlat=0;
 while(f>>a[0]){
  for(size_t i=1;i<a.size();i++)if(!(f>>a[i]))return 3;
  if(a[0]){d.clear_state=1;tick(d);d.clear_state=0;if(d.busy||d.duty_permille||d.fault)return 4;}
  d.theta=a[1];d.omega=a[2];d.phi=a[3];d.vel=a[4];d.angle_filtered=a[5];
  d.ref_pos=a[6];d.ref_vel=a[7];d.ref_acc=a[8];d.ref_jerk=a[9];
  d.research_limit=150;d.init_state=a[10];d.prior_duty_permille=a[11];d.start=1;tick(d);d.start=0;
  unsigned latency=0;while(!d.done&&latency<300){
   // Live input changes must not modify the accepted snapshot.
   d.research_limit=150;d.theta=0x7fffffff;d.omega=0x80000000;d.vel=0x12345678;
   tick(d);latency++;
  }
  if(latency>maxlat)maxlat=latency;
  bool okay=d.done&&s(d.duty_permille)==a[12]&&int(d.fault)==a[17];
  if(!a[17])okay=okay&&s(d.dbg_lean)==a[13]&&s(d.dbg_accel)==a[14]&&s(d.dbg_vreq)==a[15]&&s(d.dbg_integral)==a[16];
  if(!okay){std::cerr<<"FAIL vector "<<n<<" out "<<s(d.duty_permille)<<" expected "<<a[12]<<" fault "<<int(d.fault)<<"/"<<a[17]<<" dbg "<<s(d.dbg_lean)<<","<<s(d.dbg_accel)<<","<<s(d.dbg_vreq)<<","<<s(d.dbg_integral)<<" expected "<<a[13]<<","<<a[14]<<","<<a[15]<<","<<a[16]<<"\n";return 5;}
  tick(d);if(d.done)return 6;n++;
 }
 // Cancellation at every stage: no stale completion/output may survive.
 for(unsigned stage=0;stage<maxlat;stage++){
  d.clear_state=1;tick(d);d.clear_state=0;
  d.theta=10000;d.omega=20000;d.phi=0;d.vel=0;d.angle_filtered=10000;
  d.ref_pos=0;d.ref_vel=0;d.ref_acc=0;d.ref_jerk=0;d.init_state=1;d.prior_duty_permille=75;
  d.start=1;tick(d);d.start=0;
  for(unsigned i=0;i<stage;i++)tick(d);
  d.clear_state=1;tick(d);d.clear_state=0;
  for(unsigned i=0;i<maxlat+2;i++){tick(d);if(d.busy||d.done||d.fault||d.duty_permille)return 7;}
 }
 // Async reset cancels active computation even before the next rising edge.
 d.start=1;tick(d);d.start=0;tick(d);d.clk=0;d.eval();d.rst_n=0;d.eval();
 if(d.busy||d.done||d.fault||d.duty_permille)return 8;
 std::cout<<"PASS vectors="<<n<<" clocks="<<cycles<<" latency="<<maxlat<<" cancellation_stages="<<maxlat<<" async_reset=pass\n";
}
