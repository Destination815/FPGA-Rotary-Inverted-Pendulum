#include "Vr10_observer_core.h"
#include "Vr10_observer_core___024root.h"
#include "verilated.h"
#include <fstream>
#include <iostream>
#include <cstdint>
static void cycle(Vr10_observer_core &t){t.clk=0;t.eval();t.clk=1;t.eval();}
int main(int argc,char **argv){
 VerilatedContext c;Vr10_observer_core t{&c};t.handoff_reblend=0;t.rst_n=0;cycle(t);t.rst_n=1;cycle(t);
 std::ifstream f(argv[1]);int mature,permission,pred,brake,automatic,hold,fallback,reset,theta,phi,previous,cap,ref,rv,omega,current;
 int n=0,maxcy=0,active=0;
 while(f>>mature>>permission>>pred>>brake>>automatic>>hold>>fallback>>reset>>theta>>phi>>previous>>cap>>ref>>rv>>omega>>current){
  int u,ready,fault,z[7],samples,integral,lp,age,good,motion_latched,motion_phase,motion_ticks,rate_good;int64_t correction;
  f>>u>>ready>>fault;for(int &v:z)f>>v;f>>samples>>integral>>lp>>age>>good>>correction>>motion_latched>>motion_phase>>motion_ticks>>rate_good;
  int dq,ds,de,dh,da,dw,dr,dm,replace;f>>dq>>ds>>de>>dh>>da>>dw>>dr>>dm>>replace;
  if(reset){t.clear=1;cycle(t);t.clear=0;cycle(t);}
  t.mature_static_hold=mature;t.motion_assist_permission=permission;t.prediction_duty=pred;t.automatic_brake=brake;t.automatic_policy=automatic;t.automatic_hold=hold;t.fallback_duty=fallback;t.theta=theta;t.phi=phi;t.reference_offset=ref;t.reference_velocity=rv;t.previous_duty=previous;t.limit_permille=cap;t.measured_omega=omega;t.measurement_current=current;
  t.start=1;cycle(t);t.start=0;int cy=0;
  while(!t.done&&cy<500){
   t.mature_static_hold=!mature;t.motion_assist_permission=!permission;t.prediction_duty=123456;t.reference_offset=0x7fffffff;t.theta=0x7fffffff;t.phi=0x80000000;t.previous_duty=123456;t.limit_permille=0;t.measured_omega=0x7fffffff;t.measurement_current=!current;
   cycle(t);cy++;
  }
  if(cy==500||int32_t(t.duty)!=u||t.ready!=ready||t.fault!=fault||int32_t(t.debug_integral)!=integral||int32_t(t.debug_lp)!=lp||t.debug_age!=age||t.debug_samples!=samples){std::cerr<<"output/integrator mismatch "<<n<<" u="<<int32_t(t.duty)<<"/"<<u<<" I="<<int32_t(t.debug_integral)<<"/"<<integral<<" LP="<<int32_t(t.debug_lp)<<"/"<<lp<<"\n";return 1;}
  for(int j=0;j<7;j++)if(int32_t(t.debug_state[j])!=z[j]){std::cerr<<"z mismatch "<<n<<" index "<<j<<"\n";return 2;}
  if(int(t.rootp->r10_observer_core__DOT__r36_fast_good)!=good){std::cerr<<"good mismatch "<<n<<"\n";return 3;}
  if(int(t.rootp->r10_observer_core__DOT__motion_latched)!=motion_latched||int(t.rootp->r10_observer_core__DOT__motion_phase)!=motion_phase||int(t.rootp->r10_observer_core__DOT__motion_ticks)!=motion_ticks){std::cerr<<"RTL motion schedule mismatch "<<n<<" phase="<<int(t.rootp->r10_observer_core__DOT__motion_phase)<<"/"<<motion_phase<<" latch="<<int(t.rootp->r10_observer_core__DOT__motion_latched)<<"/"<<motion_latched<<" ticks="<<int(t.rootp->r10_observer_core__DOT__motion_ticks)<<"/"<<motion_ticks<<"\n";return 6;}
  if(int(t.rootp->r10_observer_core__DOT__r37_rate_was_good)!=rate_good){std::cerr<<"rate history mismatch "<<n<<"\n";return 7;}
  if(int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__quiet)!=dq || int8_t(t.rootp->r10_observer_core__DOT__damping_gate__DOT__sign << 6)/64!=ds || int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__elapsed)!=de || int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__halves)!=dh || int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__active)!=da || int(t.rootp->r10_observer_core__DOT__damping_weight)!=dw || int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__ramp)!=dr || int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__mode)!=dm){std::cerr<<"damping state mismatch "<<n<<" weight="<<int(t.rootp->r10_observer_core__DOT__damping_weight)<<"/"<<dw<<" elapsed="<<int(t.rootp->r10_observer_core__DOT__damping_gate__DOT__elapsed)<<"/"<<de<<"\n";return 8;}
  if(int32_t(t.rootp->r10_observer_core__DOT__tf_old)!=replace){std::cerr<<"RTL rate replacement mismatch "<<n<<" got="<<int32_t(t.rootp->r10_observer_core__DOT__tf_old)<<" expected="<<replace<<"\n";return 10;}
  active+=correction!=0;maxcy=std::max(maxcy,cy);cycle(t);n++;
 }
 for(int cut=0;cut<=maxcy;cut++){
  t.clear=1;cycle(t);t.clear=0;t.mature_static_hold=1;t.motion_assist_permission=0;t.automatic_policy=1;t.automatic_hold=1;t.automatic_brake=0;t.measured_omega=120000;t.measurement_current=1;t.reference_offset=0;t.reference_velocity=0;t.theta=100;t.phi=300;t.previous_duty=0;t.prediction_duty=0;t.limit_permille=150;t.start=1;cycle(t);t.start=0;
  for(int i=0;i<cut;i++)cycle(t);t.clear=1;cycle(t);
  if(t.busy||t.done||t.duty||t.fault||t.ready||t.rootp->r10_observer_core__DOT__r37_rate_was_good||t.rootp->r10_observer_core__DOT__r36_fast_good||t.rootp->r10_observer_core__DOT__r36_fast_enabled_s||t.rootp->r10_observer_core__DOT__motion_latched||t.rootp->r10_observer_core__DOT__motion_phase||t.rootp->r10_observer_core__DOT__motion_ticks)return 4;
  if(t.rootp->r10_observer_core__DOT__damping_weight||t.rootp->r10_observer_core__DOT__damping_gate__DOT__quiet||t.rootp->r10_observer_core__DOT__damping_gate__DOT__elapsed||t.rootp->r10_observer_core__DOT__damping_gate__DOT__halves||t.rootp->r10_observer_core__DOT__damping_gate__DOT__active||t.rootp->r10_observer_core__DOT__damping_gate__DOT__mode||t.rootp->r10_observer_core__DOT__r38_current_s||t.rootp->r10_observer_core__DOT__r38_explicit_endpoint_s)return 9;
  t.clear=0;for(int i=0;i<maxcy+4;i++){cycle(t);if(t.done||t.duty)return 5;}
 }
 std::cout<<"PASS transactions="<<n<<" active_residual="<<active<<" maxcycles="<<maxcy<<" clear_cuts="<<maxcy+1<<"\n";
}
