#include "hold_observer.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char **argv){
 std::ifstream f(argv[1]);HoldObserver m;int n=0;
 int mature,permission,pred,brake,automatic,hold,fallback,reset,t,p,previous,cap,ref,rv,omega,current;
 while(f>>mature>>permission>>pred>>brake>>automatic>>hold>>fallback>>reset>>t>>p>>previous>>cap>>ref>>rv>>omega>>current){
  int u,ready,fault,z[7],samples,integral,lp,age,good,motion_latched,motion_phase,motion_ticks,rate_good;int64_t correction;
  f>>u>>ready>>fault;for(int &v:z)f>>v;f>>samples>>integral>>lp>>age>>good>>correction>>motion_latched>>motion_phase>>motion_ticks>>rate_good;
  int dq,ds,de,dh,da,dw,dr,dm,replace;f>>dq>>ds>>de>>dh>>da>>dw>>dr>>dm>>replace;
  if(reset)m.clear();int actual=m.policy_tick(t,p,previous,cap,ref,rv,automatic,hold,fallback,brake,pred,permission,mature,omega,current);
  if(actual!=u||m.ready!=ready||m.fault!=fault||m.samples!=samples||m.integral!=integral||m.lp!=lp||m.age!=age||int(m.fast_good)!=good||m.fast_correction!=correction){std::cerr<<"C++ mismatch "<<n<<" u="<<actual<<"/"<<u<<" correction="<<m.fast_correction<<"/"<<correction<<"\n";return 1;}
  for(int j=0;j<7;j++)if(m.z[j]!=z[j])return 2;
  if(int(m.motion_latched)!=motion_latched||m.motion_phase!=motion_phase||m.motion_ticks!=motion_ticks){std::cerr<<"C++ motion schedule mismatch "<<n<<" phase="<<m.motion_phase<<"/"<<motion_phase<<" latch="<<int(m.motion_latched)<<"/"<<motion_latched<<" ticks="<<m.motion_ticks<<"/"<<motion_ticks<<"\n";return 3;}
  if(int(m.rate_good)!=rate_good)return 4;
  const auto& g=m.damping_gate;
  if(g.quiet!=dq||g.sign!=ds||g.elapsed!=de||g.halves!=dh||int(g.active)!=da||g.weight!=dw||g.ramp!=dr||int(g.mode)!=dm){std::cerr<<"C++ damping state mismatch "<<n<<"\n";return 5;}
  if(m.rate_replacement_delta!=replace){std::cerr<<"C++ rate replacement mismatch "<<n<<"\n";return 6;}
  n++;
 }
 std::cout<<"PASS C++ transactions="<<n<<"\n";
}
