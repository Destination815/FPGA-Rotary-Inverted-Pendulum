#include "hold_observer.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv){try{
 if(argc!=2)throw std::runtime_error("usage: check observer_vectors.txt");
 std::ifstream f(argv[1]);HoldObserver m;int permission,pred,brake,automatic,hold,fallback,reset,t,p,previous,cap,ref,rv,u,ready,fault;int n=0,latched=0,full_gain=0,cold_starts=0;
 while(f>>permission>>pred>>brake>>automatic>>hold>>fallback>>reset>>t>>p>>previous>>cap>>ref>>rv>>u>>ready>>fault){
  if(reset)m.clear();cold_starts+=automatic&&m.samples==0;
  int expected_z[7],samples,integral,lp,age;for(int&i:expected_z)f>>i;f>>samples>>integral>>lp>>age;
  int actual=m.policy_tick(t,p,previous,cap,ref,rv,automatic,hold,fallback,brake,pred,permission);
  if(actual!=u||m.ready!=ready||m.fault!=fault||m.samples!=samples||m.integral!=integral||m.lp!=lp||m.age!=age){
   std::cerr<<"at "<<n<<" output "<<actual<<"/"<<u<<" I="<<m.integral<<"/"<<integral<<" LP="<<m.lp<<"/"<<lp<<" age="<<m.age<<"/"<<age<<'\n';throw std::runtime_error("C++/Python transaction mismatch");}
  for(int i=0;i<7;i++)if(m.z[i]!=expected_z[i]){std::cerr<<"at "<<n<<" z"<<i<<" "<<m.z[i]<<"/"<<expected_z[i]<<'\n';throw std::runtime_error("C++/Python state mismatch");}
  latched+=m.motion_latched;full_gain+=m.motion_phase==7;n++;
 }
 if(n!=12000||!latched||!full_gain||cold_starts!=4)throw std::runtime_error("coverage changed");
 // Hard limit must dominate the slew window for both polarities and policies.
 for(bool a:{false,true})for(int prev:{-350,350}){m.clear();int q=m.tick(0,0,prev,150,a,0,0,-.5,a,false,0);if(std::abs(q)>150)throw std::runtime_error("hard cap violation");}
 // Illegal current commands remain fail-closed; prediction is not slew input.
 for(int prev:{-351,351}){m.clear();if(m.policy_tick(0,0,prev,150,0,0,true,true,0,false,0)||!m.fault)throw std::runtime_error("input guard changed");}
 std::cout<<"PASS C++/Python/previously-verified-RTL vectors="<<n<<" zero_initialized_auto_starts="<<cold_starts<<" quality_latched="<<latched<<" full_gain="<<full_gain<<" cap_and_guard=6\n";
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
