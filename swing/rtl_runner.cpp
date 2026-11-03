#include "Vr01_swing_core.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
#include <cstdint>
int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv); Vr01_swing_core d; d.clk=0;d.rst_n=0;d.eval();
 auto cycle=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};cycle();d.rst_n=1;cycle();
 std::string op; int a,b,c,e;
 while(std::cin>>op){int cycles=0;
 if(op=="S"){std::cin>>a>>b>>c>>e;d.kick_negative=a<0;d.manual_capture=b;d.phi_q18=c;d.velocity_q18=e;d.session_start=1;cycle();d.session_start=0;cycle();}
 else if(op=="U") {std::cin>>a>>b>>c>>e;d.theta_q18=a;d.phi_q18=b;d.omega_q18=c;d.velocity_q18=e;
  int32_t old_u=d.duty_permille,old_ref=d.reference_q18;int old_mode=d.mode_out;
  d.tick_start=1;cycle();d.tick_start=0;cycles=1;
  while(!d.done && cycles<15000){if((int32_t)d.duty_permille!=old_u || (int32_t)d.reference_q18!=old_ref ||d.mode_out!=old_mode)throw std::runtime_error("outputs were not atomic");cycle();cycles++;}
  if(!d.done)throw std::runtime_error("timeout");
 }
 else if(op=="X"){d.stop=1;cycle();d.stop=0;cycle();}
 else if(op=="A"){ // abort an in-flight transaction and ensure no old result commits.
  d.tick_start=1;cycle();d.tick_start=0;for(int i=0;i<25;i++)cycle();d.stop=1;cycle();d.stop=0;for(int i=0;i<250;i++)cycle();
  if(d.busy ||(int32_t)d.duty_permille!=0 ||d.mode_out!=3)throw std::runtime_error("stop abort failed");
 }
 else throw std::runtime_error("bad opcode");
 std::cout<<(int32_t)d.duty_permille<<' '<<(int)d.mode_out<<' '<<(int32_t)d.reference_q18<<' '<<(int)d.fault<<' '<<cycles<<'\n'<<std::flush;
 }
}
