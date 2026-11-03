#include "Vr01_swing_core.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
int main(){Vr01_swing_core d;auto cy=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};d.rst_n=0;cy();d.rst_n=1;cy();
auto start=[&](){d.stop=0;d.session_start=1;d.manual_capture=0;d.kick_negative=1;d.phi_q18=0;d.velocity_q18=0;cy();d.session_start=0;cy();d.theta_q18=-818301;d.omega_q18=0;d.tick_start=1;cy();d.tick_start=0;};
auto finish=[&](){int n=0;while(!d.done&&n<20000){cy();n++;}if(n==20000)throw std::runtime_error("timeout");return n;};
start();int maxcy=finish();int u=d.duty_permille,ref=d.reference_q18,mode=d.mode_out;
for(int at=0;at<=maxcy;at++){
 start();for(int j=0;j<at;j++)cy();d.stop=1;cy();if(d.busy||d.duty_permille||d.mode_out!=3)throw std::runtime_error("stop");d.stop=0;
 for(int j=0;j<12;j++){cy();if(d.busy||d.duty_permille||d.done)throw std::runtime_error("stale completion");}
 start();finish();if(int(d.duty_permille)!=u||int(d.reference_q18)!=ref||d.mode_out!=mode||d.fault)throw std::runtime_error("restart residue");
}
// Reset and session restart in ROM fetch, RAM write, multiply and lookup phases.
for(int at=0;at<maxcy;at+=17){start();for(int j=0;j<at;j++)cy();d.rst_n=0;cy();if(d.busy||d.duty_permille||d.fault)throw std::runtime_error("reset");d.rst_n=1;cy();start();finish();if(int(d.duty_permille)!=u||d.fault)throw std::runtime_error("reset residue");}
std::cout<<"PASS every_clock_STOP_and_restart="<<maxcy+1<<" maxcycles="<<maxcy<<" reset_phases="<<(maxcy+16)/17<<"\n";}
