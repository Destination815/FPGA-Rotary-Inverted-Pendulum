#include "Vstate_estimator.h"
#include "verilated.h"
#include <iostream>
#include <cstdint>
#include <stdexcept>
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);Vstate_estimator d;d.clk=0;d.rst_n=0;d.eval();auto cycle=[&](){d.clk=0;d.eval();d.clk=1;d.eval();};cycle();d.rst_n=1;d.zero_raw=746;cycle();
std::string op;int a,b,c,e;
while(std::cin>>op){int clocks=0;if(op=="U"){std::cin>>a>>b>>c>>e;d.adc=a;d.encoder=b;d.gap_allow=c;d.adc_ready=e;d.tick=1;cycle();d.tick=0;clocks=1;while(!d.step_valid&&clocks<500){cycle();clocks++;}if(!d.step_valid)throw std::runtime_error("timeout");}
else if(op=="C"){std::cin>>a;d.encoder=a;d.calibrate=1;cycle();d.calibrate=0;cycle();}
else throw std::runtime_error("opcode");
std::cout<<(int32_t)d.theta<<' '<<(int32_t)d.dtheta<<' '<<(int32_t)d.phi<<' '<<(int32_t)d.dphi<<' '<<(int32_t)d.filtered_theta_q18<<' '<<(int)d.measurement_ok<<' '<<(int)d.gap_active<<' '<<(int)d.gap_ms<<' '<<clocks<<'\n'<<std::flush;}}
