#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
static VerilatedContext c;static Vr01_control_harness t{&c};
void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
void init(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=1;clk();t.begin_swing=0;clk();clk();clk();}
void tick(){t.step=1;clk();t.step=0;int n=0;do{clk();if(++n>49000)throw 1;}while(!t.done);for(int j=0;j<64;j++)clk();}
int main(){try{int cases=0;for(int delay:{0,1,2,200,410,411,416,417})for(int how=0;how<3;how++){
 init();tick();t.theta=20;t.filtered=80;for(int j=0;j<850;j++)tick();if(t.fault)throw 2;
 t.step=1;clk();t.step=0;int n=0;while(t.rootp->r01_control_harness__DOT__control__DOT__phase!=4){clk();if(++n>49000)throw 3;}
 for(int j=0;j<delay;j++)clk();if(how==0)t.stop=1;else if(how==1)t.armed=0;else t.rst_n=0;clk();if(t.duty)throw 4;
 t.stop=0;for(int j=0;j<26000;j++){clk();if(t.duty)throw 5;}t.step=1;clk();t.step=0;if(t.duty)throw 6;cases++;
 }std::cout<<"PASS observer handoff STOP/disarm/reset stages="<<cases<<"\n";
 }catch(int e){std::cerr<<"FAIL "<<e<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<"\n";return e;}}
