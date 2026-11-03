#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
static VerilatedContext c;static Vr01_control_harness t{&c};
void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
void require(bool v,const char*s){if(!v)throw std::runtime_error(s);}
void init(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=1;clk();t.begin_swing=0;clk();clk();clk();}
void tick(){t.step=1;clk();t.step=0;int n=0;do{clk();if(++n>49000)throw std::runtime_error("deadline");}while(!t.done);for(int j=0;j<64;j++)clk();require(!t.fault,"fault");}
int main(){try{init();tick();t.theta=17694;t.filtered=70776;t.omega=-176947;
 int age=0;int prior_u=0;
 for(int j=0;j<250;j++){
  tick();auto*r=t.rootp;require(t.state==6,"BRAKE required");
  require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==0,"BRAKE may not blend");
  age=r->r01_control_harness__DOT__control__DOT__observer__DOT__age;
  if(j>0)require((int)t.duty==prior_u,"warmup changed committed fallback");
  prior_u=(int)r->r01_control_harness__DOT__control__DOT__swing_u;
 }
 require(age>=249,"BRAKE observer not warm");auto*r=t.rootp;int anchor=r->r01_control_harness__DOT__control__DOT__automatic_anchor;
 t.theta=655;t.filtered=2620;t.omega=0;
 for(int j=0;j<31;j++)tick();require(t.state==2,"CAPTURE required");
 require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==anchor,"origin moved at catch");
 require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age>250,"observer cleared at catch");
 require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend>0,"handoff not started after qualified warmup");
 for(int j=0;j<201;j++)tick();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==16777216,"blend not complete");
 t.stop=1;clk();require(t.duty==0,"STOP did not clear");for(int j=0;j<600;j++)clk();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"STOP did not clear warmup");
 // Leaving the local domain retracts all observer history before a new catch.
 init();tick();t.theta=17694;t.filtered=70776;t.omega=-176947;for(int j=0;j<240;j++)tick();
 t.theta=20000;t.filtered=80000;tick();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"out-of-domain warmup not cleared");
 t.theta=17694;t.filtered=70776;tick();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age<=1,"reentry inherited old warmup");
 // Nonzero origins in both directions: reference changes without rebasing the
 // observation states while BRAKE moves and confirmation freezes a later goal.
 for(int sign:{-1,1}){
  init();tick();const int base=sign*65536;int j=0;
  t.theta=sign*17694;t.filtered=sign*70776;t.omega=-sign*176947;t.velocity=sign*26214;
  for(;j<250;j++){
   t.phi=base+sign*j*26;tick();
   require(t.state==6,"moving BRAKE required");
   require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==base*4,"moving BRAKE changed anchor");
   require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__phi_s==((int)t.phi-base)*256,"moving BRAKE phi offset wrong");
   require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==((int)t.phi-base)*256,"moving BRAKE reference offset wrong");
   require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==0,"moving BRAKE mixed PWM");
  }
  t.theta=sign*655;t.filtered=sign*2620;t.omega=0;
  for(int k=0;k<40 && t.state!=2;k++,j++){t.phi=base+sign*j*26;tick();}
  require(t.state==2,"moving capture not confirmed");const int caught=(int)t.phi;
  require(caught!=base,"fixture did not move origin");
  require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==base*4,"moving catch rebased observer");
  require((int)r->r01_control_harness__DOT__control__DOT__fixed_reference==caught*4,"moving catch target not frozen current position");
  require((int)t.__SYM__reference==caught,"reported catch reference wrong");
  require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==(caught-base)*256,"caught target/observer origin offset wrong");
  require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age>250,"moving catch cleared warmup");
  for(int k=0;k<220;k++){
   t.phi=caught+sign*(k+1)*13;tick();
   require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==base*4,"holding changed observer anchor");
   require((int)r->r01_control_harness__DOT__control__DOT__fixed_reference==caught*4,"holding moved frozen goal");
   require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==(caught-base)*256,"holding target/observer offset changed");
   require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__phi_s==((int)t.phi-base)*256,"holding measurement rebased to goal");
  }
  require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==16777216,"moving handoff incomplete");
 }
 std::cout<<"PASS R25 BRAKE250 fallback commit unchanged; persistent anchor; capture warm handoff;200ms blend; STOP; domain withdrawal/reentry; two moving nonzero-origin BRAKE/catch/hold fixtures\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<"\n";return 1;}}
