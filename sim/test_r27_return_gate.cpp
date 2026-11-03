#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
#ifndef OBSERVER_WARM
#error observer schedule must be supplied from source
#endif
#ifndef OBSERVER_FULL
#error observer full-blend sample must be supplied from source
#endif
static VerilatedContext c;static Vr01_control_harness t{&c};
void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
void require(bool v,const char*s){if(!v)throw std::runtime_error(s);}
void init(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=1;clk();t.begin_swing=0;clk();clk();clk();}
void tick(){t.step=1;clk();t.step=0;int n=0;do{clk();if(++n>49000)throw std::runtime_error("deadline");}while(!t.done);for(int j=0;j<64;j++)clk();require(!t.fault,"fault");}
int main(){try{int changed=0,transactions=0;auto*r=t.rootp;
for(int sign:{-1,1}){
 init();tick();const int base=sign*65536;int prior_selected=0;
 // Enter BRAKE outside observer domain first; its entry origin differs from warm anchor.
 t.theta=sign*26214;t.filtered=sign*104856;t.omega=-sign*176947;t.velocity=sign*26214;t.phi=base-sign*6000;
 tick();transactions++;require(t.state==6,"outside-domain BRAKE entry failed");
 require(!r->r01_control_harness__DOT__control__DOT__automatic_observing,"observer warmed outside domain");
 require((int)r->r01_control_harness__DOT__control__DOT__swing_ref==(base-sign*6000)*4,"BRAKE entry origin fixture failed");
 t.theta=sign*17694;t.filtered=sign*70776;
 int expected_brake_phi=base;
 for(int j=0;j<1100;j++){
  t.phi=base+sign*j*13;
  if(r->r01_control_harness__DOT__control__DOT__observer__DOT__age<1000)expected_brake_phi=(int)t.phi;
  tick();transactions++;
  require(t.state==6,"BRAKE must remain unconfirmed");
  require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==base*4,"moving BRAKE changed anchor");
  require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==(expected_brake_phi-base)*256,"BRAKE mature reference contract changed");
  require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__phi_s==((int)t.phi-base)*256,"BRAKE observation coordinate mismatch");
  require((int)r->r01_control_harness__DOT__control__DOT__swing_ref==(base-sign*6000)*4,"legacy BRAKE fallback origin changed");
  int age=r->r01_control_harness__DOT__control__DOT__observer__DOT__age;
  int blended=r->r01_control_harness__DOT__control__DOT__observer_u;
  int fallback=r->r01_control_harness__DOT__control__DOT__swing_u;
  require((int)r->r01_control_harness__DOT__control__DOT__selected_u==blended,"BRAKE observer not selected");
  if(age<OBSERVER_WARM)require(blended==fallback,"observer used before source-defined warm-up");
  if(age>=OBSERVER_FULL)require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==16777216,"BRAKE mix did not reach full");
  changed+=blended!=fallback;
  if(j>0)require((int)t.duty==prior_selected,"queued PWM not previous selected value");
  prior_selected=blended;
 }
 require(changed>0,"fixture never exercised distinct observer output");
 t.theta=sign*655;t.filtered=sign*2620;t.omega=0;
 int count=0;for(;count<40&&t.state!=2;count++){t.phi+=sign*13;tick();transactions++;}
 require(t.state==2,"capture did not confirm");require(count==30,"30 consecutive joint samples changed");
 require((int)t.phi!=base,"capture position fixture is degenerate");
 require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==base*4,"capture rebased observer");
 const int capture_phi=(int)t.phi;
 require((int)r->r01_control_harness__DOT__control__DOT__fixed_reference==capture_phi*4,"capture did not freeze current target");
 require((int)t.__SYM__reference==capture_phi,"reported capture target mismatch");
 require((int)r->r01_control_harness__DOT__control__DOT__swing_ref==capture_phi*4,"fallback capture target differs");
 require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==(capture_phi-base)*256,"observer capture target discontinuity");
 require(r->r01_control_harness__DOT__control__DOT__observer__DOT__blend==16777216,"capture restarted blend");
 for(int j=0;j<100;j++){t.phi+=sign*13;tick();transactions++;require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==(capture_phi-base)*256,"confirmed target did not stay fixed");}
 // Actual loss passes through pump mode0 and clears history; direct2→1 is not a transition.
 t.theta=sign*58982;t.filtered=sign*235928;t.omega=0;tick();transactions++;
 require(t.state==1,"capture loss did not enter SWING");
 require(!r->r01_control_harness__DOT__control__DOT__automatic_observing,"loss retained observing state");
 require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"loss retained observer age");
 t.phi=base+sign*24000;t.theta=sign*17694;t.filtered=sign*70776;t.omega=-sign*176947;
 tick();transactions++;require(t.state==6,"reentry did not enter BRAKE");
 require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==(base+sign*24000)*4,"reentry reused old anchor");
 require((int)r->r01_control_harness__DOT__control__DOT__observer__DOT__reference_s==0,"reentry did not use current BRAKE position");
 require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age<=1,"reentry reused old maturity");

 // With the longer BRAKE fixture,24000 keeps recatch outside the existing return deadband.
 // Confirm the recatch, then exercise early INTERNAL return with unchanged external gate.
 t.theta=sign*655;t.filtered=sign*2620;t.omega=0;t.velocity=0;
 for(int j=0;j<40&&t.state!=2;j++)tick();
 require(t.state==2,"recatch confirmation");
 for(int j=0;j<1050;j++){
  t.velocity=j<600?sign*39322:0;
  tick();
  require(t.state==2,"external two-second capture state changed");
  require(!t.reference_ready,"external target admitted before two seconds");
  int ht=r->r01_control_harness__DOT__control__DOT__hold_ticks;
  bool ready=r->r01_control_harness__DOT__control__DOT__recovery_return_ready;
  if(ht<500 || j<899)require(!ready,"internal return before minimum age/continuous quiet");
 }
 require(r->r01_control_harness__DOT__control__DOT__recovery_return_ready,"quiet mature return not admitted");
 require(r->r01_control_harness__DOT__control__DOT__tracking_initialized,"reference not initialized early");
 require((int)t.__SYM__reference!=((int)t.phi),"internal return did not move reference");
 t.stop=1;clk();require(t.duty==0,"STOP failed immediate zero");for(int j=0;j<600;j++)clk();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"STOP retained warm state");require(!r->r01_control_harness__DOT__control__DOT__recovery_return_ready,"STOP retained internal return permission");
}
init();tick();t.theta=17694;t.filtered=70776;t.omega=-176947;for(int j=0;j<420;j++)tick();
t.theta=20000;t.filtered=80000;tick();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"domain exit retained warm state");
t.phi=7777;t.theta=17694;t.filtered=70776;tick();require((int)r->r01_control_harness__DOT__control__DOT__automatic_anchor==7777*4,"domain reentry did not reseed");require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age<=1,"domain reentry inherited maturity");
t.armed=0;clk();require(t.duty==0,"disarm failed immediate zero");for(int j=0;j<600;j++)clk();require(r->r01_control_harness__DOT__control__DOT__observer__DOT__age==0,"disarm retained warm state");
std::cout<<"PASS moving BRAKE/CAPTURE/hold transactions="<<transactions<<" changed_BRAKE_outputs="<<changed<<" two mirrored origins; 200 sample gate; 200ms blend; fixed BRAKE target and fixed capture goal/observer anchor; mode2 loss→0→1 reseed; unchanged30ms confirmation; phase-selected prediction PWM; STOP/disarm/domain clear\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<"\n";return 1;}}
