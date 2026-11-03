#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
void need(bool b,const char*s){if(!b)throw std::runtime_error(s);}
void clk(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
int tick(){t.step=1;clk();t.step=0;int c=0;while(!t.done&&!t.fault){clk();need(++c<49000,"scheduler overrun");}for(int j=0;j<64;j++)clk();return c;}
void reset(){t.ext2_switch=0;t.ext2_value=0;t.target_valid=0;t.shuttle_valid=0;t.shuttle=0;t.target=0;t.full_turn_recovery=1;t.shuttle_amplitude=68640;t.motion_speed=3;t.motion_speed_limit=71165;t.shuttle_period=20;t.fractional_mode=0;t.fractional_valid=0;t.adc_ready=0;t.calibrate=0;t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=411840;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=1;clk();t.begin_swing=0;clk();clk();clk();tick();t.theta=0;t.filtered=0;for(int j=0;j<2300;j++)tick();need(!t.fault&&t.reference_ready,"initial capture fixture");}
void start_shuttle(){t.shuttle=1;t.shuttle_valid=1;clk();t.shuttle_valid=0;for(int j=0;j<1200;j++){t.phi=C(reference);t.velocity=t.ref_velocity;tick();need(!t.fault,"ordinary shuttle fixture");}need(t.trajectory_running&&C(automatic_goal)==2,"ordinary shuttle active");}
void inject_guard(){need(!t.handoff_recovery,"baseline guard already active");C(handoff_guard__DOT__guard_active)=1;clk();}
int main(){try{
reset();start_shuttle();int local=t.phi;inject_guard();tick();
need(C(r37_deferred_target)&&C(pending_target),"rescue must defer target");need(C(saved_target)==0&&C(saved_shuttle)&&C(automatic_goal)==2,"retained shuttle and zero return");
for(int j=0;j<3;j++){t.phi=local;t.velocity=0;tick();}
need(!t.trajectory_running,"ordinary sine must freeze on rescue");need(C(tracking_initialized),"reference reseeded");need(C(r41_rescue_ticks)>0,"rescue failure timer resumed after one reseed");
int released=-1,returned=-1,restarted=-1;bool seen_move=false;
for(int j=0;j<14000;j++){
 t.theta=0;t.filtered=0;t.omega=0;t.velocity=0;
 if(!t.handoff_recovery)t.phi=C(reference);else t.phi=local;
 tick();need(!t.fault,"recovery return failed");
 if(!t.handoff_recovery&&released<0)released=j;
 if(released>=0&&!t.reference_ready)seen_move=true;
 if(seen_move&&t.reference_ready&&returned<0)returned=j;
 if(returned>=0&&t.trajectory_running){restarted=j;break;}
}
need(released>=0&&returned>=0&&restarted>returned,"release -> zero move -> quiet -> restart missing");need(C(saved_shuttle)&&C(automatic_goal)==2,"shuttle request lost");
std::cout<<"PASS ordinary shuttle rescue local="<<local<<" release="<<released<<" return="<<returned<<" restart="<<restarted<<"\n";
// Persistent rescue: inject a latched guard, then keep non-quiet but captured measurements.
reset();start_shuttle();local=t.phi;inject_guard();tick();int reset_count=0,prev=0,fault_tick=-1;
for(int j=0;j<1300;j++){
 t.phi=local;t.velocity=30000;t.theta=8000;t.filtered=32000;t.omega=50000;
 tick();int n=C(r41_rescue_ticks);if(n<prev&&!t.fault)reset_count++;prev=n;
 if(t.fault){fault_tick=j;break;}
}
need(t.fault==10&&t.duty==0,"persistent guard must fault10 and cut output");need(reset_count<=1&&fault_tick<=1010,"recenter repeatedly reset failure timer");
std::cout<<"PASS persistent rescue fault_tick="<<fault_tick<<" timer_resets="<<reset_count<<"\n";
t.stop=1;clk();need(t.duty==0,"STOP after fault");std::cout<<"PASS checks=12 actual_RTL=true physical_plant=false guard_injection=white_box\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<(int)t.state<<" fault="<<(int)t.fault<<" duty="<<(int32_t)t.duty<<" guard="<<(int)t.handoff_recovery<<" initialized="<<(int)C(tracking_initialized)<<" deferred="<<(int)C(r37_deferred_target)<<" pending="<<(int)C(pending_target)<<" phase="<<(int)C(phase)<<"\n";return 1;}}
