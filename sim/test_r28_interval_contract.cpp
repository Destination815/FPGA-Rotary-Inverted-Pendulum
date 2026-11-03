#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
#include <cstdint>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define O(n) t.rootp->r01_control_harness__DOT__control__DOT__observer__DOT__##n
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void req(bool x,const char*s){if(!x)throw std::runtime_error(s);}
int snaps=0,changed=0,rawchecks=0,prior_checks=0,old_leads=0,causal=0,compat=0,causal_changed=0,compat_changed=0,rawstepmin=999999,rawstepmax=0;
void init(bool start_session=true){
 t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.fractional_mode=1;t.fractional_valid=1;t.adc_ready=1;t.adc=792;t.adc_q4=12661;t.zero=746;t.zero_q4=11936;t.encoder=0;t.calibrate=0;
 t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=217360;t.shuttle_amplitude=17157;t.shuttle_period=20;t.motion_speed=0;t.full_turn_recovery=1;t.target_valid=0;t.shuttle_valid=0;
 ck();t.rst_n=1;for(int k=0;k<40;k++){t.step=1;ck();t.step=0;for(int j=0;j<210;j++)ck();}
 if(start_session){t.armed=1;t.begin_swing=1;ck();t.begin_swing=0;for(int j=0;j<5;j++)ck();req(t.state==1,"start admission failed");}
}
void frame(bool verify){
 int interval=(int32_t)t.duty;bool use_interval=t.state==3||t.state==4;t.step=1;ck();t.step=0;int now=(int32_t)t.duty;
#ifdef NEW_INTERVAL
 if(verify){req((int32_t)C(interval_command)==interval,"frame snapshot did not use old committed command");req(C(prediction_use_interval)==use_interval,"frame snapshot phase selector mismatch");}
#endif
 bool snapshot=false,prior=false;int stepdelay=-1;
 for(int cycle=1;cycle<49000;cycle++){
  if(C(phase)==1&&stepdelay<0)stepdelay=cycle;
  if(C(observer_busy)&&O(state)==323&&!snapshot){snapshot=true;snaps++;changed+=interval!=now;
   req((int32_t)O(previous)==now,"slew previous is not current real committed output");
#ifdef NEW_INTERVAL
   req((int32_t)O(prediction_previous)==(use_interval?interval:now),"observer prediction not selected phase contract");
   if(use_interval){causal++;causal_changed+=interval!=now;}else{compat++;compat_changed+=interval!=now;}
#else
   if(interval!=now){req((int32_t)O(previous)!=interval,"baseline unexpectedly aligned");old_leads++;}
#endif
  }
  if(snapshot&&C(observer_busy)&&O(state)==1&&!prior){prior=true;prior_checks++;
#ifdef NEW_INTERVAL
   req((int32_t)O(prior_q)==(use_interval?interval:now)*16777,"prior_q did not consume phase-selected command");
#else
   req((int32_t)O(prior_q)==now*16777,"baseline prior_q not current command");
#endif
  }
  if(t.done){if(snapshot&&t.raw_mode){req(stepdelay>1,"raw ADC did not delay controller step after frame");rawchecks++;rawstepmin=std::min(rawstepmin,stepdelay);rawstepmax=std::max(rawstepmax,stepdelay);}for(int j=0;j<8;j++)ck();return;}
  ck();
 }
 throw std::runtime_error("deadline");
}
void prepare(){init();frame(false);t.theta=17694;t.filtered=70776;t.omega=-176947;t.velocity=26214;for(int j=0;j<6;j++)frame(true);req(t.state==6||t.state==2,"no local capture domain");}
int main(){try{
 prepare();for(int j=0;j<15;j++){t.phi=j*17;frame(true);}t.raw_mode=1;for(int j=0;j<15;j++){t.encoder=j/3;frame(true);}req(rawchecks>0,"raw mode never reached observer");req(changed>0,"fixture never changed published PWM");
 // Prescribed stable supervisor states isolate both HOLD/TRACK frame contracts.
 t.raw_mode=0;t.theta=0;t.filtered=0;t.omega=0;t.velocity=0;
 for(int j=0;j<24;j++){t.state=(j%2)?3:4;C(phase)=0;C(hold_ticks)=2000;t.phi=100+j*7;frame(true);}
 req(causal>0&&compat>0,"both prediction domains not covered");req(causal_changed>0&&compat_changed>0,"both domains need changed published PWM");
 // STOP exactly on publication frame wins over output, interval state and queue.
 t.stop=1;t.step=1;ck();t.step=0;req(t.duty==0,"STOP frame failed immediate zero");
#ifdef NEW_INTERVAL
 req(C(interval_command)==0&&!C(prediction_use_interval),"STOP retained prediction state");
#endif
 for(int j=0;j<700;j++){ck();req(t.duty==0,"STOP stale output");}req(!C(observer_busy)&&O(age)==0,"STOP retained observer");
 // An early frame while phase!=0 is rejected; no queued output may survive.
 prepare();t.step=1;ck();t.step=0;for(int j=0;j<20;j++)ck();req(C(phase)!=0,"overrun fixture idle");t.step=1;ck();t.step=0;req(t.duty==0,"frame collision not immediately zero");for(int j=0;j<700;j++)ck();req(t.fault==3&&!C(observer_busy)&&t.duty==0,"overrun did not cancel observer");
 // STOP on the observer snapshot cycle clears next edge and cannot publish it.
 prepare();t.step=1;ck();t.step=0;int wait=0;while(!(C(observer_busy)&&O(state)==323)&&wait++<49000)ck();req(wait<49000,"snapshot STOP fixture missing");t.stop=1;ck();req(t.duty==0,"snapshot STOP failed immediate zero");for(int j=0;j<700;j++)ck();req(!C(observer_busy)&&t.duty==0,"snapshot STOP leaked result");
 // Start admission colliding with frame publication is safe and has zero history.
 init(false);t.armed=1;t.begin_swing=1;t.step=1;ck();t.step=0;t.begin_swing=0;for(int j=0;j<8;j++)ck();req(t.state==1&&t.duty==0,"start on frame failed");
#ifdef NEW_INTERVAL
 req(C(interval_command)==0,"start on frame stale history");
#endif
 // New session after reset must begin at zero output/prediction history.
 init();req(t.duty==0,"restart nonzero output");
#ifdef NEW_INTERVAL
 req(C(interval_command)==0,"restart stale interval history");
#endif
 std::cout<<"PASS snapshots="<<snaps<<" distinct_old_new="<<changed<<" raw_adc_delayed_step="<<rawchecks<<" raw_step_delay_min="<<rawstepmin<<" raw_step_delay_max="<<rawstepmax<<" prior_q_checks="<<prior_checks<<" baseline_lead_observations="<<old_leads<<" causal_HOLD_TRACK="<<causal<<" compatibility_capture="<<compat<<" causal_changed="<<causal_changed<<" compatibility_changed="<<compat_changed<<" stop_on_frame=1 early_frame_abort=1 stop_on_observer_snapshot=1 reset_start_zero=1 start_on_frame=1\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<" phase="<<int(C(phase))<<" observer="<<int(O(state))<<"\n";return 1;}}
