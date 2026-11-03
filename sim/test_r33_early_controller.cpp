// Actual complete controller wiring with prescribed raw boundaries/completions.
// No plant or threshold tuning; the nonlinear full-RTL checks are separate.
#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define E(n) t.rootp->r01_control_harness__DOT__control__DOT__early_saturation__DOT__##n
#define G(n) t.rootp->r01_control_harness__DOT__control__DOT__handoff_guard__DOT__##n
#define O(n) t.rootp->r01_control_harness__DOT__control__DOT__observer__DOT__##n
#define R(n) t.rootp->r01_control_harness__DOT__control__DOT__references__DOT__##n
#define S(n) t.rootp->r01_control_harness__DOT__control__DOT__swing_core__DOT__##n
static int checks=0,cancelcases=0,bypass=0,forces=0,skips=0,static_busy_clocks=0;
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void need(bool b,const char*m){++checks;if(!b)throw std::runtime_error(m);}
void init(){
 t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.clear=0;t.begin_swing=0;t.begin_capture=0;
 t.raw_mode=0;t.fractional_mode=0;t.fractional_valid=1;t.adc_ready=1;t.calibrate=0;
 t.theta=0;t.omega=0;t.phi=0;t.velocity=0;t.filtered=0;t.measurement_ok=1;
 t.workspace_limit=217360;t.shuttle_amplitude=17157;t.motion_speed=0;t.shuttle_period=20;
 t.full_turn_recovery=1;t.target_valid=0;t.shuttle_valid=0;t.shuttle=0;t.target=0;
 ck();t.rst_n=1;ck();t.armed=1;
 t.state=3;C(hold_ticks)=2000;C(swing_manual)=0;C(swing_mode)=2;
 C(phase)=0;C(observer_clear_q)=0;C(reference_clear_q)=0;C(swing_clear_q)=0;C(trip_code)=0;
 C(xt)=10013;C(xw)=uint32_t(-20025);C(xv)=0;C(swing_measurement_current)=1;
 C(track_tick)=0;C(swing_tick)=0;R(mode)=0;C(ref_busy)=0;
 t.duty=350;C(reported_limit)=350;C(pending_command)=350;
 C(pending_command_limit)=350;C(pending_command_valid)=1;ck();
 need(!E(saturated_count)&&!E(fired)&&!E(pending),"init early state");
}
void raw(int oldcommand=350,int oldcap=350,int nextcommand=350,int nextcap=350){
 C(phase)=0;t.duty=uint32_t(oldcommand);C(reported_limit)=oldcap;
 C(pending_command)=uint32_t(nextcommand);C(pending_command_limit)=nextcap;C(pending_command_valid)=1;
 t.step=1;ck();t.step=0;C(phase)=0;C(track_tick)=0;C(swing_tick)=0;ck();
}
void prime(){init();raw();need(E(saturated_count)==0,"first mature frame cannot count");
 for(int i=1;i<=99;i++){raw();need(E(saturated_count)==i&&!E(pending),"99 intervals");}}
void offer(int32_t theta,int32_t omega){
 t.state=3;C(phase)=4;C(swing_manual)=0;C(swing_mode)=2;C(hold_ticks)=2000;
 C(xt)=uint32_t(theta);C(xw)=uint32_t(omega);C(xv)=0;R(mode)=0;C(ref_busy)=0;
 C(swing_measurement_current)=1;C(observer_clear_q)=0;C(transaction_limit)=350;
 C(observer_u)=100;C(swing_u)=230;C(observer_done)=0;C(observer_fault)=0;
 C(observer_busy)=1;O(state)=322;O(fallback_s)=100;O(rounded)=0;O(negative)=0;
 C(track_tick)=0;C(swing_tick)=0;t.step=0;ck();
 need(C(observer_done),"prescribed real observer final edge");
}
void testforce(int32_t theta,int32_t omega,bool expected){
 prime();raw(350,350,-150,150);
 need(E(pending)&&E(fired)&&E(saturated_count)==100,"100th old interval");
 need(int32_t(t.duty)==-150&&int32_t(t.active_limit)==150,"same edge publication fixture");
 offer(theta,omega);ck();
 need(E(pending)&&E(inflight),"guard accept cannot retire pending");
 need(bool(t.handoff_recovery)==expected,"conditional force");
 if(expected){need(G(rescue_age)==1&&G(opposed_count)==0,"force first rescue1/opposed0");++forces;}else ++skips;
 ck();need(C(guard_done)&&E(pending),"done publication before counter retirement");
 need(int16_t(C(guarded_u))==(expected?230:100),"force versus old observer output");
 ck();need(!E(pending)&&!E(inflight)&&E(fired),"retire exactly matching done");
 need(C(pending_command_valid)&&int32_t(C(pending_command))==(expected?230:100),"selected output queued");
 need(int32_t(t.duty)==-150,"automatic compute did not publish early");
 t.stop=1;ck();need(!t.duty&&!E(pending)&&!E(fired)&&!C(pending_command_valid),"STOP immediate cancel");
}
int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv);
 try{
  // A real static reference computation must not erase a raw-frame interval
  // run merely because stationary includes !busy during internal arithmetic.
  init();raw();for(int i=0;i<50;i++)raw();need(E(saturated_count)==50,"pre-busy run50");
  C(ref_tick)=1;C(ref_reseed)=0;ck();need(C(ref_busy)&&!C(ref_stationary),"actual static reference entered busy");
  do{ck();need(E(saturated_count)==50&&E(prior_mature),"static reference busy preserved run");
     if(++static_busy_clocks>5000)throw std::runtime_error("reference deadline");}while(C(ref_busy));
  raw();need(E(saturated_count)==51,"after actual busy continue51");
  // raw CAPTURE1999 is the frame whose unchanged step increments to2000.
  init();t.state=2;C(hold_ticks)=1998;ck();raw();
  need(C(hold_ticks)==1999&&!E(prior_mature),"raw CAPTURE1998 not mature");
  raw();need(C(hold_ticks)==2000&&E(prior_mature)&&!E(saturated_count),"raw CAPTURE1999 will-mature count0");
  C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;ck();ck();
  need(t.state==3,"original phase2 enters HOLD at2000");C(phase)=0;C(track_tick)=0;ck();
  raw();need(E(saturated_count)==1,"next raw native mature interval count1");
  for(int sign:{-1,1}){testforce(sign*10013,-sign*20025,true);testforce(sign*10013,-sign*20026,false);testforce(sign*10013,-sign*20027,false);}
  testforce(0,0,false);testforce(10013,0,true);testforce(-10013,20025,true);
  // Old unsaturatedcap350 cannot be relabelled saturated by same-edge cap150.
  init();raw();for(int i=1;i<=98;i++)raw();
  raw(350,350,150,350);need(E(saturated_count)==99&&int32_t(t.duty)==150,"publish unsaturated interval");
  raw(150,350,150,150);need(!E(pending)&&E(saturated_count)==0,"old reported cap attribution");
  // Out-of-domain policy completion consumes pending but never queues force
  // for the following observer-domain reentry.
  prime();raw();t.theta=18350;t.filtered=73400;raw();
  need(C(xt)==73400,"real sensor snapshot for domain boundary");
  C(phase)=2;C(swing_mode)=2;C(swing_done)=0;C(swing_fault)=0;
  S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;ck();
  need(C(swing_done),"real swing completion for discard");ck();
  need(E(fired)&&!E(pending)&&!E(inflight),"phase2 outside-domain consumes attempt");
  C(phase)=0;C(swing_done)=0;t.theta=0;t.filtered=0;ck();
  for(int i=0;i<120;i++)raw();need(E(fired)&&!E(pending),"no late domain-reentry force");
  // Every controller state/manual/reference mode/qualification boundary.
  for(int state=0;state<8;state++)for(int manual=0;manual<2;manual++)
   for(int refmode=0;refmode<3;refmode++)for(int age:{1999,2000}){
    init();t.state=state;C(swing_manual)=manual;C(hold_ticks)=age;
    R(mode)=refmode;C(ref_busy)=0;C(phase)=0;ck();
    E(saturated_count)=99;E(prior_mature)=1;E(fired)=0;
    bool eligible=((state==3&&age>=2000)||(state==2&&age==1999))&&!manual&&refmode==0;
    raw();need(bool(E(fired))==eligible&&bool(E(pending))==eligible,"actual mature permission combinations");
    if(!eligible){need(!E(saturated_count)&&!E(prior_mature),"inactive resets new state only");++bypass;}
   }
  prime();raw();t.target_valid=1;ck();need(!E(pending)&&!E(fired),"new target immediately cancels early attempt");
  need(C(pending_target)&&C(automatic_goal)==1,"original target admission preserved");t.target_valid=0;
  // Cancellation at pending, observer done, guard accept, guard done, queued
  // result and the next publish opportunity. No canceled work may return.
  for(int stage=0;stage<6;stage++)for(int how=0;how<3;how++){
   prime();raw();if(stage>0)offer(10013,-20025);
   for(int i=1;i<stage;i++)ck();
   if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.rst_n=0;
   t.step=stage==5;ck();t.step=0;
   need(!t.duty&&!E(pending)&&!E(inflight)&&!E(fired)&&!C(guard_done)&&!C(guard_reblend),"new force cancel priority");
   for(int i=0;i<80;i++){ck();need(!t.duty&&!C(guard_done)&&!C(guard_reblend),"new force stale completion");}
   ++cancelcases;
  }
  std::cout<<"{\"status\":\"PASS\",\"assertions\":"<<checks<<",\"force_cases\":"<<forces
   <<",\"skip_cases\":"<<skips<<",\"permission_bypass_cases\":"<<bypass<<",\"cancel_stage_cases\":"<<cancelcases
   <<",\"static_reference_busy_clocks\":"<<static_busy_clocks<<",\"will_mature_raw_hold_ticks\":1999"
   <<",\"mature_alignment_to_native\":\"RAW_BOUNDARY_PROVED_FULL_PLANT_SEPARATE\",\"plant_cases\":0}\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<" state="<<int(t.state)<<" phase="<<int(C(phase))<<" count="<<int(E(saturated_count))<<" fired="<<int(E(fired))<<" pending="<<int(E(pending))<<" mature="<<int(C(early_mature))<<" mode="<<int(C(swing_mode))<<" hold="<<int(C(hold_ticks))<<" stationary="<<int(C(ref_stationary))<<" refbusy="<<int(C(ref_busy))<<" refmode="<<int(R(mode))<<" prior="<<int(E(prior_mature))<<" trip="<<int(C(trip_code))<<"\n";return 1;}
}
