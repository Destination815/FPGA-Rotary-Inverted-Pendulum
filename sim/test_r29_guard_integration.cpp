// Prescribed completed transactions on actual full controller RTL; no plant.
// Internal injection deliberately isolates scheduling, narrowing and cancellation.
#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;
static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define G(n) t.rootp->r01_control_harness__DOT__control__DOT__handoff_guard__DOT__##n
#define O(n) t.rootp->r01_control_harness__DOT__control__DOT__observer__DOT__##n
static int checks=0,transactions=0;
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void req(bool x,const char*why){if(!x)throw std::runtime_error(why);++checks;}
void init(){
 t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.clear=0;t.begin_swing=0;t.begin_capture=0;
 t.raw_mode=0;t.theta=0;t.omega=0;t.phi=0;t.velocity=0;t.filtered=0;
 t.measurement_ok=1;t.workspace_limit=217360;t.motion_speed=0;t.shuttle_period=20;
 t.full_turn_recovery=1;t.target_valid=0;t.shuttle_valid=0;t.calibrate=0;
 ck();t.rst_n=1;ck();t.armed=1;t.state=2;C(swing_manual)=0;C(swing_mode)=2;
 C(phase)=0;C(observer_clear_q)=0;C(xt)=20000;C(xw)=2000;C(xv)=0;
 C(swing_measurement_current)=1;C(transaction_limit)=350;ck();
}
void offer(int observer,int fallback,int mode=2,bool valid=true,int32_t theta=20000,int32_t omega=2000,int32_t velocity=0,bool manual=false){
 t.state=mode==1?6:2;C(phase)=4;C(swing_manual)=manual;C(swing_mode)=mode;
 C(xt)=uint32_t(theta);C(xw)=uint32_t(omega);C(xv)=uint32_t(velocity);
 C(swing_measurement_current)=valid;C(observer_clear_q)=0;
 C(observer_u)=uint32_t(observer);C(swing_u)=uint32_t(fallback);
 // Complete the observer through its real final state. Directly forcing the
 // done wire would bypass Verilator's settled combinational dependencies.
 C(observer_done)=0;C(observer_fault)=0;C(observer_busy)=1;O(state)=322;
 O(fallback_s)=uint32_t(observer);O(rounded)=0;O(negative)=0;
 C(track_tick)=0;C(swing_tick)=0;t.step=0;
 ck();req(C(observer_done),"prescribed observer completion missing");
}
void tx(int observer,int fallback,int expected,int mode=2,bool valid=true,int32_t theta=20000,int32_t omega=2000,int32_t velocity=0){
 offer(observer,fallback,mode,valid,theta,omega,velocity);
 ck();req(C(phase)==4&&!t.done&&!C(guard_done),"observer completion bypassed guard latency");
 ck();req(C(phase)==4&&C(guard_done)&&int16_t(C(guarded_u))==expected,"guard result did not complete at next edge");
 ck();req(C(phase)==0&&t.done&&C(pending_command_valid),"controller failed to accept guarded completion");
 req(int32_t(C(pending_command))==expected,"wrong signed guarded queue value");
 req(t.duty==0,"automatic result changed actuator before frame");
 ck();req(!t.done&&!C(guard_done)&&!C(guard_reblend),"stale done/reblend");
 ++transactions;
}
int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv);
 try{
  // BRAKE mode1 must complete through inactive guard without delay deadlock.
  init();for(int i=0;i<24;i++)tx(-123,230,-123,1);
  req(!t.handoff_recovery&&G(opposed_count)==0,"BRAKE activated guard");
  // Exactly20 same-direction opposed valid samples, then prediction continuity.
  for(int sign:{-1,1}){
   init();
   for(int i=1;i<=20;i++){
    tx(-sign*150,sign*230,i==20?sign*230:-sign*150,2,true,sign*20000,sign*2000);
    req(bool(t.handoff_recovery)==(i==20),"wrong 20 transaction boundary");
   }
   tx(100,240,240,2,false,0,0,0);
   req(t.handoff_recovery&&G(opposed_count)==0&&G(release_count)==0,"invalid must retain latch but clear counters");
   // Seed observer mature state to check that reblend changes only its weight.
   O(age)=800;O(samples)=200;O(integral)=321;O(integral_lp)=654;
   O(z0)=11;O(z1)=22;O(z2)=33;O(z3)=44;O(z4)=55;O(z5)=66;O(z6)=77;
   O(motion_latched)=1;O(motion_phase)=4;O(motion_ticks)=9;O(blend)=16777216;
   for(int i=1;i<=99;i++){
    tx(100,240,240,2,true,0,0,0);
    req(t.handoff_recovery&&G(release_count)==i,"early100 release");
   }
   offer(100,240,2,true,0,0,0);ck();
   req(!t.handoff_recovery&&!C(guard_reblend),"release pulse before completed transaction");
   ck();req(C(guard_done)&&C(guard_reblend)&&int16_t(C(guarded_u))==240,"release result must still select fallback");
   req(O(blend)==16777216,"observer consumed reblend before pulse");
   ck();req(t.done&&C(pending_command)==240&&O(blend)==0,"reblend failed at completion");
   req(O(age)==800&&O(samples)==200&&O(integral)==321&&O(integral_lp)==654,"reblend reset observer age/I");
   req(O(z0)==11&&O(z1)==22&&O(z2)==33&&O(z3)==44&&O(z4)==55&&O(z5)==66&&O(z6)==77,"reblend reset observer state");
   req(O(motion_latched)&&O(motion_phase)==4&&O(motion_ticks)==9,"reblend reset motion qualification");
   ck();req(!C(guard_reblend),"reblend pulse persisted");++transactions;
   tx(100,240,100,2,true,0,0,0);
  }
  // Domain exit clears latch but preserves transaction bypass behavior.
  init();for(int i=0;i<20;i++)tx(-150,230,i==19?230:-150);
  tx(-123,230,-123,2,true,73400,2000);req(!t.handoff_recovery,"domain equality did not deactivate");
  // Full32-bit values must be rejected before a low16 alias can pass.
  for(int field=0;field<2;field++)for(int value:{351,-351,65536,-65536,INT32_MIN,INT32_MAX}){
   init();offer(field==0?value:100,field==1?value:230);ck();
   req(t.state==5&&t.fault==7&&t.duty==0&&!G(pending),"full-width invalid result was narrowed/accepted");
   for(int i=0;i<5;i++)ck();
   req(!C(pending_command_valid)&&!C(guard_done)&&t.duty==0,"invalid result leaked later");
  }
  // Pending result, completed guard pulse, and queued publication cancellation.
  for(int delay=0;delay<4;delay++)for(int how=0;how<3;how++){
   init();offer(-150,230);
   for(int i=0;i<delay;i++)ck();
   if(how==0)t.stop=1;else if(how==1)t.armed=0;else t.rst_n=0;
   t.step=delay==3;ck();t.step=0;
   req(t.duty==0&&!C(pending_command_valid)&&!C(guard_done)&&!C(guard_reblend)&&!t.handoff_recovery,"cancel retained output/guard transaction");
   for(int i=0;i<20;i++){ck();req(t.duty==0&&!C(guard_done),"cancel leaked old result");}
  }
  // Manual phase4 consumes observer_done on its original edge, guard bypassed.
  init();offer(-123,230,2,true,0,0,0,true);ck();
  req(t.done&&C(phase)==0&&int32_t(t.duty)==-123&&!C(guard_done),"manual gained guard latency");
  // Automatic command remains queued until the raw frame edge.
  init();tx(-123,230,-123,1);t.step=1;ck();t.step=0;
  req(int32_t(t.duty)==-123,"frame failed to publish queued inactive-guard result");
  t.stop=1;ck();req(t.duty==0,"STOP failed after publication");
  std::cout<<"PASS checks="<<checks<<" prescribed_transactions="<<transactions
   <<" widths_fault_cases=12 stop_stages=12 manual_original_edge=1 reblend_only_weight=1"
   <<" no_plant_closed_loops=1\n";
 }catch(const std::exception&e){
  std::cerr<<"FAIL "<<e.what()<<" phase="<<int(C(phase))<<" state="<<int(t.state)
   <<" fault="<<int(t.fault)<<" done="<<int(t.done)<<" guard_done="<<int(C(guard_done))
   <<" pending="<<int(G(pending))<<" recovery="<<int(t.handoff_recovery)<<"\n";return 1;
 }
}
