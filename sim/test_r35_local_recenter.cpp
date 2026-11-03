// Actual controller/reference/guard RTL with prescribed observer completion.
// No plant or performance claim; tests the new reference transaction semantics.
#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define G(n) t.rootp->r01_control_harness__DOT__control__DOT__handoff_guard__DOT__##n
#define O(n) t.rootp->r01_control_harness__DOT__control__DOT__observer__DOT__##n
#define S(n) t.rootp->r01_control_harness__DOT__control__DOT__swing_core__DOT__##n
#define R(n) t.rootp->r01_control_harness__DOT__control__DOT__references__DOT__##n
static int checks=0,recenters=0,bypasses=0,cancels=0,fastbypass=0,slowadmit=0,recapture_rearm=0,used_cancel=0,quiet_accept=0,quiet_reject=0,quiet_scope=0,quiet_explicit=0,quiet_rearm=0,first_capture_quiet_bypass=0,real_recapture_permission=0,recovered_permission_clear=0,explicit_capture_bypass=0;
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void need(bool b,const char*m){++checks;if(!b)throw std::runtime_error(m);}
void init(int32_t xp=110000){
 t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.clear=0;t.begin_swing=0;t.begin_capture=0;
 t.raw_mode=0;t.fractional_mode=0;t.fractional_valid=1;t.adc_ready=1;t.calibrate=0;
 t.theta=0;t.omega=0;t.phi=xp>>2;t.velocity=0;t.filtered=0;t.measurement_ok=1;
 t.workspace_limit=217360;t.shuttle_amplitude=17157;t.motion_speed=0;t.shuttle_period=20;
 t.full_turn_recovery=1;t.target_valid=0;t.shuttle_valid=0;t.shuttle=0;t.target=0;
 ck();t.rst_n=1;ck();t.armed=1;ck();
 t.state=3;C(hold_ticks)=2000;C(swing_manual)=0;C(swing_mode)=2;
 C(phase)=0;C(observer_clear_q)=0;C(reference_clear_q)=0;C(swing_clear_q)=0;C(trip_code)=0;
 C(xt)=20000;C(xw)=30000;C(xp)=uint32_t(xp);C(xv)=0;C(swing_measurement_current)=1;
 C(automatic_goal)=3;C(automatic_home_valid)=1;C(saved_target)=0;
 C(pending_target)=0;C(pending_shuttle)=0;C(saved_shuttle)=0;
 C(tracking_initialized)=1;C(fixed_reference)=0;C(rp)=0;R(mode)=0;C(rv)=0;C(ref_busy)=0;
 C(automatic_observing)=1;C(automatic_anchor)=uint32_t(-22222);
 C(track_tick)=0;C(swing_tick)=0;C(ref_tick)=0;
 C(transaction_limit)=350;C(observer_fault)=0;C(swing_fault)=0;
 t.duty=100;C(reported_limit)=350;C(pending_command_valid)=0;
 ck();need(!C(r35_guard_completed_active)&&!C(r35_local_recenter_used),"history and once-per-catch clear");
}
void offer(bool newly_active=true){
 C(observer_clear_q)=0;
 C(xt)=20000;C(xw)=30000;
 C(phase)=4;C(observer_done)=0;C(observer_busy)=1;O(state)=322;
 O(fallback_s)=uint32_t(-100);O(rounded)=0;O(negative)=0;
 C(observer_u)=uint32_t(-100);C(swing_u)=230;C(observer_fault)=0;
 C(track_tick)=0;C(swing_tick)=0;t.step=0;
 if(newly_active){C(guard_active)=0;G(opposed_count)=19;}
 else{C(guard_active)=1;G(rescue_age)=100;G(release_count)=0;}
 ck();need(C(observer_done),"real observer completion");
 ck();need(t.handoff_recovery,"actual guard activated");
 need(!C(guard_done),"activation precedes completion");
 ck();need(C(guard_done),"matching real guard done");
}
void offer_quiet(int ticks=300,bool prescribed_recaptured=true){
 if(prescribed_recaptured)C(r35_recovered_catch)=1;
 C(observer_clear_q)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=1;
 O(state)=322;O(fallback_s)=100;O(rounded)=0;O(negative)=0;
 C(observer_u)=100;C(swing_u)=230;C(observer_fault)=0;
 C(track_tick)=0;C(swing_tick)=0;t.step=0;
 C(xt)=0;C(xw)=0;C(xv)=0;C(automatic_quiet_ticks)=ticks;
 C(guard_active)=0;G(opposed_count)=0;
 ck();need(C(observer_done),"quiet real observer completion");
 ck();need(!t.handoff_recovery,"quiet path never forces guard");
 ck();need(C(guard_done)&&!t.handoff_recovery,"quiet matching inactive guard done");
}
void retire(){ck();need(C(phase)==0&&t.done,"completed controller transaction");}
void seed_reference(int32_t seed){
 // Exercise the actual phase1 -> reference start -> reference done sequence.
 C(phase)=1;C(track_tick)=0;C(swing_tick)=0;
 ck();need(C(ref_reseed)&&C(ref_tick)&&C(phase)==3,"next phase1 reseeds");
 ck();need(C(ref_busy),"real reference busy");
 int n=0;while(!C(ref_done)&&n++<6000)ck();
 need(n<6000,"reference completion deadline");
 need(int32_t(C(rp))==seed&&int32_t(C(rv))==0,"actual rp reseeded");
 ck();need(C(tracking_initialized),"reference initialized only after done");
 need(int32_t(t.__SYM__reference)==(seed>>2)&&int32_t(t.ref_velocity)==0,"public reference agrees");
 C(swing_tick)=0;C(phase)=0;ck();
}
void check_recenter(int32_t xp,int32_t expected){
 init(xp);offer();need(!C(r35_guard_completed_active),"completion not consumed early");
 int32_t anchor=C(automatic_anchor);auto z0=O(z0);auto z1=O(z1);auto age=O(age);auto samples=O(samples);auto rescue_age=G(rescue_age);auto release_count=G(release_count);auto opposed_count=G(opposed_count);
 retire();++recenters;
 need(C(r35_guard_completed_active)&&C(r35_local_recenter_used),"completed-active and consumed catch history latched");
 need(int32_t(C(fixed_reference))==expected&&!C(tracking_initialized),"fixed seed and pending reseed");
 need(int32_t(C(saved_target))==expected&&int32_t(t.debug_home)==expected&&C(automatic_home_valid)&&C(automatic_goal)==3,"goal provenance synchronized");
 need(!C(pending_target)&&!C(pending_shuttle),"no old target queued");
 need(int32_t(C(automatic_anchor))==anchor&&O(z0)==z0&&O(z1)==z1&&O(age)==age&&O(samples)==samples,"observer anchor and state untouched");
 need(C(xt)==20000&&C(xw)==30000&&C(xv)==0,"angle velocity feedback untouched");
 need(t.handoff_recovery&&G(rescue_age)==rescue_age&&G(release_count)==release_count&&G(opposed_count)==opposed_count,"original rescue state retained");
 need(int32_t(t.__SYM__reference)==(expected>>2)&&!t.ref_velocity,"public seed immediate");
 seed_reference(expected);
 // Same rescue episode, later completed transaction at a new arm position.
 C(xp)=uint32_t(xp+12345);offer(false);retire();
 need(int32_t(C(fixed_reference))==expected&&C(tracking_initialized),"same rescue never reanchors twice");
 need(int32_t(C(rp))==expected,"rp retained across same rescue");
 // Completed inactive result records release; a later distinct activation may yield once.
 C(phase)=4;C(guard_active)=0;G(pending)=1;C(observer_done)=0;ck();need(C(guard_done),"inactive guard transaction");ck();
 need(!C(r35_guard_completed_active),"completed inactive clears history");
 C(xp)=uint32_t(xp);offer();retire();
 need(C(tracking_initialized)&&int32_t(C(fixed_reference))==expected&&C(r35_local_recenter_used),"same catch second rescue activation cannot reanchor again");
 // Actual pre-capture -> core done(mode2) -> CAPTURE transition rearms use.
 // An inactive completed guard during the new capture also consumes prior
 // episode activity, exactly as normal observer/capture transactions do.
 t.state=1;C(swing_mode)=0;C(phase)=0;C(track_tick)=0;ck();ck();
 need(!C(r35_local_recenter_used),"real pre-capture rearms used bit");
 C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;
 S(pending_limit)=350;S(pending_reference)=uint32_t(expected);ck();ck();
 need(t.state==2&&!C(r35_local_recenter_used),"actual new confirmed capture remains armed");
 C(track_tick)=0;C(swing_tick)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=0;
 C(guard_active)=0;G(pending)=1;ck();ck();
 need(!C(r35_guard_completed_active)&&!C(r35_local_recenter_used),"new capture inactive completion consumes old guard episode");
 t.state=3;C(hold_ticks)=2000;C(tracking_initialized)=1;R(mode)=0;C(ref_busy)=0;C(xp)=uint32_t(xp);
 offer();retire();
 need(!C(tracking_initialized)&&int32_t(C(fixed_reference))==expected&&C(r35_local_recenter_used),"new real catch may recenter once again");++recapture_rearm;
}
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);try{
 check_recenter(110000,110000);check_recenter(-110000,-110000);
 check_recenter(900000,823688);check_recenter(-900000,-823688);
 // Original capture-inside arm velocity bound: strict |xv|<1rad/s.
 // Include both observed bad-event speeds and the real B4 slow event.
 for(int32_t velocity:{-467141,-372507,-262145,-262144,262144,262145,372507,467141,-262143,262143,-148111}){
  init();C(xv)=uint32_t(velocity);offer();retire();
  bool allowed=velocity> -262144&&velocity<262144;
  need(bool(C(r35_local_recenter_used))==allowed,"capture speed domain eligibility");
  need(bool(C(tracking_initialized))!=allowed,"fast arms keep original target initialized");
  need(int32_t(C(fixed_reference))==(allowed?110000:0),"velocity gate controls local seed");
  if(allowed)++slowadmit;else ++fastbypass;
 }
 // New guard activation before maturity/CAPTURE must not be delayed until HOLD.
 init();t.state=2;C(hold_ticks)=1999;offer();retire();
 need(C(r35_guard_completed_active)&&C(fixed_reference)==0,"capture activation consumed but not recentered");
 t.state=3;C(hold_ticks)=2000;offer(false);retire();
 need(C(fixed_reference)==0&&C(tracking_initialized),"already-active capture guard cannot recenter later");
 // Cross the actual scope exclusions without altering guard logic.
 for(int test=0;test<13;test++){
  init();
  if(test==0)C(automatic_goal)=0;if(test==1)C(automatic_goal)=1;if(test==2)C(automatic_goal)=2;
  if(test==3)C(swing_manual)=1;if(test==4)t.state=2;if(test==5)t.state=4;
  if(test==6)C(hold_ticks)=1999;if(test==7)C(pending_target)=1;if(test==8)C(pending_shuttle)=1;
  if(test==9)R(mode)=1;if(test==10)R(mode)=2;if(test==11)C(tracking_initialized)=0;
  if(test==12)t.measurement_ok=0;
  if(test==3){C(observer_clear_q)=0;C(phase)=4;C(observer_done)=0;O(state)=322;C(observer_busy)=1;O(fallback_s)=100;O(rounded)=0;ck();ck();}
  else{offer();retire();}
  need(C(fixed_reference)==0,"excluded scope fixed reference unchanged");++bypasses;
 }
 // A same-retirement-cycle explicit command has priority over yielding home.
 for(int shuttle=0;shuttle<2;shuttle++){
  init();offer();if(shuttle){t.shuttle_valid=1;t.shuttle=1;}else{t.target_valid=1;t.target=1234;}
  retire();need(C(fixed_reference)==0&&C(tracking_initialized),"explicit command blocks recenter");
  need(C(automatic_goal)==(shuttle?2:1),"explicit goal priority");
  if(shuttle)need(C(pending_shuttle)&&C(saved_shuttle),"explicit shuttle preserved");
  else need(C(pending_target)&&int32_t(C(saved_target))==4936,"explicit target preserved");
 }
 // Established quiet-platform alternative: reuse the old300ms counter,
 // strict existing5degree position domain and original strict home bounds.
 struct QuietCase{int ticks;int32_t xp,rp;bool allowed;};
 for(QuietCase q: {QuietCase{299,110000,0,false},{300,0,0,false},
  {300,22876,0,false},{300,-22876,0,false},
  {300,22877,0,true},{300,-22877,0,true},
  {300,823688,0,false},{300,-823688,0,false},
  {300,823689,0,false},{300,-823689,0,false},
  {300,823687,0,true},{300,-823687,0,true},
  {300,467280,505296,true}}){
  init(q.xp);C(rp)=uint32_t(q.rp);C(fixed_reference)=uint32_t(q.rp);C(saved_target)=uint32_t(q.rp);
  offer_quiet(q.ticks);retire();
  need(bool(C(r35_local_recenter_used))==q.allowed,"quiet300/error/home scope");
  need(int32_t(C(fixed_reference))==(q.allowed?q.xp:q.rp),"quiet exact reference seed or unchanged");
  need(bool(C(tracking_initialized))!=q.allowed,"quiet only valid case requests real reseed");
  need(!t.handoff_recovery,"quiet yielding does not enable guard");
  if(!q.allowed){++quiet_reject;continue;}++quiet_accept;
  need(int32_t(C(saved_target))==q.xp&&int32_t(t.debug_home)==q.xp,"quiet shared goal synchronized");
  seed_reference(q.xp);
  int32_t moved=q.xp+(q.xp>0?-50000:50000);C(xp)=uint32_t(moved);
  offer_quiet();retire();need(C(tracking_initialized)&&int32_t(C(fixed_reference))==q.xp,"same catch second quiet yield rejected");
  offer();retire();need(C(tracking_initialized)&&int32_t(C(fixed_reference))==q.xp,"quiet consumes same permission as later guard");
  // True new capture transition, then a quiet-platform event may yield once.
  t.state=1;C(swing_mode)=0;C(phase)=0;C(track_tick)=0;ck();ck();need(!C(r35_local_recenter_used),"quiet used rearms only after pre-capture");
  C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;S(pending_reference)=uint32_t(q.xp);ck();ck();
  need(t.state==2&&!C(r35_local_recenter_used),"quiet actual new capture armed");
  C(track_tick)=0;C(swing_tick)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=0;C(guard_active)=0;G(pending)=1;ck();ck();
  t.state=3;C(hold_ticks)=2000;C(tracking_initialized)=1;R(mode)=0;C(ref_busy)=0;C(xp)=uint32_t(moved);C(rp)=uint32_t(q.xp);
  offer_quiet();retire();
  need(C(r35_local_recenter_used)&&!C(tracking_initialized)&&int32_t(C(fixed_reference))==moved,"new catch rearms quiet once");++quiet_rearm;
 }
 // Quiet must not bypass the common automatic/maturity/reference/command domains.
 for(int test=0;test<13;test++){
  init();
  if(test==0)C(automatic_goal)=0;if(test==1)C(automatic_goal)=1;if(test==2)C(automatic_goal)=2;
  if(test==3)C(swing_manual)=1;if(test==4)t.state=2;if(test==5)t.state=4;
  if(test==6)C(hold_ticks)=1999;if(test==7)C(pending_target)=1;if(test==8)C(pending_shuttle)=1;
  if(test==9)R(mode)=1;if(test==10)R(mode)=2;if(test==11)C(tracking_initialized)=0;
  if(test==12)t.measurement_ok=0;
  if(test==3){C(observer_clear_q)=0;C(phase)=4;C(observer_done)=0;O(state)=322;C(observer_busy)=1;O(fallback_s)=100;O(rounded)=0;C(automatic_quiet_ticks)=300;ck();ck();}
  else{offer_quiet();retire();}
  need(C(fixed_reference)==0&&!C(r35_local_recenter_used),"quiet common scope unchanged");++quiet_scope;
 }
 for(int shuttle=0;shuttle<2;shuttle++){
  init();offer_quiet();if(shuttle){t.shuttle_valid=1;t.shuttle=1;}else{t.target_valid=1;t.target=1234;}
  retire();need(!C(r35_local_recenter_used)&&C(fixed_reference)==0&&C(tracking_initialized),"same-cycle explicit command excludes quiet yield");
  need(C(automatic_goal)==(shuttle?2:1),"quiet explicit command retains goal priority");++quiet_explicit;
 }
 // The first actual capture cannot admit the quiet-platform alternative;
 // a real subsequent default-home capture can, without external labels.
 for(int direction:{-1,1}){
  init(direction*110000);C(r35_recovered_catch)=0;C(automatic_home_valid)=0;C(automatic_goal)=0;
  t.state=1;C(swing_mode)=0;C(phase)=0;C(track_tick)=0;ck();ck();
  C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;S(pending_reference)=0;ck();ck();
  need(t.state==2&&C(automatic_home_valid)&&C(automatic_goal)==3&&!C(r35_recovered_catch),"first real catch never grants recovered quiet permission");
  C(track_tick)=0;C(swing_tick)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=0;C(guard_active)=0;G(pending)=1;ck();ck();
  t.state=3;C(hold_ticks)=2000;C(tracking_initialized)=1;R(mode)=0;C(ref_busy)=0;C(xp)=uint32_t(direction*110000);C(rp)=0;
  offer_quiet(300,false);retire();
  need(!C(r35_recovered_catch)&&!C(r35_local_recenter_used)&&C(fixed_reference)==0&&C(tracking_initialized),"first mature quiet platform remains unchanged");++first_capture_quiet_bypass;
  t.state=1;C(swing_mode)=0;C(phase)=0;C(track_tick)=0;ck();ck();
  C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;S(pending_reference)=0;ck();ck();
  need(t.state==2&&C(r35_recovered_catch)&&!C(r35_local_recenter_used),"actual second default catch grants recovered quiet permission");
  C(track_tick)=0;C(swing_tick)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=0;C(guard_active)=0;G(pending)=1;ck();ck();
  t.state=3;C(hold_ticks)=2000;C(tracking_initialized)=1;R(mode)=0;C(ref_busy)=0;C(xp)=uint32_t(direction*110000);C(rp)=0;
  offer_quiet(300,false);retire();
  need(C(r35_recovered_catch)&&C(r35_local_recenter_used)&&!C(tracking_initialized)&&int32_t(C(fixed_reference))==direction*110000,"second-catch quiet yield actually admitted");
  seed_reference(direction*110000);++real_recapture_permission;
 }
 for(int goal:{1,2}){
  init();C(r35_recovered_catch)=0;C(automatic_goal)=goal;
  t.state=1;C(swing_mode)=0;C(phase)=0;C(track_tick)=0;ck();ck();
  C(phase)=2;C(swing_done)=0;S(phase)=8;S(pending_mode)=2;S(pending_limit)=350;S(pending_reference)=0;ck();ck();
  need(t.state==2&&!C(r35_recovered_catch),"explicit target or shuttle capture cannot grant default quiet history");++explicit_capture_bypass;
 }
 for(int how=0;how<5;how++){
  init();C(r35_recovered_catch)=1;
  if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.clear=1;
  if(how==3)C(swing_manual)=1;if(how==4)C(start_accepted)=1;
  ck();ck();need(!C(r35_recovered_catch),"session/manual/clear cancels real-recapture history");++recovered_permission_clear;
 }
 // Already-consumed catch permissions clear on all real session/capture
 // cancellation routes, not merely when the new event was still pending.
 for(int how=0;how<5;how++){
  init();offer();retire();need(C(r35_local_recenter_used),"used permission established");
  if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.clear=1;
  if(how==3)C(swing_manual)=1;if(how==4)C(start_accepted)=1;
  ck();ck();need(!C(r35_local_recenter_used),"session/manual/clear cancels consumed catch permission");++used_cancel;
 }
 // Cancellation at observer complete, guard active-before-done, guard done,
 // plus actual COAST cancels local seeding. No stale result changes reference.
 for(int stage=0;stage<4;stage++)for(int how=0;how<5;how++){
  init();C(observer_clear_q)=0;C(phase)=4;C(observer_done)=0;C(observer_busy)=1;O(state)=322;O(fallback_s)=uint32_t(-100);O(rounded)=0;
  C(swing_u)=230;C(guard_active)=0;G(opposed_count)=19;
  for(int j=0;j<stage;j++)ck();
  if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.rst_n=0;if(how==3)t.clear=1;
  if(how==4)t.state=7;
  ck();need(C(fixed_reference)==0,"cancellation wins over seed");
  need(!C(r35_guard_completed_active),"cancellation clears transaction history");
  for(int j=0;j<12;j++){ck();need(C(fixed_reference)==0,"no canceled stale recenter");}
  if(how<3)need(!t.duty,"STOP/disarm/reset PWM zero");++cancels;
 }
 std::cout<<"{\"status\":\"PASS\",\"assertions\":"<<checks<<",\"bounded_recenter_cases\":"<<recenters<<",\"scope_bypass_cases\":"<<bypasses<<",\"cancellation_cases\":"<<cancels<<",\"fast_velocity_bypass_cases\":"<<fastbypass<<",\"slow_velocity_admission_cases\":"<<slowadmit<<",\"new_capture_rearm_cases\":"<<recapture_rearm<<",\"used_permission_cancel_cases\":"<<used_cancel<<",\"quiet_platform_admission_cases\":"<<quiet_accept<<",\"quiet_platform_rejection_cases\":"<<quiet_reject<<",\"quiet_scope_bypass_cases\":"<<quiet_scope<<",\"quiet_same_cycle_explicit_cases\":"<<quiet_explicit<<",\"quiet_new_capture_rearm_cases\":"<<quiet_rearm<<",\"first_capture_quiet_bypass_cases\":"<<first_capture_quiet_bypass<<",\"actual_recapture_permission_cases\":"<<real_recapture_permission<<",\"recovered_permission_clear_cases\":"<<recovered_permission_clear<<",\"explicit_capture_history_bypass_cases\":"<<explicit_capture_bypass<<",\"new_history_bits\":3,\"plant_cases\":0}\n";
}catch(const std::exception&e){std::cerr<<e.what()<<" state="<<int(t.state)<<" phase="<<int(C(phase))<<" guard="<<int(t.handoff_recovery)<<" done="<<int(C(guard_done))<<" hist="<<int(C(r35_guard_completed_active))<<" init="<<int(C(tracking_initialized))<<" fixed="<<int32_t(C(fixed_reference))<<" rp="<<int32_t(C(rp))<<" goal="<<int(C(automatic_goal))<<" observer_clear="<<int(C(observer_clear_q))<<" observer_state="<<int(O(state))<<" swingmode="<<int(C(swing_mode))<<" xt="<<int32_t(C(xt))<<" trip="<<int(C(trip_code))<<"\n";return 1;}}
