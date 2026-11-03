#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
static VerilatedContext c;static Vr01_control_harness t{&c};
static int transactions=0,maximum=0;
#define REG(x) t.rootp->r01_control_harness__DOT__control__DOT__##x
int q(double deg){return std::lround(deg*M_PI/180.*65536.);}
void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
void req(bool v,const char*s){if(!v)throw std::runtime_error(s);}
void tick(bool ok=true){t.step=1;clk();t.step=0;int n=0;do{clk();if(++n>49000)throw std::runtime_error("deadline");}while(!t.done);maximum=std::max(maximum,n);for(int j=0;j<64;j++)clk();transactions++;if(ok)req(t.fault==0,"unexpected controller fault");}
void sense(double theta,double phi,double omega=0,double v=0){t.theta=q(theta);t.filtered=q(theta)*4;t.phi=q(phi);t.omega=std::lround(omega*65536);t.velocity=std::lround(v*65536);}
void init(bool manual,bool full){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.target_valid=0;t.shuttle_valid=0;t.shuttle=0;t.target=0;t.measurement_ok=1;t.workspace_limit=q(190);t.shuttle_amplitude=q(15);t.motion_speed=1;t.shuttle_period=20;t.full_turn_recovery=full;sense(manual?0:179,0);clk();t.rst_n=1;clk();t.armed=1;t.begin_capture=manual;t.begin_swing=!manual;clk();t.begin_capture=0;t.begin_swing=0;clk();clk();tick();}
void catch_at(double phi){sense(15,phi,-2.7,.2);tick();req(t.state==6,"did not enter BRAKE");sense(.1,phi,0,0);for(int i=0;i<40&&t.state!=2;i++)tick();req(t.state==2,"did not confirm capture");}
void mature(){for(int i=0;i<2010;i++)tick();req(t.state==3||t.state==4,"capture gate did not mature");}
void lose(double phi){sense(55,phi);tick();req(t.state==1,"loss not SWING");}
void command_target(double phi){
 // Match opcode06 admission: never inject a request the real board would reject.
 req(t.armed&&t.reference_ready&&std::abs(q(phi))<=int(t.workspace_limit)-11438,"target blocked by actual board ready/bound gate");
 t.target=q(phi);t.target_valid=1;clk();t.target_valid=0;clk();
 req(REG(pending_target)&&REG(automatic_goal)==1&&!t.reference_ready,"ordinary queued user target became ready");
}
int main(){try{
 for(int sign:{-1,1}){
  init(false,false);sense(170,sign*181);tick(false);req(t.fault==1,"old automatic180deg bound absent");
  init(false,true);sense(170,sign*359);tick();req(t.state==1,"full-turn opt-in rejected359deg");sense(170,sign*360.1);tick(false);req(t.fault==1,"360deg recovery cap absent");
  init(true,true);sense(0,sign*191);tick(false);req(t.fault==1,"manual workspace cap bypassed");
 }
 init(false,true);catch_at(100);const int home=REG(saved_target);req(REG(automatic_home_valid)&&home==q(100)*4,"first capture home not saved");mature();
 lose(110);req(REG(automatic_home_valid)&&REG(saved_target)==home,"loss cleared home");catch_at(220);req(REG(saved_target)==home,"recapture overwrote home");req(REG(fixed_reference)==q(220)*4,"recapture target not current position");req(REG(automatic_goal)==3&&REG(saved_target)==home,"home return request lost");
 // At2s but with4deg measured rod angle, internal motion must wait while the
 // normal observer/hold loop keeps completing. One quiet sample cannot open it.
 sense(4,220);mature();req(REG(automatic_quiet_ticks)==0&&REG(pending_target)&&int(t.__SYM__reference)==q(220),"unquiet recovery moved home or stopped holding");
 req(t.reference_ready,"quiet-gated home return blocked explicit target readiness");
 sense(.1,220);for(int i=0;i<299;i++)tick();req(REG(pending_target)&&int(t.__SYM__reference)==q(220)&&t.reference_ready,"return launched or override blocked before300quiet samples");tick();req(!REG(pending_target)&&!t.reference_ready,"300quiet samples did not launch internal target with ready closed");
 req(REG(tracking_initialized)&&REG(automatic_goal)==3,"reference not seeded after recapture");
 // With the plant following measured reference exactly, exercise the real bounded
 // reference sequencer to completion. This is a scheduling test, not a plant claim.
 int first=q(220),prev=(int)t.__SYM__reference;bool moved=false;int maxstep=0,maxvelocity=0;
 for(int i=0;i<19000;i++){t.phi=t.__SYM__reference;t.velocity=t.ref_velocity;tick();int now=(int)t.__SYM__reference;maxstep=std::max(maxstep,std::abs(now-prev));maxvelocity=std::max(maxvelocity,std::abs((int)t.ref_velocity));moved|=now<first;prev=now;if(t.reference_ready&&std::abs(now-home/4)<=1)break;}
 req(moved&&t.reference_ready&&std::abs((int)t.__SYM__reference-home/4)<=1,"automatic smooth return did not reach first home");req(maxvelocity<=q(20),"reference velocity output violated20deg/s bound");req(maxstep<=q(.1),"home reference discontinuity above0.1deg");
 // A new loss queues another internal return. Before the quiet qualification,
 // use the actual board-visible ready flag to admit the explicit override.
 lose(110);catch_at(220);sense(4,220);mature();
 req(REG(pending_target)&&REG(automatic_goal)==3&&REG(automatic_quiet_ticks)==0&&t.reference_ready,"unquiet home override setup not reachable");
 command_target(80);tick();req(REG(automatic_goal)==1&&REG(saved_target)==q(80)*4&&!REG(pending_target),"user target did not replace home goal");
 req(!t.reference_ready&&REG(automatic_quiet_ticks)==0,"explicit target failed to launch before quiet or moving reference became ready");
 lose(110);catch_at(220);mature();req(REG(automatic_goal)==1&&REG(saved_target)==q(80)*4,"recapture overrode user target");
 t.stop=1;clk();req(t.duty==0&&!REG(automatic_home_valid)&&REG(automatic_goal)==0,"STOP retained command/home");t.stop=0;for(int i=0;i<100;i++)clk();req(t.duty==0,"stale commit after STOP");
 init(false,true);catch_at(0);mature();t.shuttle=1;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();tick();req(REG(automatic_goal)==2,"user shuttle goal lost");lose(-110);catch_at(-220);mature();req(REG(automatic_goal)==2&&REG(saved_shuttle),"recapture overrode shuttle goal");t.armed=0;clk();req(t.duty==0&&!REG(automatic_home_valid)&&REG(automatic_goal)==0&&REG(automatic_quiet_ticks)==0,"disarm retained home/quiet");
 // First catch outside normal target bounds must move toward bounded home.
 init(false,true);catch_at(220);req(REG(saved_target)==(q(190)-11438)*4&&REG(pending_target),"first out-of-workspace capture not bounded and queued");
 t.rst_n=0;clk();req(t.duty==0&&!REG(automatic_home_valid)&&REG(automatic_goal)==0,"reset retained home");
 std::cout<<"PASS recovery opt-in180/190/360 mirrored caps; persistent firsthome; recapture currenttarget; smooth bounded return; actual-ready-gated explicit override before quiet; queued user/moving reference blocked; usertarget/shuttle priority; STOP/disarm/reset. transactions="<<transactions<<" maxcycles="<<maximum<<" max_reference_step_q16="<<maxstep<<" max_reference_velocity_q16="<<maxvelocity<<" quiet_gate=300ms and explicit_user_bypass PASS\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<" home="<<int(REG(saved_target))<<" goal="<<int(REG(automatic_goal))<<" ref="<<int(t.__SYM__reference)<<"\n";return 1;}}
