#define main existing_home_test_main
#include "test_r26_recovery_home.cpp"
#undef main
int main(){try{
 init(false,true);catch_at(0);mature();req(t.reference_ready,"initial shuttle board ready closed");t.shuttle=1;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();req(REG(pending_shuttle)&&!t.reference_ready,"queued shuttle became ready");tick();req(REG(automatic_goal)==2,"initial shuttle rejected");req(t.trajectory_running&&!t.reference_ready,"initial shuttle did not run or moving reference became ready");
 lose(-20);catch_at(-30);mature();req(REG(automatic_goal)==2&&REG(saved_shuttle),"recapture overrode shuttle goal");req(REG(saved_target)==0,"recapture did not request center");req(!t.trajectory_running,"shuttle restarted away from center");
 // Lose the rod again during the smooth center return. The same goal must be
 // requeued once, with a fresh temporary catch, without stale trajectory PWM.
 for(int i=0;i<100;i++){t.phi=t.__SYM__reference;t.velocity=t.ref_velocity;tick();}
 lose(-35);catch_at(-45);mature();req(REG(automatic_goal)==2&&REG(saved_target)==0&&!t.trajectory_running,"second recovery lost shuttle intent");
 bool resumed=false;for(int i=0;i<9000;i++){t.phi=t.__SYM__reference;t.velocity=t.ref_velocity;tick();if(t.trajectory_running){resumed=true;break;}}
 req(resumed&&std::abs((int)t.__SYM__reference)<q(5.1),"shuttle did not resume at center");
 // Stop-shuttle overrides future recovery resumption, including after another
 // subsequent loss. This preserves the existing explicit stop semantics.
 t.shuttle=0;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();tick();req(REG(automatic_goal)==0,"shuttle-off did not clear future resume");
 lose(-15);catch_at(-10);mature();req(REG(automatic_goal)==0&&!REG(pending_shuttle)&&!t.trajectory_running,"canceled shuttle resumed after loss");
 t.stop=1;clk();req(t.duty==0&&!REG(automatic_home_valid)&&REG(automatic_quiet_ticks)==0,"STOP retained recovery state");
 // A center return waiting for quiet shares the same explicit-target override.
 init(false,true);catch_at(0);mature();req(t.reference_ready,"shuttle override setup board ready closed");t.shuttle=1;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();tick();
 lose(-20);catch_at(-30);sense(4,-30);mature();
 req(REG(automatic_goal)==2&&REG(pending_target)&&REG(automatic_quiet_ticks)==0&&t.reference_ready,"unquiet center override setup not reachable");
 command_target(20);tick();req(REG(automatic_goal)==1&&REG(saved_target)==q(20)*4&&!REG(pending_target)&&!t.reference_ready&&REG(automatic_quiet_ticks)==0,"explicit target did not replace unquiet center return");
 t.stop=1;clk();req(t.duty==0&&REG(automatic_goal)==0&&!REG(pending_target)&&!REG(pending_shuttle),"STOP retained overridden center goal");
 // The same ready flag admits shuttle-on near center. Its explicit command
 // must retire the old home return so it cannot later collide with the shuttle.
 init(false,true);catch_at(100);mature();lose(110);catch_at(0);sense(4,0);mature();
 req(t.reference_ready&&REG(pending_target)&&REG(automatic_goal)==3&&REG(automatic_quiet_ticks)==0,"unquiet near-center home setup not reachable");
 t.shuttle=1;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();
 req(!REG(pending_target)&&REG(pending_shuttle)&&!t.reference_ready,"explicit shuttle-on retained queued home");
 tick();req(t.trajectory_running,"explicit shuttle-on did not start");sense(.1,0);for(int i=0;i<305;i++)tick();
 req(t.trajectory_running&&!REG(pending_target),"old home return survived into quiet shuttle motion");
 // Off must also cancel a center return that has not launched yet.
 lose(-20);catch_at(-30);sense(4,-30);mature();
 req(REG(pending_target)&&REG(automatic_goal)==2&&REG(automatic_quiet_ticks)==0,"unquiet shuttle-off setup not reachable");
 t.shuttle=0;t.shuttle_valid=1;clk();t.shuttle_valid=0;clk();tick();
 req(REG(automatic_goal)==0&&!REG(pending_target)&&!REG(pending_shuttle)&&!t.trajectory_running,"shuttle-off retained internal return");
 sense(.1,-30);for(int i=0;i<305;i++)tick();
 req(int(t.__SYM__reference)==q(-30)&&!t.trajectory_running&&t.reference_ready,"shuttle-off later resumed queued center move");
 std::cout<<"PASS shuttle0→loss→catch−30→return→loss→catch−45→center→resume→explicitoff→loss→catch−10→noresume; actual-ready target/home/center overrides; quiet-wait shuttle-on/off discard replaced internal return; transactions="<<transactions<<" maxcycles="<<maximum<<"\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" fault="<<int(t.fault)<<" goal="<<int(REG(automatic_goal))<<" ref="<<int(t.__SYM__reference)<<"\n";return 1;}}
