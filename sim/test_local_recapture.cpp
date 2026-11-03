// Actual full controller RTL with prescribed completed swing-core transactions.
// Tests target bookkeeping only; no plant and no claim of swing performance.
#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define S(n) t.rootp->r01_control_harness__DOT__control__DOT__swing_core__DOT__##n
static int checks=0;
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void need(bool b,const char*m){++checks;if(!b)throw std::runtime_error(m);}
void init(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.clear=0;t.begin_swing=0;t.begin_capture=0;t.raw_mode=0;t.fractional_mode=0;t.fractional_valid=1;t.adc_ready=1;t.calibrate=0;t.theta=0;t.omega=0;t.phi=0;t.velocity=0;t.filtered=0;t.measurement_ok=1;t.workspace_limit=217360;t.shuttle_amplitude=17157;t.motion_speed=0;t.shuttle_period=20;t.full_turn_recovery=1;t.target_valid=0;t.shuttle_valid=0;t.shuttle=0;t.target=0;ck();t.rst_n=1;ck();t.armed=1;t.state=6;C(swing_manual)=0;C(phase)=0;C(trip_code)=0;C(swing_clear_q)=0;C(reference_clear_q)=0;C(observer_clear_q)=0;C(swing_mode)=1;C(xt)=100;C(xw)=0;C(xv)=0;C(swing_measurement_current)=1;C(automatic_home_valid)=1;C(saved_target)=123456;C(pending_target)=1;C(pending_shuttle)=0;ck();}
void capture(int goal,int32_t p,bool first=false,bool stop=false){init();C(automatic_goal)=goal;C(automatic_home_valid)=!first;C(phase)=2;C(swing_done)=0;C(swing_fault)=0;C(swing_tick)=0;C(track_tick)=0;C(hold_ticks)=600;S(phase)=8;S(pending_mode)=2;S(pending_reference)=uint32_t(p);S(pending_limit)=350;t.stop=stop;ck();need(C(swing_done)||stop,"swing completion missing");ck();}
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);bool baseline=argc>1;try{
 for(int32_t p:{-600000,-300000,-1,0,1,300000,600000}){
  capture(3,p);need(t.state==2,"new capture state altered");need(C(hold_ticks)==0,"capture maturity altered");
  need(C(automatic_goal)==3,"implicit local goal identity changed");need(int32_t(C(saved_target))==(baseline?123456:p),"local recapture target wrong");need(bool(C(pending_target))==baseline,"unsolicited return not canceled");
  for(int g:{1,2}){capture(g,p);need(C(automatic_goal)==g,"explicit goal identity lost");need(C(saved_target)==123456,"explicit target overwritten");need(C(pending_target),"explicit pending target canceled");}
 }
 for(int32_t p:{-900000,900000}){capture(3,p);need(C(pending_target),"workspace correction canceled");need(int32_t(C(saved_target))==(baseline?123456:(p<0?-823688:823688)),"bounded target wrong");}
 for(int32_t p:{-300000,300000}){capture(0,p,true);need(C(automatic_home_valid),"first global home not preserved");need(C(automatic_goal)==3&&int32_t(C(saved_target))==p,"first local target wrong");need(!C(pending_target),"first in-bounds catch returned");}
 capture(3,400000,false,true);need(t.state==0&&t.duty==0&&!C(pending_target)&&!C(automatic_home_valid),"STOP priority changed");
 std::cout<<"{\"status\":\"PASS\",\"checks\":"<<checks<<",\"baseline\":"<<(baseline?"true":"false")<<",\"plant_cases\":0}\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" state="<<int(t.state)<<" phase="<<int(C(phase))<<" goal="<<int(C(automatic_goal))<<" saved="<<int32_t(C(saved_target))<<" pending="<<int(C(pending_target))<<" ref="<<int32_t(C(swing_ref))<<"\n";return 1;}}
