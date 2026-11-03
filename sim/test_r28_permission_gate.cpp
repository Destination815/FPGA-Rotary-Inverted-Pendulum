#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <stdexcept>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
#define C(n) t.rootp->r01_control_harness__DOT__control__DOT__##n
#define R(n) t.rootp->r01_control_harness__DOT__control__DOT__references__DOT__##n
void ck(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void req(bool b,const char*m){if(!b)throw std::runtime_error(m);}
void reset(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.clear=0;t.raw_mode=0;t.begin_swing=0;t.begin_capture=0;t.target_valid=0;t.shuttle_valid=0;t.measurement_ok=1;t.theta=0;t.omega=0;t.phi=0;t.velocity=0;t.filtered=0;t.workspace_limit=217360;ck();t.rst_n=1;ck();t.armed=1;}
int main(){try{int cases=0;for(int state=0;state<8;state++)for(int initialized=0;initialized<2;initialized++)for(int manual=0;manual<2;manual++)for(int goal=0;goal<4;goal++)for(int running=0;running<2;running++){
 reset();t.state=state;C(tracking_initialized)=initialized;C(swing_manual)=manual;C(automatic_goal)=goal;R(mode)=running?2:0;C(phase)=0;
 bool expected=!manual&&initialized&&(state==3||state==4)&&(goal==1||(goal==2&&running));
 t.step=1;ck();t.step=0;req(C(motion_assist_permission)==expected,"frame phase admission mismatch");
 // Admission remains a frame snapshot even if the goal changes before compute.
 C(automatic_goal)=0;ck();req(C(motion_assist_permission)==expected,"permission recomputed outside frame");
 t.stop=1;ck();req(!C(motion_assist_permission)&&t.duty==0,"STOP did not revoke permission");cases++;
 }
 std::cout<<"PASS complete_controller_permission_combinations="<<cases<<" frame_snapshot=1 stop_priority=1\n";
}catch(const std::exception&e){std::cerr<<e.what()<<" state="<<int(t.state)<<" phase="<<int(C(phase))<<" goal="<<int(C(automatic_goal))<<" permission="<<int(C(motion_assist_permission))<<"\n";return 1;}}
