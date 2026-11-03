#include "Vr01_control_harness.h"
#include "verilated.h"
#include <cstdint>
double sc_time_stamp(){return 0.;}
struct Run { VerilatedContext ctx; Vr01_control_harness top{&ctx}; int maxcycles=0; int changes=0; int change_cycle[8]={};int change_value[8]={}; int sample_cycle=0;int last_value=0;bool tracing=false; };
static void clock(Run*r){r->top.clk=0;r->top.eval();r->ctx.timeInc(1);r->top.clk=1;r->top.eval();r->ctx.timeInc(1);
 if(r->tracing){int value=(int32_t)r->top.duty;if(value!=r->last_value){if(r->changes<8){r->change_cycle[r->changes]=r->sample_cycle;r->change_value[r->changes]=value;}r->changes++;r->last_value=value;}r->sample_cycle++;}
}
extern "C" {
void* r01_new(){auto*r=new Run;r->top.full_turn_recovery=0;r->top.workspace_limit=68629;r->top.shuttle_amplitude=17157;r->top.motion_speed=0;r->top.motion_speed_limit=0;r->top.shuttle_period=20;r->top.rst_n=0;clock(r);r->top.rst_n=1;clock(r);return r;}
int r29_guard_status(void*p){return static_cast<Run*>(p)->top.handoff_recovery;}
void r41t2_sine(void*p,int32_t amplitude,uint32_t phase_step){auto&r=static_cast<Run*>(p)->top;r.sine_amplitude=amplitude;r.sine_step=phase_step;}
int r40_ext2_status(void*p){return static_cast<Run*>(p)->top.ext2_active;}
void r40_ext2(void*p,int enabled){auto*r=static_cast<Run*>(p);r->top.ext2_switch=1;r->top.ext2_value=enabled!=0;r->top.shuttle=enabled!=0;r->top.shuttle_valid=1;clock(r);r->top.ext2_switch=0;r->top.shuttle_valid=0;clock(r);}
int r10_status(void*p){auto&r=static_cast<Run*>(p)->top;return r.recovery_boosting | (r.recovery_used<<1) | (r.settled<<2) | (r.settle_timeout<<3);}
void r14_config(void*p,int32_t range,int32_t amp,int speed,int period){auto&r=static_cast<Run*>(p)->top;r.workspace_limit=range;r.shuttle_amplitude=amp;r.motion_speed=speed;r.motion_speed_limit=0;r.shuttle_period=period;}
void r36_numeric_speed(void*p,int32_t internal_q16){static_cast<Run*>(p)->top.motion_speed_limit=internal_q16;}
// Read-only harness pin readback; no controller or output mutation.
void r33_config_readback(void*p,int32_t*out){auto&r=static_cast<Run*>(p)->top;out[0]=r.workspace_limit;out[1]=r.shuttle_amplitude;out[2]=r.motion_speed;out[3]=r.shuttle_period;out[4]=r.full_turn_recovery;}
void r26_recovery_config(void*p,int enabled){static_cast<Run*>(p)->top.full_turn_recovery=enabled!=0;}
void r01_delete(void*p){delete static_cast<Run*>(p);}
void r01_input(void*p,int32_t t,int32_t w,int32_t f,int32_t v,int32_t filt,int valid){auto&r=static_cast<Run*>(p)->top;r.theta=t;r.omega=w;r.phi=f;r.velocity=v;r.filtered=filt;r.measurement_ok=valid;}
void r01_raw_input(void*p,int32_t adc,int32_t enc,int32_t zero,int valid){auto&r=static_cast<Run*>(p)->top;r.raw_mode=1;r.fractional_mode=0;r.adc=adc;r.encoder=enc;r.zero=zero;r.adc_ready=valid;}
void r01_fraction_input(void*p,int32_t adc,int32_t adc_q4,int32_t enc,int32_t zero_q4,int valid,int fractional_valid){auto&r=static_cast<Run*>(p)->top;r.raw_mode=1;r.fractional_mode=1;r.adc=adc;r.adc_q4=adc_q4;r.encoder=enc;r.zero=zero_q4>>4;r.zero_q4=zero_q4;r.adc_ready=valid;r.fractional_valid=fractional_valid;}
void r01_calibrate(void*p){auto*r=static_cast<Run*>(p);r->top.calibrate=1;clock(r);r->top.calibrate=0;clock(r);}
void r01_begin(void*p,int manual){auto*r=static_cast<Run*>(p);r->top.armed=1;r->top.begin_capture=manual;r->top.begin_swing=!manual;clock(r);r->top.begin_capture=0;r->top.begin_swing=0;clock(r);clock(r);}
void r01_stop(void*p){auto*r=static_cast<Run*>(p);r->top.stop=1;clock(r);r->top.stop=0;r->top.armed=0;clock(r);}
void r01_command(void*p,int code,int32_t value){auto*r=static_cast<Run*>(p);r->top.target=value;r->top.shuttle=value!=0;r->top.target_valid=code==6;r->top.shuttle_valid=code==7;clock(r);r->top.target_valid=0;r->top.shuttle_valid=0;clock(r);}
int r01_tick(void*p,int32_t*out){auto*r=static_cast<Run*>(p);r->changes=0;r->sample_cycle=0;r->last_value=(int32_t)r->top.duty;r->tracing=true;r->top.step=1;clock(r);r->top.step=0;int n=1;while(!r->top.done&&n<49000){clock(r);n++;}if(n>r->maxcycles)r->maxcycles=n;
 out[0]=r->top.duty;out[1]=r->top.state;out[2]=r->top.fault;out[3]=r->top.__SYM__reference;out[4]=r->top.ref_velocity;out[5]=r->top.reference_ready;out[6]=r->top.trajectory_running;out[7]=n;out[8]=r->top.active_limit;out[9]=r->top.measured_theta;out[10]=r->top.measured_omega;out[11]=r->top.measured_phi;out[12]=r->top.measured_velocity;out[13]=r->top.measured_filtered;out[14]=r->top.gap_active;out[15]=r->top.gap_ms;out[16]=r->top.measurement_valid;
 clock(r);r->tracing=false;return n<49000;}
int r20_commit_info(void*p,int32_t*out){auto*r=static_cast<Run*>(p);for(int i=0;i<r->changes&&i<8;i++){out[i*2]=r->change_cycle[i];out[i*2+1]=r->change_value[i];}return r->changes;}
int r20_idle(void*p,int n){auto*r=static_cast<Run*>(p);int before=(int32_t)r->top.duty;for(int i=0;i<n;i++){clock(r);if((int32_t)r->top.duty!=before)return i+1;}return 0;}

}
