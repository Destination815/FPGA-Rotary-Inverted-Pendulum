#include "Vstate_estimator.h"
#include "verilated.h"
#include <cstdint>
double sc_time_stamp(){return 0.;}
struct E{VerilatedContext ctx;Vstate_estimator top{&ctx};};
static void clk(E*p){p->top.clk=0;p->top.eval();p->ctx.timeInc(1);p->top.clk=1;p->top.eval();p->ctx.timeInc(1);}
extern "C" {
void* est_new(){auto*p=new E;p->top.rst_n=0;clk(p);p->top.rst_n=1;clk(p);return p;}
void est_reacquire(void*ptr){auto*p=static_cast<E*>(ptr);p->top.restart_observation=1;clk(p);p->top.restart_observation=0;clk(p);}
void est_free(void*ptr){delete static_cast<E*>(ptr);}
void est_tick(void*ptr,int adc,int fine,int enc,int gap,int*out){auto*p=static_cast<E*>(ptr);auto&t=p->top;t.adc=adc;t.adc_q4=fine;t.zero_raw=746;t.zero_q4=11936;t.encoder=enc;t.adc_ready=1;t.adc_q4_valid=(fine>=96&&fine<=16272);t.gap_allow=gap;t.tick=1;clk(p);t.tick=0;int n=1;while(!t.step_valid&&n<500){clk(p);n++;}out[0]=t.theta;out[1]=t.dtheta;out[2]=t.phi;out[3]=t.dphi;out[4]=t.measurement_ok;out[5]=t.gap_active;out[6]=t.gap_ms;out[7]=n;out[8]=t.filtered_theta_q18;clk(p);}
}
