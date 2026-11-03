#include "Vstate_estimator.h"
#include "Vstate_estimator___024root.h"
#include "verilated.h"
#include <cstdint>
double sc_time_stamp(){return 0.;}struct E{VerilatedContext c;Vstate_estimator t{&c};};
void clk(E*p){p->t.clk=0;p->t.eval();p->c.timeInc(1);p->t.clk=1;p->t.eval();p->c.timeInc(1);}
extern "C" {
void* make(int zero,int enc){auto*p=new E;p->t.rst_n=0;clk(p);p->t.rst_n=1;p->t.zero_q4=zero;p->t.zero_raw=zero>>4;p->t.encoder=enc;p->t.calibrate=1;clk(p);p->t.calibrate=0;clk(p);return p;}
void step(void*h,int raw,int fine,int enc,int gap,int*out){auto*p=(E*)h;auto&t=p->t;t.adc=raw;t.adc_q4=fine;t.adc_q4_valid=(fine>=96&&fine<=16272);t.adc_ready=1;t.encoder=enc;t.gap_allow=gap;t.tick=1;clk(p);t.tick=0;int n=0;while(!t.step_valid&&n++<1000)clk(p);auto*r=t.rootp;out[0]=t.theta;out[1]=t.dtheta;out[2]=t.phi;out[3]=t.dphi;out[4]=t.measurement_ok;out[5]=t.gap_active;out[6]=t.gap_ms;out[7]=r->state_estimator__DOT__gap_bridge__DOT__phase_n;out[8]=r->state_estimator__DOT__gap_bridge__DOT__unc;out[9]=r->state_estimator__DOT__gap_bridge__DOT__ok;clk(p);}
void release(void*h){delete(E*)h;}
}
