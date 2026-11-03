#include "Vr01_tracking_core.h"
#include "verilated.h"
#include <cstdint>
double sc_time_stamp(){return 0;}
static void clock_tick(Vr01_tracking_core *d){d->clk=0;d->eval();d->clk=1;d->eval();}
extern "C" {
void *tracking_create(){auto*d=new Vr01_tracking_core;d->research_limit=150;d->rst_n=0;d->start=0;d->clear_state=0;clock_tick(d);d->rst_n=1;clock_tick(d);return d;}
void tracking_destroy(void*p){delete static_cast<Vr01_tracking_core*>(p);}
void tracking_clear(void*p){auto*d=static_cast<Vr01_tracking_core*>(p);d->start=0;d->clear_state=1;clock_tick(d);d->clear_state=0;}
void tracking_update(void*p,const int32_t*x,int32_t*y){
 auto*d=static_cast<Vr01_tracking_core*>(p);
 d->theta=x[0];d->omega=x[1];d->phi=x[2];d->vel=x[3];d->angle_filtered=x[4];d->ref_pos=x[5];d->ref_vel=x[6];d->ref_acc=x[7];d->ref_jerk=x[8];d->init_state=x[9];d->prior_duty_permille=x[10];
 d->start=1;clock_tick(d);d->start=0;int n=0;while(!d->done&&n<1000){clock_tick(d);n++;}
 y[0]=d->duty_permille;y[1]=d->dbg_lean;y[2]=d->dbg_accel;y[3]=d->dbg_vreq;y[4]=d->dbg_integral;y[5]=d->fault;y[6]=n;
 if(!d->done)y[5]=2;
 clock_tick(d);
}
}
