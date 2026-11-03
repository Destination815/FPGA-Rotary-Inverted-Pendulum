#include "hold_observer.hpp"
#include "m03_plant.hpp"
#include "hybrid_fixed.hpp"
extern "C" void* est_new();extern "C" void est_free(void*);extern "C" void est_tick(void*,int,int,int,int,int*);
static double cl(double x,double a,double b){return std::clamp(x,a,b);}
static const double K=97763./(396*256);
extern "C" void simulate(const double*p,const double*cfg,int speed,int sign,int horizon,double*out,double*trace){
 void*est=est_new();int eo[9]={};HoldObserver h;h.cfg=cfg;
 hybrid_fixed::Machine cpu;cpu.mem[hybrid_fixed::mode]=2*hybrid_fixed::S;cpu.mem[hybrid_fixed::age]=2001*hybrid_fixed::S;cpu.mem[hybrid_fixed::ever]=hybrid_fixed::S;
 double x[7]={0,0,cfg[5]/K,0,cfg[5]/K,0,0},old=0,previous=0,reference=cfg[5]/K,velocity=0,start_ref=cfg[5]/K,target=cfg[5]/K;int age=-1,T=6000,segment=0;bool permission=false;int fault=0;
 int command_ticks[3]={5000,30000,57000};h.z[2]=llround(cfg[5]/K*h.Q);double targets[3]={sign*M_PI/2/K,-sign*M_PI/2/K,0};
 auto sense=[&](){double t=atan2(sin(x[0]),cos(x[0]))*p[16];double code=746+t/(386./65536);int fine=std::clamp((int)llround(code*16),0,16368);est_tick(est,fine>>4,fine,llround(x[2]/(396./65536)),1,eo);};
 for(int j=0;j<40;j++)sense();int j=0;
 for(;j<horizon;j++){if(cfg[7]>0&&j==30000)h.integral=llround(cfg[8]*sign*h.Q);if(cfg[7]>0&&j==57000)h.integral=llround(-cfg[9]*sign*h.Q);
  if(segment<3&&j==command_ticks[segment]){start_ref=reference;target=targets[segment++];double d=fabs(target-start_ref);int bins[]={2000,4000,6000,8000,12000,16000,20000,24000,32000,48000};for(int b:bins){T=b;double sec=T*.001;if(d*1.875/sec<=speed*M_PI/180/K && d*5.774/(sec*sec)<=M_PI/6/K)break;}age=0;permission=true;}
  velocity=0;if(age>=0){double q=cl(age/double(T),0,1),d=target-start_ref;reference=start_ref+d*(10*pow(q,3)-15*pow(q,4)+6*pow(q,5));velocity=d*(30*q*q-60*pow(q,3)+30*pow(q,4))/(T*.001);age++;if(age>T){reference=target;velocity=0;age=-1;}}
  sense();if(!eo[4]||std::abs(eo[0])>20000||fabs(x[2]*K)>M_PI*1.9){fault=1;break;}
  for(int q=0;q<4;q++)cpu.mem[hybrid_fixed::measurement_0+q]=int64_t(eo[q])*512;
  cpu.mem[hybrid_fixed::applied]=llround(previous*hybrid_fixed::S);cpu.mem[hybrid_fixed::observer_ready_input]=h.ready?hybrid_fixed::S:0;cpu.mem[hybrid_fixed::progress_measurement_valid]=hybrid_fixed::S;
  cpu.mem[hybrid_fixed::target_enable]=permission?hybrid_fixed::S:0;cpu.mem[hybrid_fixed::target_position]=llround(reference*hybrid_fixed::S);cpu.mem[hybrid_fixed::target_velocity]=llround(velocity*hybrid_fixed::S);cpu.run();
  int cap=llround(cpu.mem[hybrid_fixed::cap]*1000./hybrid_fixed::S);
  bool stationary=age<0;bool mature=j>=2000&&stationary;
  int u=h.policy_tick(eo[0],eo[2],llround(previous*1000),cap,llround(reference*65536),llround(velocity*65536),true,true,0,false,llround(old*1000),permission,mature,eo[1],true,stationary);
  if(h.fault){fault=2;break;}
  double*r=trace+16*j;r[0]=j*.001;r[1]=x[0]*180/M_PI;r[2]=x[2]*K*180/M_PI;r[3]=reference*K*180/M_PI;r[4]=velocity*K*180/M_PI;r[5]=u;r[6]=h.integral/double(h.Q);r[7]=h.motion_phase;r[8]=h.lp/double(h.Q);r[9]=h.z[1]*10./h.Q;r[10]=h.z[3]/double(h.Q);r[11]=eo[1]/65536.;r[12]=cap;
 #ifdef BASELINE
 r[13]=0;
 #else
 r[13]=h.r40_unwind_active;
 #endif
 r[14]=eo[0]*180/M_PI/65536;r[15]=eo[2]*K*180/M_PI/65536;
  m03_step(x,previous,p,.001,4);old=previous;previous=u*.001;
 }
 est_free(est);out[0]=fault;out[1]=j;
}
