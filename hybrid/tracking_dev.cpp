#include <cmath>
#include <algorithm>
#include "nonlinear_native.cpp"
extern "C" void* est_new(); extern "C" void est_free(void*); extern "C" void est_tick(void*,int,int,int,int,int*);
static double cl(double v,double l,double h){return std::max(l,std::min(h,v));}
static double wr(double x){return atan2(sin(x),cos(x));}
#include "hybrid_fixed.hpp"
#include "hold_observer.hpp"
struct Hybrid {
 HoldObserver obs;double fixed_origin=0;bool observing=false;
 hybrid_fixed::Machine cpu; const double*bank=nullptr; double minangle=4,origin=0,vreq=0,old=0;int mode=0,attempt=0,entries=0,restarts=0,control_fault=0;
 double tick(double t,double w,double f,double v,const double*c,int sign){
  using namespace hybrid_fixed;double x[]={t,w,f,v};for(int i=0;i<4;i++)cpu.mem[measurement_0+i]=llround(x[i]*S);
  cpu.mem[applied]=llround(old*S);cpu.mem[kick_sign]=sign*S;
  cpu.run();control_fault=cpu.mem[hybrid_fixed::fault]/S;mode=cpu.mem[hybrid_fixed::mode]/S;attempt=cpu.mem[hybrid_fixed::attempt]/S;entries=cpu.mem[hybrid_fixed::entries]/S;restarts=cpu.mem[hybrid_fixed::restarts]/S;
  origin=double(cpu.mem[hybrid_fixed::origin])/S;vreq=double(cpu.mem[hybrid_fixed::vreq])/S;minangle=std::min(minangle,fabs(t));
  double u=std::nearbyint(double(cpu.mem[output])*1000/S)/1000;
  if(c[2]>0){
   if(mode==2 && fabs(t)<.28){
    if(!observing){obs.clear();fixed_origin=f;observing=true;}
    int capvalue=llround(double(cpu.mem[cap])*1000/S);int ou=obs.tick(llround(t*65536),llround((f-fixed_origin)*65536),llround(old*1000),capvalue,c[2]>1);
    double blend=obs.samples>200?1.:0.;u=std::nearbyint((u*(1-blend)+ou*.001*blend)*1000)/1000;
   }else {obs.clear();observing=false;}
  }
  old=u;return old;
 }
};
extern "C" void simulate(const double*bank,const double*p,int mass,const double*c,int sign,int horizon,int event,double*out,double*trace){
 void*e=est_new();int eo[9]={};double x[4]={sign*(M_PI-.025),0,0,0},old=0;Hybrid h;h.bank=bank;int fault=0,first=-1,eventtick=-1,lastcapture=-1;double ts=0,fs=0;int nt=0;
 bool prior_blind=false;int contact_age=999;double contact_rail=0;
 auto sense=[&](){double code=fmod(746+wr(x[0])/(386./65536)+2*M_PI/(386./65536),2*M_PI/(386./65536));bool blind=code>1023;
  if(prior_blind&&!blind){contact_age=0;contact_rail=code>512?1022:0;}
  prior_blind=blind;if(blind)code=0;
  // Sensor-only stress: one plausible edge read followed by 8/12ms rail bounce.
  // c[0] selects a declared fixture; the controller sees no fixture or plant truth.
  int bounce=c[0]>1?12:c[0]>0?8:0;
  if(bounce&&contact_age==0&&!blind)code=cl(code+(contact_rail>512?-32:0),0,1023);
  else if(bounce&&contact_age<=bounce)code=contact_rail;
  // Additional declared observation fixture: delayed release of a blind rail.
  // c[1] is a sensor-model setting, never given to the controller.
  int release_delay=std::max(0,std::min(32,(int)std::llround(c[1])));
  if(!blind && contact_age<release_delay)code=contact_rail;
  contact_age++;int fine=std::max(0,std::min(16368,(int)round(code*16)));est_tick(e,fine>>4,fine,round(x[2]/(396./65536)),1,eo);};
 for(int k=0;k<40;k++)sense();int j=0;
 for(;j<horizon;j++){
  // Event is a simulator fixture, never an input to the controller.
  if(event&&first>=0&&j==first+6000){x[1]+=(event>0?4.:-4.);eventtick=j;}
  if(std::abs(event)==2&&first>=0&&j==first+18000)x[1]+=(event>0?-4.:4.);
  sense();if(!eo[4]){fault=2;break;}if(fabs(x[2])>M_PI){fault=1;break;}
  int before=h.mode;double u=h.tick(eo[0]/65536.,eo[1]/65536.,eo[2]/65536.,eo[3]/65536.,c,sign);
  if(h.control_fault){fault=h.control_fault;break;}
  if(before!=2&&h.mode==2){if(first<0)first=j;lastcapture=j;}
  if(h.mode!=2&&h.attempt>=12000){fault=5;break;}
  if(trace){double*r=trace+16*j;r[0]=j*.001;for(int k=0;k<4;k++)r[1+k]=x[k];for(int k=0;k<4;k++)r[5+k]=eo[k]/65536.;r[9]=u;r[10]=h.mode;r[11]=h.origin;r[12]=eo[5];r[13]=eo[6];r[14]=h.restarts;r[15]=h.vreq;}
  if(j>=horizon-1000){ts+=pow(wr(x[0]),2);fs+=pow(x[2]-h.origin,2);nt++;}
  if(mass)mass_step(x,old,p,.001,4);else effective_step(x,old,p,.001,4);old=u;
 }
 est_free(e);out[0]=fault;out[1]=h.mode;out[2]=first*.001;out[3]=j*.001;out[4]=nt?sqrt(ts/nt)*180/M_PI:999;out[5]=nt?sqrt(fs/nt)*180/M_PI:999;out[6]=h.entries;out[7]=h.restarts;out[8]=h.minangle*180/M_PI;out[9]=lastcapture*.001;out[10]=eventtick*.001;
}

extern "C" void tracking(const double*p,int mass,int useA,double ff,int kind,double*out,double*trace){
 void*e=est_new();int eo[9]={};double x[4]={0,0,0,0},old=0;HoldObserver obs;double lead=0;int fault=0;double ts=0,fs=0,peak=0;int nt=0;
 auto sense=[&](){double code=746+wr(x[0])/(386./65536);int fine=std::clamp((int)llround(code*16),0,16368);est_tick(e,fine>>4,fine,llround(x[2]/(396./65536)),1,eo);};
 for(int k=0;k<40;k++)sense();int j=0;
 for(;j<24000;j++){
  double r=0,rv=0,t=j*.001;
  if(kind==0){double q=cl((t-2)/6.,0.,1.);r=M_PI/2*(10*q*q*q-15*q*q*q*q+6*q*q*q*q*q);rv=(q>0&&q<1)?M_PI/2*(30*q*q-60*q*q*q+30*q*q*q*q)/6:0;}
  else if(t>2){double w=2*M_PI/12;r=.5235987756*(1-cos(w*(t-2)));rv=.5235987756*w*sin(w*(t-2));}
  sense();if(!eo[4]||fabs(wr(x[0]))>.5||fabs(x[2])>M_PI){fault=1;break;}
  int cap=((fabs(eo[0]/65536.)>.02618&&eo[0]*double(eo[1])>0)||j<200)?350:150;
  double governed=r;
  int u=obs.tick(eo[0],eo[2],llround(old*1000),cap,useA,llround(governed*65536),llround(rv*65536),ff);
  if(useA==2){double want=6.996196749162687*eo[0]/65536.+.6847930331950819*eo[1]/65536.+.3162277660168408*(eo[2]/65536.-r)+.48012438370034877*eo[3]/65536.+ff*rv;u=llround(cl(cl(want,-cap*.001,cap*.001),old-.02,old+.02)*1000);}
  if(j>12000){ts+=x[0]*x[0];fs+=(x[2]-r)*(x[2]-r);nt++;}peak=std::max(peak,fabs(wr(x[0])));
  if(trace){double*z=trace+j*8;z[0]=t;for(int i=0;i<4;i++)z[1+i]=x[i];z[5]=r;z[6]=u*.001;z[7]=rv;}
  if(mass)mass_step(x,old,p,.001,4);else effective_step(x,old,p,.001,4);old=u*.001;
 }
 est_free(e);out[0]=fault;out[1]=j*.001;out[2]=nt?sqrt(ts/nt)*180/M_PI:999;out[3]=nt?sqrt(fs/nt)*180/M_PI:999;out[4]=peak*180/M_PI;
}
