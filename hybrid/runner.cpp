#include <cmath>
#include <algorithm>
#include "nonlinear_native.cpp"
#include "m03_plant.hpp"
extern "C" void est_reacquire(void*); extern "C" void* est_new(); extern "C" void est_free(void*); extern "C" void est_tick(void*,int,int,int,int,int*);
static double cl(double v,double l,double h){return std::max(l,std::min(h,v));}
static double wr(double x){return atan2(sin(x),cos(x));}
#include "hybrid_fixed.hpp"
#include "hold_observer.hpp"
#include "r29_handoff_contract.hpp"
struct Hybrid {
 RescueHandoff rescue;EarlySaturation early;int previous_cap=0;double debug_mixed=0,debug_fallback=0;bool debug_selected=false;
 HoldObserver obs;double brake_reference=0;double fixed_origin=0;bool observing=false;int64_t blendq=0;int prediction_previous=0;
 hybrid_fixed::Machine cpu; const double*bank=nullptr; double minangle=4,origin=0,vreq=0,old=0;int mode=0,attempt=0,entries=0,restarts=0,control_fault=0;
 // R28 observer policy only; this wrapper retains its declared 1ms plant
 // command delay and is not a substitute for complete controller RTL tests.
 // This research wrapper has no full controller command state machine. Its
 // confirmed-mode age maps only the unchanged two-second CAPTURE-to-HOLD gate.
 // The actual controller snapshots its pre-frame state; hence age==2000 still
 // uses current PWM, and the following frame uses the prior physical interval.
 int observer_prediction_input()const{
  return mode==2 && cpu.mem[hybrid_fixed::age]>2000*hybrid_fixed::S
       ?prediction_previous:std::lround(old*1000);
 }
 int blend_output(int observed,int fallback,bool hold=true){
  int mixed=obs.mix(observed,fallback,true,hold,blendq);
  return (mode==1||mode==2)?mixed:fallback;
 }
 double tick(double t,double w,double f,double v,const double*c,int sign){
  using namespace hybrid_fixed;double x[]={t,w,f,v};for(int i=0;i<4;i++)cpu.mem[measurement_0+i]=llround(x[i]*S);
  cpu.mem[applied]=llround(old*S);cpu.mem[kick_sign]=sign*S;
  cpu.mem[observer_ready_input]=(c[2]>0?(obs.ready?S:0):S);cpu.run();control_fault=cpu.mem[hybrid_fixed::fault]/S;mode=cpu.mem[hybrid_fixed::mode]/S;attempt=cpu.mem[hybrid_fixed::attempt]/S;entries=cpu.mem[hybrid_fixed::entries]/S;restarts=cpu.mem[hybrid_fixed::restarts]/S;
  origin=double(cpu.mem[hybrid_fixed::origin])/S;vreq=double(cpu.mem[hybrid_fixed::vreq])/S;minangle=std::min(minangle,fabs(t));
  double u=std::nearbyint(double(cpu.mem[output])*1000/S)/1000;
  const int fallback=llround(u*1000);debug_fallback=.001*fallback;
  if(c[2]>0){
   if((mode==1 || mode==2) && fabs(t)<.28){
    if(!observing){obs.clear();blendq=0;fixed_origin=f;brake_reference=f;observing=true;}
    if(obs.age<1000)brake_reference=f;
    int capvalue=llround(double(cpu.mem[cap])*1000/S);int ou=obs.tick(llround(t*65536),llround((f-fixed_origin)*65536),llround(old*1000),capvalue,c[2]>1,llround(((mode==2?origin:brake_reference)-fixed_origin)*65536),0,-.5,true,mode==1,observer_prediction_input(),false,mode==2 && cpu.mem[hybrid_fixed::age]>2000*hybrid_fixed::S,llround(w*65536),cpu.mem[hybrid_fixed::progress_measurement_valid]!=0);
    int fallback=llround(u*1000);u=.001*blend_output(ou,fallback);
   }else {obs.clear();observing=false;blendq=0;}
  }
  const bool mature=mode==2 && cpu.mem[hybrid_fixed::age]>=2000*hybrid_fixed::S;
  const bool early_request=early.tick(mature,std::llround(old*1000),previous_cap);
  const bool early_trigger=early_request && early_returning_gate(std::llround(t*262144),std::llround(w*262144));
  previous_cap=std::llround(double(cpu.mem[hybrid_fixed::cap])*1000/hybrid_fixed::S);
  debug_mixed=u;
  int mixed=llround(u*1000);
  // Estimator inputs are Q16, exactly representable here; convert to the
  // guard's Q18 explicitly. The ISA memory above is Q25, not Q18.
  int selected=rescue.tick(mode,llround(t*262144),llround(w*262144),
      llround(v*262144),observing,
      cpu.mem[progress_measurement_valid]!=0,mixed,fallback,false,early_trigger,mature&&obs.ready);
  debug_selected=(selected!=mixed);
  if(selected!=mixed)u=.001*selected;
  if(rescue.released)blendq=0;
  prediction_previous=llround(old*1000);old=llround(u*1000)/1000.;return old;
 }
};
extern "C" void simulate(const double*bank,const double*p,int mass,const double*c,int sign,int horizon,int event,double*out,double*trace){
 void*e=est_new();int eo[9]={};double x[7]={sign*(M_PI-.025)+(mass>=2?p[15]:0)+c[67],0,0,0,0,0,0},old=0;Hybrid h;h.bank=bank;int fault=0,first=-1,eventtick=-1,lastcapture=-1;double ts=0,fs=0;int nt=0;
 int coast=0,good=0;bool prior_blind=false;int contact_age=999;double contact_rail=0;
 auto sense=[&](){double sensed=wr(x[0]);if(mass>=2){if(sensed>1.7/p[16])sensed-=2*M_PI;sensed*=p[16];}double code=fmod(746+sensed/(386./65536)+2*M_PI/(386./65536),2*M_PI/(386./65536));bool blind=code>1023;
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
 for(int k=0;k<40;k++)sense();const int measured_kick=eo[0]<0?-1:1;int j=0;
 for(;j<horizon;j++){
  // Event is a simulator fixture, never an input to the controller.
  if(event&&first>=0&&j==first+6000){x[1]+=(event>0?1.:-1.)*(std::abs(event)==3?8.:4.);eventtick=j;}
  if(std::abs(event)==2&&first>=0&&j==first+18000)x[1]+=(event>0?-4.:4.);
  sense();
  if((!eo[4] || (int(c[3])==1&&j==1760) || (int(c[3])==2&&j==1020) || (int(c[3])==3&&(j==1500||j==2240))) && h.mode!=2 && !coast){coast=1;good=0;est_reacquire(e);h.obs.clear();h.observing=false;h.blendq=0;h.rescue.reset();}
  if(coast){
   good=(eo[4]&&!eo[5]&&fabs(eo[1]/65536.)<30&&fabs(eo[3]/65536.)<12)?good+1:0;
   if(coast>=600){fault=2;break;}
   if(good<32){
    if(trace){double*r=trace+62*j;r[0]=j*.001;for(int wi=0;wi<4;wi++)r[54+wi]=h.cpu.mem[hybrid_fixed::weights_0+wi]/double(hybrid_fixed::S);for(int k=0;k<4;k++)r[1+k]=x[k];for(int k=0;k<4;k++)r[5+k]=eo[k]/65536.;r[9]=0;r[10]=7;r[11]=h.origin;r[12]=eo[5];r[13]=eo[6];r[14]=h.restarts;r[15]=h.vreq;r[43]=h.debug_mixed;r[44]=h.debug_fallback;r[45]=h.rescue.eligible;r[46]=h.debug_selected;r[47]=h.rescue.opposed_ticks;r[48]=h.rescue.latched;r[49]=h.rescue.stable_ticks;r[50]=h.rescue.released;r[51]=h.blendq;r[52]=h.rescue.activated;r[53]=h.cpu.mem[hybrid_fixed::cap]/double(hybrid_fixed::S);r[58]=h.cpu.mem[hybrid_fixed::phase_clock]/double(hybrid_fixed::S);r[59]=h.cpu.mem[hybrid_fixed::phase_active]/double(hybrid_fixed::S);r[60]=h.cpu.mem[hybrid_fixed::phase_knot]/double(hybrid_fixed::S);r[61]=h.cpu.mem[hybrid_fixed::phase_rate]/double(hybrid_fixed::S);}
    if(mass>=2)m03_step(x,0,p,.001,4);else if(mass)mass_step(x,0,p,.001,4);else effective_step(x,0,p,.001,4);coast++;old=0;h.old=0;h.prediction_previous=0;continue;}
   h.cpu.mem[hybrid_fixed::resume_elapsed]=coast*hybrid_fixed::S;coast=0;good=0;
  }
  if(!eo[4]){fault=2;break;}if(fabs(x[2])>(2*M_PI)){fault=1;break;}
  int before=h.mode;auto cycles_before=h.cpu.cycles;
#ifdef R29_PROGRESS_INPUT
  h.cpu.mem[hybrid_fixed::progress_measurement_valid]=eo[5]?0:hybrid_fixed::S;
#endif
  double u=h.tick(eo[0]/65536.,eo[1]/65536.,eo[2]/65536.,eo[3]/65536.,c,measured_kick);auto policy_cycles=h.cpu.cycles-cycles_before;
  if(h.control_fault){fault=h.control_fault;break;}
  if(before!=2&&h.mode==2){if(first<0)first=j;lastcapture=j;}
  if(h.mode!=2&&h.attempt>=12000){fault=5;break;}
  if(trace){double*r=trace+62*j;r[0]=j*.001;for(int wi=0;wi<4;wi++)r[54+wi]=h.cpu.mem[hybrid_fixed::weights_0+wi]/double(hybrid_fixed::S);for(int k=0;k<4;k++)r[1+k]=x[k];for(int k=0;k<4;k++)r[5+k]=eo[k]/65536.;r[9]=u;r[10]=h.mode;r[11]=h.origin;r[12]=eo[5];r[13]=eo[6];r[14]=h.restarts;r[15]=h.vreq;r[43]=h.debug_mixed;r[44]=h.debug_fallback;r[45]=h.rescue.eligible;r[46]=h.debug_selected;r[47]=h.rescue.opposed_ticks;r[48]=h.rescue.latched;r[49]=h.rescue.stable_ticks;r[50]=h.rescue.released;r[51]=h.blendq;r[52]=h.rescue.activated;r[53]=h.cpu.mem[hybrid_fixed::cap]/double(hybrid_fixed::S);r[58]=h.cpu.mem[hybrid_fixed::phase_clock]/double(hybrid_fixed::S);r[59]=h.cpu.mem[hybrid_fixed::phase_active]/double(hybrid_fixed::S);r[60]=h.cpu.mem[hybrid_fixed::phase_knot]/double(hybrid_fixed::S);r[61]=h.cpu.mem[hybrid_fixed::phase_rate]/double(hybrid_fixed::S);double pw=eo[1]/65536.;double et=double(h.cpu.mem[hybrid_fixed::energy_target])/hybrid_fixed::S;double th=eo[0]/65536.,ow=eo[1]/65536.;r[16]=pw;r[17]=ow*cos(th);r[18]=pw*cos(th);r[19]=.5*ow*ow/94.50762226949593+cos(th)-et;r[20]=double(h.cpu.mem[hybrid_fixed::energy])/hybrid_fixed::S;r[21]=double(h.cpu.mem[hybrid_fixed::desired])/hybrid_fixed::S;r[22]=double(h.cpu.mem[hybrid_fixed::pump])/hybrid_fixed::S;r[23]=policy_cycles;r[24]=h.cpu.mem[hybrid_fixed::envelope_latch]/double(hybrid_fixed::S);r[25]=h.cpu.mem[hybrid_fixed::envelope_phase]/double(hybrid_fixed::S);r[26]=h.cpu.mem[hybrid_fixed::envelope_vv]/double(hybrid_fixed::S);r[27]=h.cpu.mem[hybrid_fixed::envelope_original_vreq]/double(hybrid_fixed::S);r[28]=h.cpu.mem[hybrid_fixed::envelope_original_desired]/double(hybrid_fixed::S);r[29]=h.cpu.mem[hybrid_fixed::envelope_trigger]/double(hybrid_fixed::S);r[30]=h.cpu.mem[hybrid_fixed::envelope_release]/double(hybrid_fixed::S);r[31]=h.cpu.mem[hybrid_fixed::envelope_clipped]/double(hybrid_fixed::S);r[32]=h.cpu.mem[hybrid_fixed::progress_direction]/double(hybrid_fixed::S);r[33]=h.cpu.mem[hybrid_fixed::progress_turn_peak]/double(hybrid_fixed::S);r[34]=h.cpu.mem[hybrid_fixed::progress_previous_peak]/double(hybrid_fixed::S);r[35]=h.cpu.mem[hybrid_fixed::progress_half_valid]/double(hybrid_fixed::S);r[36]=h.cpu.mem[hybrid_fixed::progress_previous_valid]/double(hybrid_fixed::S);r[37]=h.cpu.mem[hybrid_fixed::progress_eligible]/double(hybrid_fixed::S);r[38]=h.cpu.mem[hybrid_fixed::progress_target]/double(hybrid_fixed::S);r[39]=h.cpu.mem[hybrid_fixed::progress_gain]/double(hybrid_fixed::S);r[40]=h.cpu.mem[hybrid_fixed::progress_event]/double(hybrid_fixed::S);r[41]=h.cpu.mem[hybrid_fixed::energy_target]/double(hybrid_fixed::S);r[42]=h.cpu.mem[hybrid_fixed::progress_measurement_valid]/double(hybrid_fixed::S);}
  if(j>=horizon-1000){ts+=pow(wr(x[0]),2);fs+=pow(x[2]-h.origin,2);nt++;}
  if(mass>=2)m03_step(x,old,p,.001,4);else if(mass)mass_step(x,old,p,.001,4);else effective_step(x,old,p,.001,4);old=u;
 }
 est_free(e);out[0]=fault;out[1]=h.mode;out[2]=first*.001;out[3]=j*.001;out[4]=nt?sqrt(ts/nt)*180/M_PI:999;out[5]=nt?sqrt(fs/nt)*180/M_PI:999;out[6]=h.entries;out[7]=h.restarts;out[8]=h.minangle*180/M_PI;out[9]=lastcapture*.001;out[10]=eventtick*.001;
}

