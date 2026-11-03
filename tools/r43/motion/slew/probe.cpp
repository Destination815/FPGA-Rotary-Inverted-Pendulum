#include "hold_observer.hpp"
#include "m03_plant.hpp"
#include "plant.hpp"
extern "C" void* est_new();
extern "C" void est_free(void*);
extern "C" void est_tick(void*,int,int,int,int,int*);
static constexpr double K=97763./101376.;
struct Budget {
 int warm=200,stable=0,elapsed=0,cap=150;bool boost=false;
 int tick(int theta,int omega,bool valid){
  if(!valid){cap=150;boost=false;stable=0;elapsed=0;return cap;}
  if(warm<200){warm++;return cap=150;}
  int64_t projected=theta+(int64_t(omega)*5040>>16);
  bool outward=((theta<0)==(omega<0))&&(omega>9830||omega<-9830);
  bool risk=outward&&(projected>5022||projected<-5022);
  bool recovered=abs(theta)<2621&&abs(omega)<19661,oldboost=boost;
  if(boost){if(recovered){if(stable>=49){boost=false;stable=0;}else stable++;}else stable=0;elapsed=std::min(255,elapsed+1);}
  else if(risk){boost=true;elapsed=0;stable=0;}
  if(oldboost||(!oldboost&&risk))cap=cap<340?cap+10:350;else cap=cap>152?cap-2:150;
  return cap;
 }
};
static void a7_advance(double*x,const double E[9][9],int pwm,double friction){
 double u=pwm/1000.,eff=u>=0?std::max(0.,u-QPAR[9]):std::min(0.,u+QPAR[9])*QPAR[8],n[7]={};
 for(int i=0;i<7;i++){for(int j=0;j<7;j++)n[i]+=E[i][j]*x[j];n[i]+=E[i][7]*eff+E[i][8]*friction;}
 std::copy(n,n+7,x);
}
// cfg: kind(0 M03,1 A7),sensor_scale,theta_init,phi_error_deg,omega_init,
// integral_init,motion_permission,reference_deg,capture_origin_deg,noise_deg,bias_deg.
extern "C" void simulate(const double*p,const double*cfg,int N,double*out,double*trace){
 void*e=est_new();int eo[9]={};Budget budget;HoldObserver h;
 double x[7]={cfg[2],cfg[4],(cfg[3]+cfg[7])/K*M_PI/180,0,(cfg[3]+cfg[7])/K*M_PI/180,0,0};
 if(cfg[0]==1){x[4]=0;x[5]=0;}
 double reference=cfg[7]/K*M_PI/180,origin=cfg[8]/K*M_PI/180;
 h.z[0]=llround(x[0]*h.Q);h.z[1]=llround(x[1]/10*h.Q);h.z[2]=llround((x[2]-origin)*h.Q);
 h.age=1000;h.samples=200;h.ready=1;h.blend=h.Q;h.integral=llround(cfg[5]*h.Q);
 h.motion_latched=bool(cfg[6]);h.motion_phase=cfg[6]?3:0;h.drift_latched=bool(cfg[6]);
 h.r43_variant=int(cfg[11]);x[3]=cfg[12]/K*M_PI/180;if(cfg[0]==0)x[5]=x[3];
 int previous=0,old=0,j=0;double sensed=0;
 auto sense=[&](int step){
  if(cfg[0]==1){sensed=SENSOR[0]*x[0]+cfg[1]*(SENSOR[1]*x[1]+SENSOR[3]*x[3]+SENSOR[4]*x[4]);}
  else sensed=p[16]*x[0]+cfg[1]*(SENSOR[1]*x[1]+SENSOR[3]*x[3]+SENSOR[4]*QPAR[0]/p[6]*x[6]);
  sensed+=cfg[10]*M_PI/180+cfg[9]*M_PI/180*(sin(step*1.31)+sin(step*.73))/2;
  int fine=std::clamp(int(llround((746+sensed/(386./65536))*16)),0,16368);
  est_tick(e,fine>>4,fine,llround(x[2]/(396./65536)),1,eo);
 };
 for(int k=0;k<40;k++)sense(-1);
 for(;j<N;j++){
  if(j==4000)x[1]+=cfg[13];
  sense(j);if(!eo[4]||fabs(sensed)>.3||fabs(x[2]*K)>M_PI*2){out[0]=1;break;}
  int cap=budget.tick(eo[8]>>2,eo[1],eo[4]);
  int u=h.policy_tick(eo[0],eo[2]-llround(origin*65536),previous,cap,llround((reference-origin)*65536),0,true,true,0,false,old,bool(cfg[6]),true,eo[1],true,true);
  if(h.fault){out[0]=2;break;}
  double*r=trace+14*j;r[0]=j*.001;r[1]=x[0]*180/M_PI;r[2]=x[2]*K*180/M_PI;r[3]=sensed*180/M_PI;r[4]=eo[0]*180/M_PI/65536.;r[5]=u;r[6]=eo[1]/65536.;r[7]=x[3]*K*180/M_PI;r[8]=cap;r[9]=h.integral/double(h.Q);r[10]=h.z[1]*10./h.Q;r[11]=h.rate_replacement_delta/double(h.Q);r[12]=h.lp/double(h.Q);r[13]=h.damping_gate.weight;
  if(cfg[0]==1){double friction=QPAR[11]*tanh(x[3]/.1);a7_advance(x,EARLY,previous,friction);a7_advance(x,LATE,u,friction);}
  else m03_step(x,previous/1000.,p,.001,4);
  old=previous;previous=u;
 }
 out[1]=j;est_free(e);
}
