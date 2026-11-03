#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstdio>
using I=int64_t;
#include "constants.hpp"
constexpr I Q=1<<24;
inline I sat(I x){return std::clamp<I>(x,-2147483648LL,2147483647LL);}
inline I rnd(I x,I d){I m=std::abs(x),q=m/d,r=m%d;return (x<0?-1:1)*(q+((2*r>d)||(2*r==d&&(q&1))));}
inline I lut(I x){I i=(std::abs(x)*10240+Q/2)>>24;return (x<0?-1:1)*(i<1025?TABLE[i]:Q);}
struct Params{double vf=29./32,pf=2,pg=1,damp=1.2,shape=2;int center=1;double ik=1;int smooth=1,slew=20,rate_shift=0,shape_fast_shift=4,shape_slow_shift=6;double rate_bound=.04,ik_sine=1,arm_scale=1,shape_bound=.04,friction_scale=1,dead_scale=1;};
struct Obs{
 I z[8]={},integral=0,lp=0,age=0,samples=0,fb=0,fast=0,slow=0,bias=0,sp=0,sv=0,sc=0;bool shvalid=false,svalid=false,smvalid=false;
 I raw=0,req=0,correction=0;I measured_rate=0; double theta_scale=1; Params cfg;
 int tick(I t,I p,I prev,I cap,I r,I rv,bool motion,bool sine,I sr,I pred){
  I old[8];std::copy(z,z+8,old);I prior=pred*16777,e=prior>=0?std::max<I>(0,prior-C_dead):std::min<I>(0,prior+C_dead);e=sat(e*(e<0?C_ratio:Q)>>24);
  I v[12];std::copy(old,old+8,v);v[8]=e;v[9]=lut(old[3]);v[10]=t*256;v[11]=p*256;
  I n[8];for(int i=0;i<8;i++){I a=0;for(int j=0;j<12;j++)a+=(v[j]*MATRIX[i][j])>>24;n[i]=sat(a);}n[7]=std::clamp<I>(n[7],-5033165,5033165);
  I oldlp=lp;
  lp=sat(lp+(((p-r)*256-lp)*16760>>24));
  if(!(cfg.center&&sine)){
   if(std::abs(n[1])>838861||std::abs(n[3])>838861)lp=(std::abs(r)>2288&&std::abs(rv)<6554)?oldlp:0;
   if(motion&&((p>=r&&oldlp<0)||(p<=r&&oldlp>0)))lp=0;
  }
  I istep=lp*I(cfg.center&&sine?llround(1048*cfg.ik_sine):4194)>>24;
  I gain[7];std::copy(GAIN,GAIN+7,gain);gain[0]=llround(74306208*(sine?theta_scale:1));gain[1]=std::llround(gain[1]*(sine?cfg.damp:1));gain[2]=std::llround(gain[2]*(sine?cfg.pg:1));gain[3]=std::llround(gain[3]*(sine?cfg.arm_scale:1));
  I pe=int32_t(n[2]-r*256),operand=motion?sat(pe+(std::clamp<I>(pe,-5856353,5856353)>>3)):pe;
  I target=std::clamp<I>((motion?2:1)*(p*256-n[2]),-335544,335544);fb=motion?fb+((target-fb)>>2):target;I fvel=sat(n[3]+fb);
  if(!smvalid){sp=p*256;sv=rnd(fvel,100);sc=0;}else{
   I prediction=sp+sv,residual=p*256-prediction,innovation=residual>50688?residual-50688:residual< -50688?residual+50688:0;
   sp=sat(prediction+rnd(15*innovation,64));sv=sat(sv+rnd(innovation,64));I targ=std::clamp<I>(rnd(sat(100*sv)-fvel,2),sine?-167772:0,sine?167772:0);sc=sine?std::clamp<I>(targ,sc-8192,sc+8192):0;
  }smvalid=true;if(cfg.smooth)fvel=sat(fvel+sc);
  I rate_delta=(sine&&cfg.rate_shift)?std::clamp<I>((measured_rate*256-10*n[1])>>cfg.rate_shift,-llround(cfg.rate_bound*Q),llround(cfg.rate_bound*Q)):0;
  I sum=0;for(int i=0;i<7;i++)sum+=((i==2?operand:i==3?fvel:i==1?sat(n[1]+rate_delta):n[i])*gain[i])>>24;
  I request=sat(sum+(lut(n[3])*I(llround(C_friction*cfg.friction_scale))>>24)+integral+(n[7]*C_disturbance_gain_q24>>24)+((rv*256)*I(std::llround((sine?-2819064*cfg.vf:-3523830)))>>24));raw=request;
  I source=sat(sum);if(shvalid){fast+=(source-fast)>>(sine?cfg.shape_fast_shift:4);slow+=(source-slow)>>(sine?cfg.shape_slow_shift:6);}else{fast=slow=source;shvalid=true;}
  correction=std::clamp<I>(I(std::llround(((fast-slow)>>(sine?0:2))*(sine?cfg.shape/2:1))),-I((sine?cfg.shape_bound:.04)*Q),I((sine?cfg.shape_bound:.04)*Q));request=sat(request-correction);
  if(sine){I s=sr*256;bias=!svalid?s:std::abs(bias)<=31?0:bias-(bias>>5);request=sat(request+(cfg.pf==1.25?((s-bias)>>3)+((s-bias)>>5):I(std::llround(((s-bias)>>3)*cfg.pf))));}else bias=0;svalid=sine;
  req=sat((request*(request<0?C_inverse_ratio:Q)>>24)+(request<0?-I(llround(C_dead*cfg.dead_scale)):request>0?I(llround(C_dead*cfg.dead_scale)):0));int wanted=rnd(req*1000,Q);int u=std::clamp<int>(std::clamp<int>(std::clamp<int>(wanted,-cap,cap),prev-(sine?cfg.slew:20),prev+(sine?cfg.slew:20)),-cap,cap);I err=req-u*16777;
  bool integral_gate=(!motion || cfg.center&&sine);
  if(integral_gate&&age>=1000&&std::abs(t*256)<2348810&&(cfg.center&&sine||std::abs(n[1])<838861&&std::abs(n[3])<838861)&&(std::abs(err)<50332||istep*err<0))integral=std::clamp<I>(integral+istep,-5872026,5872026);
  age=std::min<I>(1000,age+1);samples=std::min<I>(200,samples+1);std::copy(n,n+8,z);return u;
 }
};
