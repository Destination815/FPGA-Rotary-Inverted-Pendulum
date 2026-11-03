#include <algorithm>
#include <cmath>
// Parameters: gain+, gain-, tau_on, kinetic_PWM, static_PWM, tau_off.
// Friction is applied to the arm actuator, not the rod. No measured future
// state enters this function; u is a supplied forcing sequence.
static void advance(double &f,double &v,double u,const double*p,double h){
 const double S=p[4],C=p[3],gain=u>=0?p[0]:p[1];
 if(std::abs(v)<1e-10 && std::abs(u)<=S){v=0;return;}
 const double direction=std::abs(v)>1e-10?std::copysign(1.,v):std::copysign(1.,u);
 const double target=gain*(u-C*direction),tau=std::abs(u)<1e-12?p[5]:p[2];
 const double decay=std::exp(-h/tau),vn=target+(v-target)*decay;
 if(v*vn<0 && std::abs(u)<=S){
   // Stop at the analytic zero crossing, then static friction holds the rest.
   const double tc=std::max(0.,std::min(h,-tau*std::log(-target/(v-target))));
   f+=target*tc+(v-target)*tau*(1-std::exp(-tc/tau));v=0;
 }else{f+=target*h+(v-target)*tau*(1-decay);v=vn;}
}
extern "C" void predict(const double*xx,const double*uu,const int*lengths,
 const double*p,int n,int width,double*out){
 for(int k=0;k<n;k++){
  double f=xx[k*2],v=xx[k*2+1];
  for(int t=0;t<lengths[k];t++){
   const double u=uu[k*width+t];
   for(int j=0;j<4;j++)advance(f,v,u,p,.00025);
   out[k*width+t]=f;
  }
 }
}
