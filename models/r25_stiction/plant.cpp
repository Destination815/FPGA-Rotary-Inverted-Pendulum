// Supplemental EFFECTIVE plant with explicit low-speed arm stiction.
// Retains the original rod differential equation; no claim of full coupled
// mass-matrix identification. Include this file OR link its shared library.
#include "predict.cpp"
static double stiction_acceleration(double v,double u,const double*m){
 if(std::abs(v)<1e-10 && std::abs(u)<=m[4])return 0;
 const double direction=std::abs(v)>1e-10?std::copysign(1.,v):std::copysign(1.,u);
 const double target=(u>=0?m[0]:m[1])*(u-m[3]*direction);
 const double tau=std::abs(u)<1e-12?m[5]:m[2];
 return (target-v)/tau;
}
// x=[theta_rad,omega_rad_s,phi_rad,arm_velocity_rad_s]
// p=unchanged model-bank order: alpha,kappa,damping,rod_coulomb,
//   gain+,gain-,tau_on,tau_off,deadzone. Only p[0:4] used here.
// motor=[gain+,gain-,tau_on,kinetic_PWM,static_PWM,tau_off].
// u is logical COMMITTED PWM fraction, before MOTOR_SIGN=-1.
extern "C" void stiction_effective_step(double*x,double u,const double*p,
 const double*motor,double dt,int sub){
 const int count=std::max(1,sub);const double h=dt/count;
 auto wd=[&](double t,double w,double v,double a){
  return p[0]*std::sin(t)-p[1]*std::cos(t)*a
   +std::sin(t)*std::cos(t)*v*v-p[2]*w-p[3]*std::tanh(w/.05);
 };
 for(int k=0;k<count;k++){
  const double t=x[0],w=x[1],v=x[3];
  double fm=x[2],vm=v,ff=x[2],vf=v;
  advance(fm,vm,u,motor,h*.5);advance(ff,vf,u,motor,h);
  const double a0=stiction_acceleration(v,u,motor),am=stiction_acceleration(vm,u,motor),af=stiction_acceleration(vf,u,motor);
  const double k1t=w,k1w=wd(t,w,v,a0);
  const double k2t=w+h*k1w*.5,k2w=wd(t+h*k1t*.5,w+h*k1w*.5,vm,am);
  const double k3t=w+h*k2w*.5,k3w=wd(t+h*k2t*.5,w+h*k2w*.5,vm,am);
  const double k4t=w+h*k3w,k4w=wd(t+h*k3t,w+h*k3w,vf,af);
  x[0]=t+h*(k1t+2*k2t+2*k3t+k4t)/6;
  x[1]=w+h*(k1w+2*k2w+2*k3w+k4w)/6;
  x[2]=ff;x[3]=vf;
 }
}
