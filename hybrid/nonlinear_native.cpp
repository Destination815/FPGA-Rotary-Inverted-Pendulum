// Equivalent scalar mass-matrix / effective-speed plants for offline simulation only.
#include <cmath>
#include <algorithm>
static void deriv(const double*x,double u,const double*p,double*d){
 const double m=.06,l=.075,g=9.81,J=m*g*l/p[0],r=p[1]*g/p[0],K=.647,gain=(p[4]+p[5])/2,B=K/gain,Jr=p[6]*B-m*r*r+m*m*r*r*l*l/J;
 double s=sin(x[0]),c=cos(x[0]),a=Jr+m*r*r+J*s*s,b=m*r*l*c;
 double eff=copysign(std::max(0.,fabs(u)-p[8]),u),be=fabs(eff)>1e-12?B:(Jr+m*r*r-b*b/J)/p[7];
 double ra=K*((u>=0?p[4]:p[5])/gain)*eff-be*x[3]-2*J*s*c*x[3]*x[1]+m*r*l*s*x[1]*x[1];
 double rp=m*g*l*s+J*s*c*x[3]*x[3]-J*p[2]*x[1]-J*p[3]*tanh(x[1]/.05),det=a*J-b*b;
 d[0]=x[1];d[1]=(a*rp-b*ra)/det;d[2]=x[3];d[3]=(J*ra-b*rp)/det;
}
extern "C" void mass_step(double*x,double u,const double*p,double dt,int sub){
 double h=dt/sub,k[4][4],tmp[4];for(int n=0;n<sub;n++){
 deriv(x,u,p,k[0]);for(int j=0;j<4;j++)tmp[j]=x[j]+h*k[0][j]/2;deriv(tmp,u,p,k[1]);
 for(int j=0;j<4;j++)tmp[j]=x[j]+h*k[1][j]/2;deriv(tmp,u,p,k[2]);for(int j=0;j<4;j++)tmp[j]=x[j]+h*k[2][j];deriv(tmp,u,p,k[3]);
 for(int j=0;j<4;j++)x[j]+=h*(k[0][j]+2*k[1][j]+2*k[2][j]+k[3][j])/6;}}
extern "C" void effective_step(double*x,double u,const double*p,double dt,int sub){
 double h=dt/sub,eff=copysign(std::max(0.,fabs(u)-p[8]),u),target=(eff>=0?p[4]:p[5])*eff,tau=fabs(eff)>1e-12?p[6]:p[7],eh=exp(-h/tau),em=exp(-h/(2*tau));
 auto wdot=[&](double t,double w,double v,double a){return p[0]*sin(t)-p[1]*cos(t)*a+sin(t)*cos(t)*v*v-p[2]*w-p[3]*tanh(w/.05);};
 for(int n=0;n<sub;n++){double t=x[0],w=x[1],f=x[2],v=x[3],a0=(target-v)/tau,vm=target+(v-target)*em,am=(target-vm)/tau,tm=t+.5*h*w,wm=w+.5*h*wdot(t,w,v,a0);
 x[0]=t+h*wm;x[1]=w+h*wdot(tm,wm,vm,am);x[2]=f+target*h+(v-target)*tau*(1-eh);x[3]=target+(v-target)*eh;}}
