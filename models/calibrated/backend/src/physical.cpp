#include <cmath>
#include <algorithm>
#include "nonlinear_original.cpp"
#ifndef STEP_SUB
#define STEP_SUB 1
#endif
// Causal initial velocities from measured history; optimize only past residuals.
static void step(double*x,double u,const double*p,int kind,double dt){if(kind)mass_step(x,u,p,dt,STEP_SUB);else effective_step(x,u,p,dt,STEP_SUB);}
static bool run(const double*y,const double*u,const double*p,int kind,double w,double v,double*out){
 double x[4]={y[0],w,y[1],v};
 for(int k=0;k<30;k++){step(x,u[k],p,kind,.005);if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80)return false;
 out[2*k]=std::atan2(sin(x[0]-y[2*(k+1)]),cos(x[0]-y[2*(k+1)]));out[2*k+1]=(x[2]-y[2*(k+1)+1])*.5;}
 for(int j=0;j<4;j++)out[60+j]=x[j];return true;
}
extern "C" void forecast(const double*Y,const double*U,const double*V,const double*p,int n,int kind,double*out){
 for(int i=0;i<n;i++){
 const double*y=Y+i*142,*u=U+i*70;double w=V[2*i],v=V[2*i+1],r[64],rw[64],rv[64];bool ok=true;
 for(int iter=0;iter<4;iter++){
 if(!run(y,u,p,kind,w,v,r)||!run(y,u,p,kind,w+.01,v,rw)||!run(y,u,p,kind,w,v+.01,rv)){ok=false;break;}
 double a=.00005,b=0,c=.00005,g=.00005*(w-V[2*i]),h=.00005*(v-V[2*i+1]);
 for(int k=0;k<60;k++){double dw=(rw[k]-r[k])/.01,dv=(rv[k]-r[k])/.01;a+=dw*dw;b+=dw*dv;c+=dv*dv;g+=dw*r[k];h+=dv*r[k];}
 double det=a*c-b*b;if(det<1e-16)break;
 w-=std::clamp((c*g-b*h)/det,-4.,4.);v-=std::clamp((a*h-b*g)/det,-4.,4.);w=std::clamp(w,-35.,35.);v=std::clamp(v,-20.,20.);
 }
 if(!ok||!run(y,u,p,kind,w,v,r)){for(int k=0;k<82;k++)out[i*82+k]=100.;continue;}
 double x[4]={y[60],r[61],y[61],r[63]}; // reset angles to current measurements, velocities causal
 for(int k=0;k<=40;k++){
 out[i*82+2*k]=x[0];out[i*82+2*k+1]=x[2];
 if(k<40)step(x,u[30+k],p,kind,.005);
 if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80){for(int j=k+1;j<=40;j++){out[i*82+2*j]=100.;out[i*82+2*j+1]=100.;}break;}
 }
 }
}
