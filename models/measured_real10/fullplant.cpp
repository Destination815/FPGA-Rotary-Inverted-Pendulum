#include <cmath>
#include <algorithm>
// M05 nonlinear transmission equations. Private legacy arm coordinates.
// p[16] is the measured rod chart scale; p[17] negative rod dry friction.
static void der(const double*x,double u,const double*p,double*d){
 double delta=x[2]-x[4],spring=std::copysign(std::max(0.,std::abs(delta)-p[14]),delta)*p[12]+p[13]*(x[3]-x[5]);
 double s=std::sin(x[0]-p[15]),c=std::cos(x[0]-p[15]),a=p[3]+p[1]*p[1]+s*s,b=p[1]*c;
 double rp=p[0]*s+s*c*x[5]*x[5]-p[4]*x[1]-(x[1]>=0?p[5]:p[17])*std::tanh(x[1]/.05);
 double ra=spring-2*s*c*x[5]*x[1]+p[1]*s*x[1]*x[1],det=a-b*b;
 d[0]=x[1];d[1]=(a*rp-b*ra)/det;d[2]=x[3];d[3]=(x[6]-p[8]*x[3]-p[9]*std::tanh(x[3]/.08)-spring)/p[2];d[4]=x[5];d[5]=(ra-b*rp)/det;
 double effort=std::copysign(std::max(0.,std::abs(u)-p[11]),u)*(u<0?p[7]:1.);d[6]=(p[6]*effort-x[6])/p[10];
}
static bool step(double*x,double u,const double*p,int sub){double h=.001/sub,a[7],b[7],c[7],d[7],t[7];for(int n=0;n<sub;n++){der(x,u,p,a);for(int j=0;j<7;j++)t[j]=x[j]+h*a[j]/2;der(t,u,p,b);for(int j=0;j<7;j++)t[j]=x[j]+h*b[j]/2;der(t,u,p,c);for(int j=0;j<7;j++)t[j]=x[j]+h*c[j];der(t,u,p,d);for(int j=0;j<7;j++)x[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;}return std::isfinite(x[0])&&std::abs(x[1])<200&&std::abs(x[3])<100;}
extern "C" int rollout(const double*initial,const double*u,const double*p,int n,int sub,double*out){double x[7];std::copy(initial,initial+7,x);for(int t=0;t<=n;t++){std::copy(x,x+7,out+t*7);if(t<n&&!step(x,u[t],p,sub))return t+1;}return -1;}
// Initialization uses only the preceding150ms angles and inputs. No future observations.
static bool past(const double*y,const double*u,const double*p,double w,double v,double*r){
 double x[7]={y[0]/p[16],w/p[16],y[1],v,y[1],v,0};x[6]=p[6]*std::copysign(std::max(0.,std::abs(u[0])-p[11]),u[0])*(u[0]<0?p[7]:1.);
 for(int i=0;i<30;i++){for(int j=0;j<5;j++)if(!step(x,u[i*5+j],p,2))return false;double e=x[0]*p[16]-y[2*(i+1)];double wt=std::exp((i-29)*.005/.05);r[2*i]=wt*std::atan2(std::sin(e),std::cos(e));r[2*i+1]=wt*.5*(x[2]-y[2*(i+1)+1]);}for(int j=0;j<7;j++)r[60+j]=x[j];return true;
}
extern "C" int initialize(const double*y,const double*u,const double*vel,const double*p,int n,double*out){int failures=0;for(int i=0;i<n;i++){
 const double*h=y+i*62,*a=u+i*150;double w=vel[i*2],v=vel[i*2+1],r[67],rw[67],rv[67];bool ok=true;
 for(int it=0;it<4;it++){if(!past(h,a,p,w,v,r)||!past(h,a,p,w+.01,v,rw)||!past(h,a,p,w,v+.01,rv)){ok=false;break;}double aa=.00005,b=0,c=.00005,g=.00005*(w-vel[i*2]),hh=.00005*(v-vel[i*2+1]);for(int k=0;k<60;k++){double dw=(rw[k]-r[k])/.01,dv=(rv[k]-r[k])/.01;aa+=dw*dw;b+=dw*dv;c+=dv*dv;g+=dw*r[k];hh+=dv*r[k];}double det=aa*c-b*b;if(det<1e-16)break;w-=std::clamp((c*g-b*hh)/det,-4.,4.);v-=std::clamp((aa*hh-b*g)/det,-4.,4.);w=std::clamp(w,-35.,35.);v=std::clamp(v,-20.,20.);}
 if(!ok||!past(h,a,p,w,v,r)){failures++;for(int j=0;j<7;j++)out[i*7+j]=NAN;continue;}
 double*x=out+i*7;std::copy(r+60,r+67,x);x[0]=h[60]/p[16];double shift=h[61]-x[2];x[2]+=shift;x[4]+=shift;
 }return failures;}
extern "C" void batch_replay(const double*initial,const double*u,const double*p,int n,int horizon,int stride,double*out){
 for(int i=0;i<n;i++){double x[7];std::copy(initial+i*7,initial+i*7+7,x);for(int t=0;t<=horizon;t++){if(t%stride==0){int o=(i*(horizon/stride+1)+t/stride)*2;out[o]=x[0]*p[16];out[o+1]=x[2];}if(t<horizon&&!step(x,u[i*horizon+t],p,2)){for(int j=t+1;j<=horizon;j++)if(j%stride==0){int o=(i*(horizon/stride+1)+j/stride)*2;out[o]=out[o+1]=100.;}break;}}}
}
