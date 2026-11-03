#include <cmath>
#include <algorithm>
// Effective dynamic model, not uniquely identified rigid-body inertia.
// p: g,k,bp,cp,gain,ratio,bm,cm,tau,lag,mu,dead,obs,offset.
static void der(const double*x,double u,const double*p,double*d){
 double s=sin(x[0]-p[13]),c=cos(x[0]-p[13]);
 double effort=copysign(std::max(0.,fabs(u)-p[11]),u)*(u<0?p[5]:1.);
 double rod=p[0]*s-p[2]*x[1]-p[3]*tanh(x[1]/.05);
 double a=x[4]-p[6]*x[3]-p[10]*c*rod-p[7]*tanh(x[3]/.08);
 d[0]=x[1];d[1]=rod-p[1]*c*x[6]+s*c*x[3]*x[3];d[2]=x[3];d[3]=a;
 d[4]=(p[4]*effort-x[4])/p[8];d[5]=(a-x[5])/p[9];d[6]=(x[5]-x[6])/p[9];
}
static double obs(const double*x,double u,const double*p){double d[7];der(x,u,p,d);return x[0]+p[12]*cos(x[0]-p[13])*d[3];}
extern "C" void ng_step(double*x,double u,const double*p,double dt,int sub){double h=dt/sub,a[7],b[7],c[7],d[7],t[7];for(int n=0;n<sub;n++){der(x,u,p,a);for(int j=0;j<7;j++)t[j]=x[j]+h*a[j]/2;der(t,u,p,b);for(int j=0;j<7;j++)t[j]=x[j]+h*b[j]/2;der(t,u,p,c);for(int j=0;j<7;j++)t[j]=x[j]+h*c[j];der(t,u,p,d);for(int j=0;j<7;j++)x[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;}}
static bool past(const double*y,const double*u,const double*p,double w,double v,double*r){
 double x[7]={y[0],w,y[1],v,0,0,0};x[4]=p[4]*copysign(std::max(0.,fabs(u[0])-p[11]),u[0])*(u[0]<0?p[5]:1.);
 for(int z=0;z<3;z++)x[0]+=y[0]-obs(x,u[0],p);
 for(int i=0;i<30;i++){ng_step(x,u[i],p,.005,5);if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80)return false;double e=obs(x,u[i],p)-y[2*(i+1)];r[2*i]=atan2(sin(e),cos(e));r[2*i+1]=.5*(x[2]-y[2*(i+1)+1]);}
 for(int j=0;j<7;j++)r[60+j]=x[j];return true;
}
extern "C" void forecast(const double*Y,const double*U,const double*V,const double*p,int n,int kind,double*out){
 for(int i=0;i<n;i++){const double*y=Y+i*142,*u=U+i*70;double w=V[2*i],v=V[2*i+1],r[67],rw[67],rv[67];bool ok=true;
 for(int it=0;it<4;it++){if(!past(y,u,p,w,v,r)||!past(y,u,p,w+.01,v,rw)||!past(y,u,p,w,v+.01,rv)){ok=false;break;}double a=.00005,b=0,c=.00005,g=.00005*(w-V[2*i]),h=.00005*(v-V[2*i+1]);for(int k=0;k<60;k++){double dw=(rw[k]-r[k])/.01,dv=(rv[k]-r[k])/.01;a+=dw*dw;b+=dw*dv;c+=dv*dv;g+=dw*r[k];h+=dv*r[k];}double det=a*c-b*b;if(det<1e-16)break;w-=std::clamp((c*g-b*h)/det,-4.,4.);v-=std::clamp((a*h-b*g)/det,-4.,4.);w=std::clamp(w,-35.,35.);v=std::clamp(v,-20.,20.);}
 if(!ok||!past(y,u,p,w,v,r)){for(int k=0;k<82;k++)out[i*82+k]=100;continue;}
 double x[7];for(int j=0;j<7;j++)x[j]=r[60+j];x[2]=y[61];for(int z=0;z<3;z++)x[0]+=atan2(sin(y[60]-obs(x,u[29],p)),cos(y[60]-obs(x,u[29],p)));
 for(int k=0;k<=40;k++){out[i*82+2*k]=obs(x,u[k?29+k:29],p);out[i*82+2*k+1]=x[2];if(k<40)ng_step(x,u[30+k],p,.005,5);if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80){for(int j=k+1;j<=40;j++)out[i*82+2*j]=out[i*82+2*j+1]=100;break;}}
 }}
