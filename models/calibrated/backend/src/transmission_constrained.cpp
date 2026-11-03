#include <cmath>
#include <algorithm>
// Encoder is motor-side; load angle is a distinct latent state. Hypothesis, not diagnosed backlash.
// p: g,k,Im,Il,bp,cp,gain,ratio,Bm,Fm,tau,dead,Kshaft,Dshaft,gap,offset
static void der(const double*x,double u,const double*p,double*d){
 double delta=x[2]-x[4],spring=copysign(std::max(0.,fabs(delta)-p[14]),delta)*p[12]+p[13]*(x[3]-x[5]);
 double s=sin(x[0]-p[15]),c=cos(x[0]-p[15]),a=p[3]+p[1]*p[1]+s*s,b=p[1]*c;
 double rp=p[0]*s+s*c*x[5]*x[5]-p[4]*x[1]-(x[1]>=0?p[5]:p[17])*tanh(x[1]/.05);
 double ra=spring-2*s*c*x[5]*x[1]+p[1]*s*x[1]*x[1],det=a-b*b;
 d[0]=x[1];d[1]=(a*rp-b*ra)/det;d[2]=x[3];d[3]=(x[6]-p[8]*x[3]-p[9]*tanh(x[3]/.08)-spring)/p[2];d[4]=x[5];d[5]=(ra-b*rp)/det;
 double effort=copysign(std::max(0.,fabs(u)-p[11]),u)*(u<0?p[7]:1.);d[6]=(p[6]*effort-x[6])/p[10];
}
extern "C" void tr_step(double*x,double u,const double*p,double dt,int sub){double h=dt/sub,a[7],b[7],c[7],d[7],t[7];for(int n=0;n<sub;n++){der(x,u,p,a);for(int j=0;j<7;j++)t[j]=x[j]+h*a[j]/2;der(t,u,p,b);for(int j=0;j<7;j++)t[j]=x[j]+h*b[j]/2;der(t,u,p,c);for(int j=0;j<7;j++)t[j]=x[j]+h*c[j];der(t,u,p,d);for(int j=0;j<7;j++)x[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;}}
static bool past(const double*y,const double*u,const double*p,double w,double v,double*r){
 double x[7]={y[0]/p[16],w/p[16],y[1],v,y[1],v,0};x[6]=p[6]*copysign(std::max(0.,fabs(u[0])-p[11]),u[0])*(u[0]<0?p[7]:1.);
 for(int i=0;i<30;i++){for(int j=0;j<5;j++)tr_step(x,u[i*5+j],p,.001,2);if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80)return false;double e=x[0]*p[16]-y[2*(i+1)];double wt=exp((i-29)*.005/.05);r[2*i]=wt*atan2(sin(e),cos(e));r[2*i+1]=wt*.5*(x[2]-y[2*(i+1)+1]);}for(int j=0;j<7;j++)r[60+j]=x[j];return true;}
extern "C" void forecast(const double*Y,const double*U,const double*V,const double*p,int n,int kind,double*out){
 for(int i=0;i<n;i++){const double*src=Y+i*142,*u=U+i*350;double local[142];double wrapped=atan2(sin(src[0]),cos(src[0]));if(wrapped>1.7)wrapped-=2*acos(-1.);double shift=src[0]-wrapped;for(int j=0;j<71;j++){local[2*j]=src[2*j]-shift;local[2*j+1]=src[2*j+1];}const double*y=local;double w=V[2*i],v=V[2*i+1],r[67],rw[67],rv[67];bool ok=true;
 for(int it=0;it<4;it++){if(!past(y,u,p,w,v,r)||!past(y,u,p,w+.01,v,rw)||!past(y,u,p,w,v+.01,rv)){ok=false;break;}double a=.00005,b=0,c=.00005,g=.00005*(w-V[2*i]),h=.00005*(v-V[2*i+1]);for(int k=0;k<60;k++){double dw=(rw[k]-r[k])/.01,dv=(rv[k]-r[k])/.01;a+=dw*dw;b+=dw*dv;c+=dv*dv;g+=dw*r[k];h+=dv*r[k];}double det=a*c-b*b;if(det<1e-16)break;w-=std::clamp((c*g-b*h)/det,-4.,4.);v-=std::clamp((a*h-b*g)/det,-4.,4.);w=std::clamp(w,-35.,35.);v=std::clamp(v,-20.,20.);}
 if(!ok||!past(y,u,p,w,v,r)){for(int k=0;k<82;k++)out[i*82+k]=100;continue;}double x[7];for(int j=0;j<7;j++)x[j]=r[60+j];x[0]=y[60]/p[16];double phshift=y[61]-x[2];x[2]+=phshift;x[4]+=phshift;
 for(int k=0;k<=40;k++){out[i*82+2*k]=x[0]*p[16]+shift;out[i*82+2*k+1]=x[2];if(k<40)for(int j=0;j<5;j++)tr_step(x,u[(30+k)*5+j],p,.001,2);if(!std::isfinite(x[0])||fabs(x[1])>150||fabs(x[3])>80){for(int j=k+1;j<=40;j++)out[i*82+2*j]=out[i*82+2*j+1]=100;break;}}}}
