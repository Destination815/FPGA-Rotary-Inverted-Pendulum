#include <cmath>
#include <algorithm>
// Encoder is motor-side; load angle is a distinct latent state. Hypothesis, not diagnosed backlash.
// p: g,k,Im,Il,bp,cp,gain,ratio,Bm,Fm,tau,dead,Kshaft,Dshaft,gap,offset
static void der(const double*x,double u,const double*p,double*d){
 double delta=x[2]-x[4],spring=copysign(std::max(0.,fabs(delta)-p[14]),delta)*p[12]+p[13]*(x[3]-x[5]);
 double s=sin(x[0]-p[15]),c=cos(x[0]-p[15]),a=p[3]+p[1]*p[1]+s*s,b=p[1]*c;
 double rp=p[0]*s+s*c*x[5]*x[5]-p[4]*x[1]-p[5]*tanh(x[1]/.05);
 double ra=spring-2*s*c*x[5]*x[1]+p[1]*s*x[1]*x[1],det=a-b*b;
 d[0]=x[1];d[1]=(a*rp-b*ra)/det;d[2]=x[3];d[3]=(x[6]-p[8]*x[3]-p[9]*tanh(x[3]/.08)-spring)/p[2];d[4]=x[5];d[5]=(ra-b*rp)/det;
 double effort=copysign(std::max(0.,fabs(u)-p[11]),u)*(u<0?p[7]:1.);d[6]=(p[6]*effort-x[6])/p[10];
}
static void m03_step(double*x,double u,const double*p,double dt,int sub){double h=dt/sub,a[7],b[7],c[7],d[7],t[7];for(int n=0;n<sub;n++){der(x,u,p,a);for(int j=0;j<7;j++)t[j]=x[j]+h*a[j]/2;der(t,u,p,b);for(int j=0;j<7;j++)t[j]=x[j]+h*b[j]/2;der(t,u,p,c);for(int j=0;j<7;j++)t[j]=x[j]+h*c[j];der(t,u,p,d);for(int j=0;j<7;j++)x[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;}}
