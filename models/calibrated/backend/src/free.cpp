#include <cmath>
#include <algorithm>
static void der(const double*x,const double*p,double*d){d[0]=x[1];d[1]=p[0]*sin(x[0]-p[4])-p[1]*x[1]-(x[1]>=0?p[2]:p[3])*tanh(x[1]/.05);}
extern "C" void free_run(double theta,double omega,const double*p,int n,double dt,double*out){
 double x[2]={theta,omega},a[2],b[2],c[2],d[2],t[2];int sub=std::max(1,(int)ceil(dt/.001));double h=dt/sub;
 for(int i=0;i<n;i++){out[i]=x[0];for(int k=0;k<sub;k++){der(x,p,a);for(int j=0;j<2;j++)t[j]=x[j]+h*a[j]/2;der(t,p,b);for(int j=0;j<2;j++)t[j]=x[j]+h*b[j]/2;der(t,p,c);for(int j=0;j<2;j++)t[j]=x[j]+h*c[j];der(t,p,d);for(int j=0;j<2;j++)x[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;}}
}
