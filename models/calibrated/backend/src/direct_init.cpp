#include "nonlinear_original.cpp"
extern "C" void forecast(const double*Y,const double*U,const double*V,const double*p,int n,int kind,double*out){for(int i=0;i<n;i++){const double*y=Y+i*142,*u=U+i*70;double x[4]={y[60],V[2*i],y[61],V[2*i+1]};for(int k=0;k<=40;k++){out[i*82+2*k]=x[0];out[i*82+2*k+1]=x[2];if(k<40)mass_step(x,u[30+k],p,.005,5);}}}
