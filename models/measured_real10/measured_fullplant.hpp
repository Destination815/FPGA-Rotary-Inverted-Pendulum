#pragma once
#include <cmath>
#include <algorithm>
#include <array>
namespace r33_measured_full {
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

static const double baseline_swing[18]={94.551273717810773,1.6024405068945511,0.78799848756037516,6.0573145533815378,0.073938926902808177,0.87078020794393696,1012.110789101967,1.0249104908387834,7.8816927822951968,115.57779499392562,0.001213799806723643,1.2084401839396252e-06,20119.014778197761,31.405084960139444,0.024879109577951843,0,0.99018601030253428,0.67340752614382771};
static const double baseline_handoff[18]={94.551273717810773,1.479940503340182,0.76918057881760737,3.6083972761286947,0.073938926902808177,0.87078020794393696,818.75748701552493,1.0654316159531239,14.05094635178518,60.100103604253569,0.0010000001656362998,1.5559352526406773e-05,6035.0718945401813,83.819737490053669,0.0095467638392733173,0,0.99018601030253428,0.67340752614382771};
static const double startup_local_update[18]={94.551273717810773,1.6024405068945511,0.78799848756037516,6.0573145533815378,0.073938926902808177,0.87078020794393696,1102.0787457491931,1.0249104908387834,23.645078346739364,80.572858882979887,0.0006068999033619003,1.2084401839396252e-06,20119.014778197761,31.405084960139444,0.024879109577951843,0,0.99018601030253428,0.67340752614382771};
// <=200ms seeded local use for startup update. Not a3s accurate plant.
class Plant{public:std::array<double,7>x{};std::array<double,18>p{};
 void reset(const std::array<double,7>&s,const double*params){x=s;std::copy(params,params+18,p.begin());}
 std::array<double,2> observe()const{double pi=std::acos(-1.),a=std::fmod(x[0]+pi,2*pi);if(a<0)a+=2*pi;a-=pi;if(a>1.7/p[16])a-=2*pi;double m=a*p[16];return {m+2*pi*std::round((x[0]-m)/(2*pi)),x[2]*97763./101376.};}
 bool advance(double logical_pwm){return step(x.data(),logical_pwm,p.data(),2);}
};
}
