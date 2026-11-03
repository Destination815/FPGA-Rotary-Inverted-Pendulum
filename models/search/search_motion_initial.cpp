#include <algorithm>
#include <cmath>
static double OA[49],OB[7],OC[14],OF[7];
extern "C" void setup(const double*A,const double*B,const double*C,const double*F){std::copy(A,A+49,OA);std::copy(B,B+7,OB);std::copy(C,C+14,OC);std::copy(F,F+7,OF);}
// p: integral gain, integral bound, LP tau, kpos multiplier, reference velocity FF, acceleration FF.
extern "C" int run(int N,const double*A,const double*B,const double*C,const double*F,const double*q,const double*K,const double*L,const double*ref,const double*noise,const double*p,double bias,double kick,double*metrics,double*out){
 double x[7]={.005,0,0,0,0,0,0},z[7]={},prev=0,integ=0,lp=0;int cap=150,good=0;bool boost=false;
 x[0]=.005-bias;
 double sumt=0,sump=0,st=0,sp=0,meanp=0,maxth=0;int n=0;
 for(int i=0;i<N;i++){
  if(i==8000)x[1]+=kick;if(i==16000)x[1]-=kick;
  double obs[2]={};for(int j=0;j<7;j++){obs[0]+=C[j]*x[j];obs[1]+=C[7+j]*x[j];}obs[0]+=bias+noise[i];
  double y[2]={std::nearbyint(obs[0]/(386./65536)*16)*(386./65536)/16,std::nearbyint(obs[1]/(396./65536))*(396./65536)};
  double pred[7];for(int j=0;j<7;j++){pred[j]=OB[j]*(std::copysign(std::max(0.,fabs(prev)-.017767898479005756),prev)*(prev>=0?1:1.2460994621152977))+OF[j]*2.6127569092556304*tanh(z[3]/.1);for(int l=0;l<7;l++)pred[j]+=OA[j*7+l]*z[l];}
  double e[2]={y[0],y[1]};for(int j=0;j<7;j++){e[0]-=OC[j]*pred[j];e[1]-=OC[7+j]*pred[j];}for(int j=0;j<7;j++)z[j]=pred[j]+L[j*2]*e[0]+L[j*2+1]*e[1];
  if(i>=200&&fabs(y[0]+.07689961962701022*z[1])>.07663625093047369&&y[0]*z[1]>0){boost=true;good=0;}
  if(boost){if(fabs(y[0])<.04&&fabs(z[1])<.3)good++;else good=0;if(good>=50)boost=false;}
  int tgt=boost?350:150;cap=cap<tgt?std::min(tgt,cap+10):std::max(tgt,cap-2);
  double r=ref[3*i],rv=ref[3*i+1],ra=ref[3*i+2],err=y[1]-r;
  lp+=(err-lp)*(.001/(p[2]+.001));
  double request=0;for(int j=0;j<7;j++)request-=K[j]*(z[j]-(j==2?r:0))*(j==2?p[3]:1.);
  request+=integ+p[4]*rv+p[5]*ra+.9124706592816184*2.6127569092556304*tanh(z[3]/.1)/84.09091789902548;
  double raw=request/(request<0?1.2460994621152977:1)+(request>0?.017767898479005756:request<0?-.017767898479005756:0.);
  double limited=std::clamp(raw,-cap/1000.,cap/1000.);double u=std::clamp(std::nearbyint(limited*1000)/1000,prev-.02,prev+.02);
  // conditional anti-windup includes both amplitude and slew, never boost flag alone
  double step=p[0]*lp*.001;
  if(i>=1000&&fabs(y[0])<.14&&fabs(z[1])<4 && (fabs(raw-u)<.003||step*(raw-u)<0))integ=std::clamp(integ+step,-p[1],p[1]);
  sumt+=obs[0]*obs[0];sump+=err*err;maxth=std::max(maxth,fabs(obs[0]));if(i>=N-5000){st+=obs[0]*obs[0];sp+=err*err;meanp+=err;}
  if(out){out[6*i]=obs[0];out[6*i+1]=obs[1];out[6*i+2]=r;out[6*i+3]=u;out[6*i+4]=integ;out[6*i+5]=cap/1000.;}
  n=i+1;if(!std::isfinite(obs[0])||fabs(obs[0])>.261799||fabs(obs[1])>1.047){break;}
  double drive=std::copysign(std::max(0.,fabs(prev)-q[9]),prev)*(prev>=0?1:q[8]);double nx[7];for(int j=0;j<7;j++){nx[j]=B[j]*drive+F[j]*q[11]*tanh(x[3]/.1);for(int l=0;l<7;l++)nx[j]+=A[j*7+l]*x[l];}std::copy(nx,nx+7,x);prev=u;
 }
 metrics[0]=n;metrics[1]=sqrt(sumt/n);metrics[2]=sqrt(sump/n);metrics[3]=maxth;metrics[4]=sqrt(st/5000);metrics[5]=sqrt(sp/5000);metrics[6]=meanp/5000;metrics[7]=integ;return n;
}
