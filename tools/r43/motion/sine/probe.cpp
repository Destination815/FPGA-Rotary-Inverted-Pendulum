#include "observer.hpp"
#include "m03_plant.hpp"
extern "C" void* est_new();extern "C" void est_free(void*);extern "C" void est_tick(void*,int,int,int,int,int*);
extern "C" void simulate_sine(const double*p,const double*cfg,const long long*ref,int N,int exit_ms,double*out,double*tr){
 void* est=est_new();int eo[9]={};Obs o;const double K=97763./101376;
 o.cfg.rate_shift=cfg[0];o.cfg.rate_bound=cfg[1];o.cfg.pg=cfg[2];o.cfg.ik_sine=cfg[3];double pscale=cfg[4];o.cfg.arm_scale=cfg[8];o.cfg.shape=cfg[9];o.cfg.shape_bound=cfg[10];o.cfg.shape_fast_shift=cfg[11];o.cfg.shape_slow_shift=cfg[12];o.cfg.friction_scale=cfg[13];o.cfg.dead_scale=cfg[14];o.cfg.damp=cfg[15];
 double x[7]={.001745,0,cfg[7]/K,0,cfg[7]/K,0,0};int u=0,applied=0,k=0;auto sense=[&](){double theta=x[0]*p[16]+cfg[16]*(.002584756*x[1]-.001827150*x[3]+.001479997*84.090917899/p[6]*x[6])+cfg[6];double code=746+theta/(386./65536);int fine=std::clamp((int)llround(code*16),0,16368);est_tick(est,fine>>4,fine,llround(x[2]/(396./65536)),1,eo);};for(int j=0;j<40;j++)sense();
 for(;k<N;k++){
  sense();if(!eo[4]||std::abs(eo[0])>20000||fabs(x[2]*K)>6.3)break;
  const auto*r=ref+4*k;o.measured_rate=eo[1];o.theta_scale=pscale;
  u=o.tick(eo[0],eo[2],applied,int(cfg[5]),r[0],r[1],r[2],r[2],r[3],applied);
  double*z=tr+12*k;z[0]=x[0];z[1]=x[2]*K;z[2]=x[3]*K;z[3]=r[0]/65536.*K;z[4]=r[1]/65536.*K;z[5]=u;z[6]=o.integral/double(Q);z[7]=eo[1]/65536.;z[8]=o.z[1]*10./Q;z[9]=eo[0]/65536.;z[10]=o.lp/double(Q);z[11]=cfg[5];
  m03_step(x,applied/1000.,p,.001,4);applied=u;
 }
 est_free(est);out[0]=k;out[1]=k==N;
}
