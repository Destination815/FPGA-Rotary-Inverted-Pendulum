#include "Vr01_reference.h"
#include "Vr01_reference___024root.h"
#include "verilated.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
static int checks=0,maxcycles=0;
static void need(bool yes,const char*why){checks++;if(!yes)throw std::runtime_error(why);}
static void clock1(Vr01_reference&d){d.clk=0;d.eval();d.clk=1;d.eval();}
static int tick(Vr01_reference&d){d.start=1;clock1(d);d.start=0;int n=1;while(!d.done&&n<20000){clock1(d);n++;}need(n<20000,"reference deadline");maxcycles=std::max(n,maxcycles);clock1(d);return n;}
static int expected(int32_t delta,int32_t v){
 const int times[]={2,4,6,8,12,16,20,24,32,48,64,96,192,384,768,1536};
 uint64_t distance=std::llabs((int64_t)delta);
 for(auto t:times){uint64_t ac=std::min<int64_t>(2147483647LL,std::llround(M_PI/6*t*t/5.774*262144));if(distance*15<=(uint64_t)v*8*t&&distance<=ac)return t*1000;}
 return 1536000;
}
int main(){try{
 Vr01_reference d;d.rst_n=0;d.clear=0;clock1(d);d.rst_n=1;clock1(d);d.amplitude=68629;d.period_seconds=20;d.target_bound=1650000;d.speed_level=1;
 for(int speed: {1186,23000,44747,71165})for(int degrees:{-350,-90,-15,0,15,90,350}){
  d.clear=1;clock1(d);d.clear=0;d.reseed=1;d.seed_phi=-d.target_bound;d.target_valid=0;d.speed_limit=speed*4;tick(d);d.reseed=0;
  int32_t target=std::llround(degrees*M_PI/180*262144);d.target=target;d.target_valid=1;tick(d);d.target_valid=0;
  int duration=expected(target+(int)d.target_bound,speed*4);
  need(d.rootp->r01_reference__DOT__duration==(uint32_t)duration,"numeric duration mismatch");
  for(double f:{.0,.125,.25,.5,.75,.875,1.0}){
   uint32_t age=std::llround(duration*f);d.rootp->r01_reference__DOT__age=age;tick(d);
   double x=f*f*f*(10+f*(-15+6*f));double delta=(target+(int)d.target_bound)/262144.;double want=-(int32_t)d.target_bound/262144.+delta*x;
   double vel=delta/duration*1000*30*f*f*(1-f)*(1-f);
   need(std::abs((int32_t)d.__SYM__reference/262144.-want)<.002,"numeric position polynomial mismatch");
   need(std::abs((int32_t)d.ref_velocity/262144.-vel)<.002,"numeric velocity polynomial mismatch");
   need(std::abs((int32_t)d.ref_velocity/262144.)<speed*4/262144.+.002,"numeric velocity limit exceeded");
  }
 }
 d.clear=1;clock1(d);d.clear=0;d.speed_limit=0;d.reseed=1;d.seed_phi=0;d.target_valid=0;tick(d);d.reseed=0;d.target_valid=1;d.target=411775;d.speed_level=1;tick(d);need(d.rootp->r01_reference__DOT__duration==12000,"old speed20 duration changed");
 d.target_valid=0;d.start=1;clock1(d);d.start=0;clock1(d);d.clear=1;clock1(d);need(!d.busy&&!d.done&&d.__SYM__reference==0&&d.ref_velocity==0,"clear did not cancel reference");
 std::cout<<"{\"status\":\"PASS\",\"checks\":"<<checks<<",\"numeric_cases\":28,\"max_cycles\":"<<maxcycles<<"}\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<" checks="<<checks<<"\n";return 1;}}
