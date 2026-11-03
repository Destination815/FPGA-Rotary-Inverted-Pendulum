#include "Vs23_compute.h"
#include "verilated.h"
#include "hybrid_fixed.hpp"
#include "layout.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
static VerilatedContext ctx;static Vs23_compute t{&ctx};
static void clk(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
static int64_t sx(uint64_t v){return (int64_t)(v<<16)>>16;}
static void put(int a,int64_t v){t.ext_write=1;t.ext_address=a;t.ext_data=v;clk();t.ext_write=0;}
static int64_t get(int a){t.ext_address=a;clk();clk();return sx(t.ext_read);}
static void init(){t.cancel=0;t.start=0;t.ext_write=0;t.initialize=1;clk();t.initialize=0;for(int i=0;!t.ready&&i<10000;i++)clk();if(!t.ready)throw 1;}
int main(){try{t.rst_n=0;clk();t.rst_n=1;init();hybrid_fixed::Machine m;using namespace hybrid_fixed;unsigned maximum=0,n=0;
for(int j=0;j<240;j++){
 // Discontinuous sensor fixtures exercise pump, entry, confirmed hold, loss, return, timeout.
 double th=(j<50?3.10-.025*j:j<100?.08*sin(j*.15):j<150?1.3:0.02*sin(j*.2));double w=j<50?-.8:j<100?.1:j<150?-4:.08;double f=.1*sin(j*.01),v=.05*cos(j*.01);
 m.mem[measurement_0]=llround(th*S);m.mem[measurement_1]=llround(w*S);m.mem[measurement_2]=llround(f*S);m.mem[measurement_3]=llround(v*S);m.mem[applied]=j?m.mem[output]:0;m.mem[kick_sign]=S;
 if(j==220)m.mem[attempt]=11999*S;
 if(j==110||j==179)m.mem[resume_elapsed]=(j==110?32:599)*S;
 // Load the entire mutable state before this transaction; comparison checks every written word.
 for(int k: {measurement_0,measurement_1,measurement_2,measurement_3,applied,kick_sign})put(MAP[k],m.mem[k]);
 if(j==220)put(MAP[attempt],m.mem[attempt]);
 put(MAP[resume_elapsed],m.mem[resume_elapsed]);
 m.run();t.start=1;clk();t.start=0;unsigned c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault){std::cerr<<"fault j="<<j<<" cycles="<<c<<"\n";return 2;}maximum=std::max(maximum,c);
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE){int64_t v=get(MAP[k]);if(v!=m.mem[k]){std::cerr<<"mismatch "<<j<<" addr="<<k<<" expect="<<m.mem[k]<<" actual="<<v<<"\n";return 3;}n++;}
 if(m.mem[hybrid_fixed::fault]){m=Machine();init();}
}
// R21: a late provisional catch must not retain its old position/integral/budget age.
for(int direction: {-1,1}){
 using namespace hybrid_fixed;init();Machine m;
 m.mem[mode]=S;m.mem[confirm]=29*S;m.mem[age]=2100*S;m.mem[origin]=-direction*S/50;
 m.mem[ih]=direction*S/20;m.mem[measurement_2]=direction*S*13/10;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],m.mem[k]);
 m.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 6;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=m.mem[k])return 7;
 if(m.mem[mode]!=2*S||m.mem[origin]!=m.mem[measurement_2]||m.mem[age]!=0||m.mem[ih]!=0||m.mem[cap]<S*349/1000)return 8;
}
// Cancel and reset throughout a worst populated transaction. No stale completion allowed.
for(int cut: {0,1,2,3,5,20,50,100,200,500,1000,3000,6000,12000}){
 init();t.start=1;clk();t.start=0;for(int j=0;j<cut;j++)clk();t.cancel=1;clk();t.cancel=0;for(int j=0;j<5;j++)clk();if(t.busy||t.done||t.ready||t.fault)return 4;
}
std::cout<<"PASS transactions=240 mutable_word_checks="<<n<<" maximum_cycles="<<maximum<<" cancel_points=14 late_capture_assertions=2\n";
}catch(...){std::cerr<<"exception\n";return 5;}}
