#include "Vr01_control_harness.h"
#include "verilated.h"
#include <iostream>
#include <cassert>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
void clk(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void reset(){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=1;clk();t.begin_swing=0;clk();clk();clk();}
int tick(){t.step=1;clk();t.step=0;int c=0;do{clk();if(++c>49000)throw 1;}while(!t.done);if((int32_t)t.duty>(int32_t)t.active_limit||(int32_t)t.duty<-(int32_t)t.active_limit)throw 2;clk();return c;}
int main(){try{int tests=0,maximum=0;
for(int cut:{0,1,2,10,100,600,1000,5000,15000,22000})for(int how=0;how<3;how++){
 reset();t.step=1;clk();t.step=0;for(int i=0;i<cut;i++)clk();if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.rst_n=0;clk();if(t.duty!=0)throw 3;
 t.stop=0;for(int j=0;j<24000;j++){clk();if(t.duty!=0)throw 4;}tests++;
}
reset();maximum=std::max(maximum,tick());t.theta=0;t.filtered=0;
for(int j=0;j<2050;j++){maximum=std::max(maximum,tick());if(t.fault)throw 5;}
if(t.state!=3)throw 6;t.theta=70000;t.omega=100000;tick();if(t.state!=1||t.fault)throw 7;tests++;
t.measurement_ok=0;tick();if(t.fault!=2||t.duty)throw 8;tests++;
reset();t.phi=230000;tick();if(t.fault!=1||t.duty)throw 9;tests++;
reset();t.velocity=1400000;tick();if(t.fault!=6||t.duty)throw 10;tests++;
std::cout<<"PASS auto stop/disarm/reset phases="<<tests<<" maxcycles="<<maximum<<"\n";
}catch(int n){std::cerr<<"FAIL "<<n<<" state="<<(int)t.state<<" fault="<<(int)t.fault<<"\n";return n;}}
