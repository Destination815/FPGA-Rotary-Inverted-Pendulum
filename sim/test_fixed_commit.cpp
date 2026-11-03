#include "Vr01_control_harness.h"
#include "Vr01_control_harness___024root.h"
#include "verilated.h"
#include <iostream>
#include <cassert>
static VerilatedContext ctx;static Vr01_control_harness t{&ctx};
void clk(){t.clk=0;t.eval();ctx.timeInc(1);t.clk=1;t.eval();ctx.timeInc(1);}
void reset(bool manual=false){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.theta=manual?20:205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=manual?80:820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;t.begin_swing=!manual;t.begin_capture=manual;clk();t.begin_swing=0;t.begin_capture=0;clk();clk();clk();}
int tick(bool automatic=true,int idle=64){int before=t.duty;t.step=1;clk();t.step=0;int commit=t.duty,cap=t.active_limit,c=0;do{clk();if(automatic&&t.duty!=commit&&!t.fault)throw 1;if(automatic&&t.active_limit!=cap&&!t.fault)throw 13;if(++c>49000)throw 2;}while(!t.done);for(int i=0;i<idle;i++){clk();if(automatic&&t.duty!=commit&&!t.fault)throw 3;if(automatic&&t.active_limit!=cap&&!t.fault)throw 14;}return c;}
int main(){try{int tests=0;
reset();for(int j=0;j<100;j++)tick();if(!t.duty)throw 4;tests++;
// STOP/disarm/reset during all compute phases and after ready-to-commit.
for(int cut:{0,1,2,10,100,600,1000,5000,15000,22000,26000})for(int how=0;how<3;how++){
 reset();for(int j=0;j<3;j++)tick();t.step=1;clk();t.step=0;for(int i=0;i<cut;i++)clk();if(how==0)t.stop=1;if(how==1)t.armed=0;if(how==2)t.rst_n=0;clk();if(t.duty)throw 5;t.stop=0;for(int j=0;j<24000;j++){clk();if(t.duty)throw 6;}t.step=1;clk();t.step=0;if(t.duty)throw 7;tests++;
}
// Cached nonzero output cannot cross a stopped session into the next start.
reset();for(int j=0;j<4;j++)tick();t.stop=1;clk();t.stop=0;clk();t.begin_swing=1;clk();t.begin_swing=0;clk();t.step=1;clk();t.step=0;if(t.duty)throw 8;tests++;
// Next raw frame before compute finishes cancels output and faults on overrun.
reset();tick();t.step=1;clk();t.step=0;for(int j=0;j<100;j++)clk();t.step=1;clk();t.step=0;if(t.duty)throw 9;for(int j=0;j<3;j++)clk();if(t.fault!=3||t.duty)throw 10;tests++;
// Current measurement fault still bypasses pending commit at the decision edge.
reset();for(int j=0;j<3;j++)tick();t.measurement_ok=0;tick();if(t.fault!=2||t.duty)throw 11;tests++;
// Manual result must still commit at done, not wait for an extra frame.
reset(true);bool found=false;for(int j=0;j<40;j++){int old=t.duty;tick(false);if(t.duty!=old)found=true;}if(!found)throw 12;tests++;
reset();tick();t.theta=0;t.filtered=0;bool hi=false,lo=false;for(int j=0;j<2300;j++){tick();hi|=t.active_limit==350;lo|=t.active_limit==150;if((int32_t)t.duty>(int32_t)t.active_limit||(int32_t)t.duty<-(int32_t)t.active_limit)throw 15;}if(!hi||!lo||t.fault)throw 16;tests++;
// R21: accepted absolute target survives a rescue; reseed must not consume it.
reset();tick();t.theta=0;t.filtered=0;for(int j=0;j<2300;j++)tick();
t.target=102944;t.target_valid=1;clk();t.target_valid=0;for(int j=0;j<10;j++)tick();
if(t.rootp->r01_control_harness__DOT__control__DOT__automatic_goal!=1)throw 17;
t.theta=62000;t.filtered=248000;tick();t.theta=0;t.filtered=0;t.phi=65536;
for(int j=0;j<2300;j++)tick();
if(t.fault||t.rootp->r01_control_harness__DOT__control__DOT__pending_target||t.rootp->r01_control_harness__DOT__control__DOT__references__DOT__goal!=411776)throw 18;
t.stop=1;clk();if(t.duty||t.rootp->r01_control_harness__DOT__control__DOT__automatic_goal)throw 19;tests+=3;
std::cout<<"PASS fixed commit directional checks="<<tests<<"\n";
}catch(int n){std::cerr<<"FAIL "<<n<<" state="<<(int)t.state<<" fault="<<(int)t.fault<<" duty="<<(int32_t)t.duty<<"\n";return n;}}
