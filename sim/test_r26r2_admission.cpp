#include "Vr01_control_harness.h"
#include "verilated.h"
#include <stdexcept>
#include <iostream>
static VerilatedContext c;static Vr01_control_harness t{&c};
void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
void ck(bool x,const char*m){if(!x)throw std::runtime_error(m);}
void init(bool manual){t.rst_n=0;t.armed=0;t.step=0;t.stop=0;t.begin_swing=0;t.begin_capture=0;t.clear=0;t.raw_mode=0;t.target_valid=0;t.shuttle_valid=0;t.theta=manual?0:205000;t.omega=0;t.phi=0;t.velocity=0;t.filtered=manual?0:820000;t.measurement_ok=1;t.workspace_limit=217360;clk();t.rst_n=1;clk();t.armed=1;}
int main(){try{int tests=0;
for(int manual=0;manual<2;manual++)for(int collision=0;collision<6;collision++){
init(manual);for(int j=0;j<8;j++){t.begin_capture=manual && j==0;t.begin_swing=!manual && j==0;t.step=j==collision;clk();}t.step=0;for(int j=0;j<49000;j++)clk();ck(t.state==(manual?2:1),"start state");ck(!t.fault,"start fault");t.step=1;clk();t.step=0;int n=0;while(!t.done && ++n<49000)clk();ck(n<49000,"next frame deadline");ck(!t.fault,"frame fault");tests++;
}
for(int manual=0;manual<2;manual++)for(int stage=0;stage<5;stage++)for(int action=0;action<3;action++){
init(manual);for(int j=0;j<stage+1;j++){t.begin_capture=manual && j==0;t.begin_swing=!manual && j==0;clk();}t.begin_capture=0;t.begin_swing=0;if(action==0)t.stop=1;else if(action==1)t.armed=0;else t.rst_n=0;clk();ck(t.duty==0,"cancel output");if(action==0)t.stop=0;else if(action==1)t.armed=1;else t.rst_n=1;for(int j=0;j<8;j++){clk();ck(t.state==0 && t.duty==0,"stale start after cancel");}tests++;
}
std::cout<<"PASS "<<tests<<" admission: 12 request/sample collisions plus 30 STOP/disarm/reset stage checks; next frame deadline and stale-session cancellation\n";
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
