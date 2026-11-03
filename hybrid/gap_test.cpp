#include "Vr01_gap_bridge.h"
#include "verilated.h"
#include <iostream>
#include <cstdint>
static VerilatedContext c;static Vr01_gap_bridge t{&c};void clk(){t.clk=0;t.eval();c.timeInc(1);t.clk=1;t.eval();c.timeInc(1);}
int main(){t.rst_n=0;clk();t.rst_n=1;clk();int64_t a[16];int n=0;while(std::cin>>a[0]){for(int i=1;i<16;i++)std::cin>>a[i];if(a[0]){t.clear=1;clk();t.clear=0;clk();}t.ab_position=a[1];t.ab_velocity=a[2];t.current_arm_velocity=a[3];t.previous_arm_velocity=a[4];t.adc_raw=a[5];t.zero_q4=a[6];t.raw_wrapped=a[7];t.ab_initialized=a[8];t.gap_allow=a[9];t.start=1;clk();t.start=0;int k=0;while(!t.done&&k++<180)clk();int64_t got[]={t.use_gap,t.gap_active,t.measurement_ok,(int32_t)t.position_out,(int32_t)t.velocity_out,t.gap_ms};for(int i=0;i<6;i++)if(got[i]!=a[10+i]){std::cerr<<"FAIL "<<n<<" field "<<i<<" "<<got[i]<<" "<<a[10+i]<<"\n";return 1;}if(k>=180)return 2;clk();n++;}std::cout<<"PASS gap fixed/RTL transactions="<<n<<"\n";}
