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
 m.mem[measurement_0]=llround(th*S);m.mem[measurement_1]=llround(w*S);m.mem[measurement_2]=llround(f*S);m.mem[measurement_3]=llround(v*S);m.mem[applied]=j?m.mem[output]:0;m.mem[kick_sign]=S;m.mem[progress_measurement_valid]=(j%31>=3?S:0);
 if(j==220)m.mem[attempt]=11999*S;
 if(j==110||j==179)m.mem[resume_elapsed]=(j==110?32:599)*S;
 // Load the entire mutable state before this transaction; comparison checks every written word.
 for(int k: {measurement_0,measurement_1,measurement_2,measurement_3,applied,kick_sign,progress_measurement_valid})put(MAP[k],m.mem[k]);
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
// New zone transition branch: both directions, captured/provisional/initial/inside.
for(int direction: {-1,1})for(int kind=0;kind<4;kind++){
 using namespace hybrid_fixed;init();Machine m;
 m.mem[mode]=(kind<2?2:1)*S;m.mem[ever]=(kind==3?0:1)*S;
 m.mem[measurement_0]=direction*S;m.mem[measurement_1]=direction*S;
 m.mem[measurement_2]=direction*S*(kind==1?5:15)/10;
 m.mem[origin]=direction*S*14/10;m.mem[age]=1000*S;m.mem[attempt]=3000*S;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],m.mem[k]);
 m.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 20;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=m.mem[k])return 21;
 bool activate=kind==0||kind==2||kind==3;
 if(bool(m.mem[recovery_zone])!=activate)return 22;
 if(activate && !(m.mem[recovery_lo]<=m.mem[measurement_2] && m.mem[recovery_hi]>=m.mem[measurement_2]))return 23;
 if(kind<2 && m.mem[attempt]!=0)return 24;
 if(kind==3 && (m.mem[recovery_energy]!=S || m.mem[attempt]!=3001*S))return 25;
}
std::cout<<"PASS outside_zone_transition_cases=8 both_directions initial_gate_and_withdraw_checked\n";
// R32: a migrated pump corridor translates with phi; actual RTL must match the
// integer oracle and preserve acc/vreq/desired/output/sumu/pump in that frame.
for(int td:{-1,1})for(int wd:{-1,1})for(int vd:{-1,1})for(int shift_sign:{-1,1}){
 using namespace hybrid_fixed;int64_t expected[6]={};const int indices[]={acc,vreq,desired,output,sumu,pump};
 for(int shifted=0;shifted<2;shifted++){
  init();Machine q;const int64_t shift=shifted?shift_sign*3*S:0;
  q.mem[measurement_0]=td*S;q.mem[measurement_1]=wd*8*S;q.mem[measurement_2]=S/2+shift;q.mem[measurement_3]=vd*S*2/5;
  q.mem[counter]=S;q.mem[tick]=S;q.mem[age]=200*S;q.mem[attempt]=200*S;q.mem[startup_kick]=0;
  // This translated recovery fixture starts after first-transaction admission.
  // The common startup plan has already withdrawn; it must not be re-created.
  q.mem[phase_active]=0;q.mem[phase_enabled]=0;
  q.mem[recovery_zone]=S;q.mem[recovery_lo]=llround(-1.09756*S)+shift;q.mem[recovery_hi]=llround(1.09756*S)+shift;
  q.mem[origin]=shift;q.mem[vreq]=q.mem[measurement_3];
  for(int mi=0;mi<4;mi++)for(int j=0;j<4;j++)q.mem[pred_0+4*mi+j]=q.mem[measurement_0+j];
  for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],q.mem[k]);q.run();
  t.start=1;clk();t.start=0;int cycles=0;while(!t.done&&cycles++<50000)clk();if(!t.done||t.fault)return 61;
  maximum=std::max(maximum,(unsigned)cycles);
  for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=q.mem[k])return 62;
  for(int j=0;j<6;j++){if(!shifted)expected[j]=q.mem[indices[j]];else if(q.mem[indices[j]]!=expected[j])return 63;}
 }
}
std::cout<<"PASS R32 migrated_center_translation_pairs=16 actual_RTL_transactions=32 all_mutable_words_matched\n";
// R27 provisional350 and confirmed500ms permission; same max350 and12s timeout.
for(int direction:{-1,1})for(int kind=0;kind<6;kind++){
 using namespace hybrid_fixed;init();Machine m;m.mem[mode]=((kind==2||kind>=4)?2:1)*S;m.mem[ever]=(kind==1?0:1)*S;
 m.mem[age]=(kind==5?500:kind==4?498:kind==3?198:228)*S;m.mem[measurement_0]=direction*S*2/100;m.mem[measurement_1]=-direction*S*39/100;m.mem[measurement_3]=direction*S*87/100;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],m.mem[k]);m.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 30;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=m.mem[k])return 31;
 bool high=kind!=5; if((m.mem[cap]>S*349/1000)!=high)return 32;
 if(m.mem[cap]>S*351/1000)return 33;
}
for(int prior_mode:{0,1}){
 using namespace hybrid_fixed;init();Machine m;m.mem[mode]=prior_mode*S;m.mem[ever]=S;m.mem[attempt]=11999*S;m.mem[measurement_0]=S;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],m.mem[k]);m.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 34;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=m.mem[k])return 35;
 if(m.mem[fault]!=5*S||m.mem[output]!=0)return 36;
}
std::cout<<"PASS cap_gate_cases=12 timeout12s_cases=2 max35_unchanged=1\n";
// Real full-half plateau below the near-top region, both directions. R34
// retains measured stall eligibility during recovery for its turn reflection;
// changing the first-SWING energy target remains exclusive to first SWING.
for(int direction:{-1,1})for(int kind=0;kind<6;kind++){
 init();Machine q;q.mem[measurement_0]=llround(direction*1.7*S);q.mem[measurement_1]=direction*S*3/10;
 q.mem[progress_direction]=-direction*S;q.mem[progress_half_valid]=(kind==2?0:S);q.mem[progress_previous_valid]=S;
 q.mem[progress_measurement_valid]=(kind==3?0:S);q.mem[ever]=(kind==4?S:0);
 q.mem[progress_previous_peak]=llround((cos(1.7)+(kind==1?-.10:.01))*S);
 q.mem[progress_target]=llround((kind==5?1.39:1.1)*S);q.mem[progress_eligible]=0;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],q.mem[k]);
 q.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 40;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=q.mem[k])return 41;
 const bool change=kind==0||kind==5;
 const bool eligible=change||kind==4;
 if(bool(q.mem[progress_eligible])!=eligible){std::cerr<<"plateau fixture direction="<<direction<<" kind="<<kind<<" eligible="<<q.mem[progress_eligible]<<" expected="<<eligible<<" target="<<q.mem[progress_target]<<"\n";return 42;}
 double expected=kind==5?1.4:change?1.2:1.1;
 if(fabs(q.mem[progress_target]/double(S)-expected)>1e-6)return 43;
 if(llabs(q.mem[output])>llround(.351*S))return 44;
}
std::cout<<"PASS R30 measured_stall=12 low_peak/growth/partial/gap/recovery/cap both directions; every mutable word matched RTL\n";
// R30 phase lead: causal mixture extremes and intermediate weight, both directions.
for(int direction:{-1,1})for(int velocity_sign:{-1,1})for(int slow_percent:{0,50,100}){
 init();Machine q;q.mem[counter]=S;q.mem[tick]=S;q.mem[age]=200*S;
 const double th=direction*1.55,om=velocity_sign*6.,slow=slow_percent*.01;
 q.mem[measurement_0]=llround(th*S);q.mem[measurement_1]=llround(om*S);
 q.mem[weights_0]=llround((1-slow)*S);q.mem[weights_1]=llround(slow*S);q.mem[weights_2]=0;q.mem[weights_3]=0;
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],q.mem[k]);
 q.run();t.start=1;clk();t.start=0;int c=0;while(!t.done&&c++<50000)clk();if(!t.done||t.fault)return 45;
 maximum=std::max(maximum,(unsigned)c);
 for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE&&get(MAP[k])!=q.mem[k])return 46;
 double expected=om*cos(th+.02*slow*om);
 if(fabs(q.mem[lead_phase]/double(S)-expected)>.002)return 47;
 if((q.mem[lead_phase]>0)!=(expected>0))return 48;
 if(llabs(q.mem[output])>llround(.351*S))return 49;
}
std::cout<<"PASS R30 phase_lead=12 fast/slow/mixed, signed angular crossing, all mutable words matched RTL\n";
// Cancel and reset throughout a worst populated transaction. No stale completion allowed.
for(int cut: {0,1,2,3,5,20,50,100,200,500,1000,3000,6000,12000,20000,30000,40000}){
 init();t.start=1;clk();t.start=0;for(int j=0;j<cut;j++)clk();t.cancel=1;clk();t.cancel=0;for(int j=0;j<5;j++)clk();if(t.busy||t.done||t.ready||t.fault)return 4;
}
std::cout<<"PASS transactions=240 mutable_word_checks="<<n<<" maximum_cycles="<<maximum<<" cancel_points=17 late_capture_assertions=2\n";
}catch(...){std::cerr<<"exception\n";return 5;}}
