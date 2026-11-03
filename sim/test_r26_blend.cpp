#include "Vr10_observer_core.h"
#include "Vr10_observer_core___024root.h"
#include "verilated.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#define HYBRID_OBSERVER_ONLY
#include "hybrid_fixed.cpp"
static void clockit(Vr10_observer_core&t){t.clk=0;t.eval();t.clk=1;t.eval();}
int main(int argc,char**argv){try{VerilatedContext c;Vr10_observer_core t{&c};Hybrid h;t.rst_n=0;clockit(t);t.rst_n=1;clockit(t);int interval=0,prev=0,k=0,maxcy=0,withdraw=0;std::ofstream f(argv[1]);f<<"sample,mode,age,blend_cpp,blend_rtl,observer_duty,rtl_mixed,committed_selection,fallback\n";
// Verify the wrapper's explicit two-second stage boundary independently of
// observer math. Full controller frame-state selection is tested separately.
h.old=.123;h.prediction_previous=-71;
for(int mode:{0,1,2})for(int age:{1999,2000,2001}){
 h.mode=mode;h.cpu.mem[hybrid_fixed::age]=int64_t(age)*hybrid_fixed::S;
 int expected=mode==2&&age==2001?-71:123;
 if(h.observer_prediction_input()!=expected)throw std::runtime_error("wrapper prediction phase gate");
}
h.old=0;h.prediction_previous=0;
const int phases[][2]={{1,250},{2,210},{1,5},{2,70},{1,20},{2,50}};
for(auto&phase:phases){for(int j=0;j<phase[1];j++,k++){
 h.mode=phase[0];int th=std::lround(450*std::sin(k*.015)),ph=std::lround(210*std::sin(k*.009));int fallback=std::clamp(prev+(k%3-1)*4,-150,150);int ou=h.obs.tick(th,ph,prev,350,true,0,0,-.5,true,h.mode==1,interval);int selected=h.blend_output(ou,fallback);
 t.motion_assist_permission=0;t.automatic_policy=1;t.automatic_brake=(h.mode==1);t.automatic_hold=h.mode==1 || h.mode==2;t.fallback_duty=fallback;t.theta=th;t.phi=ph;t.reference_offset=0;t.reference_velocity=0;t.previous_duty=prev;t.prediction_duty=interval;t.limit_permille=350;t.start=1;clockit(t);t.start=0;int cy=0;while(!t.done&&cy<500){clockit(t);cy++;}maxcy=std::max(cy,maxcy);auto*r=t.rootp;
 if(cy>=500||t.fault)throw std::runtime_error("RTL fault/deadline");
 if(int32_t(r->r10_observer_core__DOT__delivered)!=ou)throw std::runtime_error("observer result mismatch");
 if(int64_t(r->r10_observer_core__DOT__blend)!=h.blendq)throw std::runtime_error("blend mismatch");
 int selected_rtl=(h.mode==1 || h.mode==2)?int32_t(t.duty):fallback;if(selected!=selected_rtl)throw std::runtime_error("committed selection mismatch");
 if(h.mode==1){if(selected!=int32_t(t.duty))throw std::runtime_error("BRAKE did not select observer output");if(h.blendq>0)withdraw++;}
 f<<k<<','<<h.mode<<','<<h.obs.age<<','<<h.blendq<<','<<r->r10_observer_core__DOT__blend<<','<<ou<<','<<int32_t(t.duty)<<','<<selected<<','<<fallback<<'\n';interval=prev;prev=selected;clockit(t);
}}
// Natural warm age200 begins at index199: 51 initial BRAKE +5+20 samples.
if(withdraw!=76)throw std::runtime_error("fixture did not cover BRAKE observer selection");
std::cout<<"PASS transactions="<<k<<" maxcycles="<<maxcy<<" active_BRAKE_blend_samples="<<withdraw<<" short_BRAKE_5ms_blend_continuity=PASS second_BRAKE_20ms_blend_continuity=PASS\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
