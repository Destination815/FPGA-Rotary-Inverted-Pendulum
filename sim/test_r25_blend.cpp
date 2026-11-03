// Explicit hold=false withdrawal contract. This is not the R28 production
// BRAKE selection policy (covered by test_r26_blend.cpp).
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
const int phases[][2]={{1,250},{2,210},{1,5},{2,70},{1,20},{2,50}};
for(auto&phase:phases){for(int j=0;j<phase[1];j++,k++){
 h.mode=phase[0];int th=std::lround(450*std::sin(k*.015)),ph=std::lround(210*std::sin(k*.009));int fallback=std::clamp(prev+(k%3-1)*4,-150,150);int ou=h.obs.tick(th,ph,prev,350,true,0,0,-.5,true,h.mode==1,interval);int mixed=h.blend_output(ou,fallback,h.mode==2);int selected=h.mode==2?mixed:fallback;
 t.motion_assist_permission=0;t.automatic_policy=1;t.automatic_brake=h.mode==1;t.automatic_hold=h.mode==2;t.fallback_duty=fallback;t.theta=th;t.phi=ph;t.reference_offset=0;t.reference_velocity=0;t.previous_duty=prev;t.prediction_duty=interval;t.limit_permille=350;t.start=1;clockit(t);t.start=0;int cy=0;while(!t.done&&cy<500){clockit(t);cy++;}maxcy=std::max(cy,maxcy);auto*r=t.rootp;
 if(cy>=500||t.fault)throw std::runtime_error("RTL fault/deadline");
 if(int32_t(r->r10_observer_core__DOT__delivered)!=ou)throw std::runtime_error("observer result mismatch");
 if(int64_t(r->r10_observer_core__DOT__blend)!=h.blendq)throw std::runtime_error("blend mismatch");
 int selected_rtl=h.mode==2?int32_t(t.duty):fallback;if(selected!=selected_rtl)throw std::runtime_error("committed selection mismatch");
 if(h.mode==1){if(selected!=fallback)throw std::runtime_error("BRAKE selected shadow output");if(h.blendq>0)withdraw++;}
 f<<k<<','<<h.mode<<','<<h.obs.age<<','<<h.blendq<<','<<r->r10_observer_core__DOT__blend<<','<<ou<<','<<int32_t(t.duty)<<','<<selected<<','<<fallback<<'\n';interval=prev;prev=selected;clockit(t);
}}
if(withdraw!=20)throw std::runtime_error("fixture did not cover partial retraction");
std::cout<<"PASS transactions="<<k<<" maxcycles="<<maxcy<<" nonzero_BRAKE_blend_samples="<<withdraw<<" short_BRAKE_5ms_then_recapture=PASS long_BRAKE_20ms_to_zero=PASS\n";
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
