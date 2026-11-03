#include "Vr29_handoff_guard.h"
#include "verilated.h"
#include "r29_handoff_contract.hpp"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

struct Input {
 bool rst_n=true,clear=false,stop=false,start=false,active=true,valid=true;
 int32_t theta=0,omega=0,velocity=0;
 int16_t observer=123,fallback=-234;
};

// Independent arithmetic oracle: widen absolute values and products instead of
// reproducing the RTL's sign-only comparisons. All state changes are transactional.
struct Gold {
 bool guard=false,pending=false,pulse=false,done=false,reblend=false;
 unsigned opposed=0,release=0,compatible=0,rescue=0;
 int16_t duty=0,slot=0;
 void edge(const Input& x) {
  if(!x.rst_n||x.clear||x.stop) {
   *this=Gold{};return;
  }
  done=pending;reblend=pending&&pulse;if(pending)duty=slot;
  pending=x.start;pulse=false;
  if(!x.active){guard=false;opposed=release=compatible=rescue=0;}
  if(!x.start)return;
  slot=x.observer;
  if(!x.active)return;
  if(!x.valid) {
   opposed=release=compatible=rescue=0;if(guard)slot=x.fallback;return;
  }
  if(guard) {
   slot=x.fallback;
   bool quiet=std::llabs(int64_t(x.theta))<6863 &&
              std::llabs(int64_t(x.omega))<52429 &&
              std::llabs(int64_t(x.velocity))<78643;
   release=quiet?release+1:0;
   bool inward=int64_t(x.theta)*x.omega<=0;
   bool agree=int64_t(x.observer)*x.fallback>0;
   bool ready=rescue>=500&&std::llabs(int64_t(x.theta))<11438&&std::llabs(int64_t(x.omega))<262144&&
              std::llabs(int64_t(x.velocity))<393216&&inward&&agree&&
              std::llabs(int64_t(x.observer)-x.fallback)<=20;
   rescue=std::min(500u,rescue+1);
   compatible=ready?compatible+1:0;
   if(release==100||compatible==20){guard=false;opposed=release=compatible=rescue=0;pulse=true;}
  } else {
   release=compatible=rescue=0;
   bool conflict=std::llabs(int64_t(x.theta))>13726 &&
                 int64_t(x.theta)*x.omega>0 &&
                 int64_t(x.theta)*x.observer<0;
   opposed=conflict?opposed+1:0;
   if(opposed==20){guard=true;slot=x.fallback;}
  }
 }
};

struct Bench {
 Vr29_handoff_guard rtl;
 Gold gold;
 RescueHandoff contract;
 uint64_t clocks=0,transactions=0,completions=0,triggers=0,releases=0,cancels=0;
 void tick(Input x,const std::string& where) {
  bool was_guard=gold.guard,was_pending=gold.pending;
  rtl.early_trigger=0;rtl.clk=0;rtl.rst_n=x.rst_n;rtl.clear=x.clear;rtl.stop=x.stop;
  rtl.start=x.start;rtl.active=x.active;rtl.valid=x.valid;
  rtl.theta=uint32_t(x.theta);rtl.omega=uint32_t(x.omega);rtl.velocity=uint32_t(x.velocity);
  rtl.observer_duty=uint16_t(x.observer);rtl.fallback_duty=uint16_t(x.fallback);
  rtl.eval();gold.edge(x);
  if(!x.rst_n||x.clear||x.stop||!x.active)contract.reset();
  if(x.start&&x.rst_n&&!x.clear&&!x.stop){
   int selected=contract.tick(2,x.theta,x.omega,x.velocity,x.active,x.valid,x.observer,x.fallback);
   if(selected!=gold.slot||contract.latched!=gold.guard||contract.opposed_ticks!=int(gold.opposed)||contract.stable_ticks!=int(gold.release)||contract.compatible_ticks!=int(gold.compatible)||contract.rescue_ticks!=int(gold.rescue)||contract.released!=gold.pulse)
    throw std::runtime_error("C++ production contract disagrees with independent oracle");
  }
  rtl.clk=1;rtl.eval();++clocks;
  transactions+=x.start&&x.rst_n&&!x.clear&&!x.stop;
  completions+=gold.done;triggers+=!was_guard&&gold.guard;releases+=gold.reblend;
  cancels+=was_pending&&(!x.rst_n||x.clear||x.stop);
  if(bool(rtl.done)!=gold.done||bool(rtl.reblend)!=gold.reblend||
     bool(rtl.guard_active)!=gold.guard||rtl.opposed_count!=gold.opposed||
     rtl.release_count!=gold.release||int16_t(rtl.duty)!=gold.duty) {
   std::cerr<<"Mismatch "<<where<<" at edge "<<clocks
    <<" RTL done/reblend/guard/counts/duty="<<unsigned(rtl.done)<<"/"<<unsigned(rtl.reblend)
    <<"/"<<unsigned(rtl.guard_active)<<"/"<<unsigned(rtl.opposed_count)
    <<"/"<<unsigned(rtl.release_count)<<"/"<<int16_t(rtl.duty)
    <<" gold="<<gold.done<<"/"<<gold.reblend<<"/"<<gold.guard<<"/"<<gold.opposed
    <<"/"<<gold.release<<"/"<<gold.duty<<"\n";
   throw std::runtime_error("RTL/gold disagreement");
  }
 }
 void reset(){Input x;x.rst_n=false;tick(x,"reset");}
 void transaction(Input x,const std::string& where) {
  x.start=true;tick(x,where+" accept");
  x.start=false;tick(x,where+" complete");
  if(!rtl.done)throw std::runtime_error("transaction must complete one edge after start");
 }
 void check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
 void latch(int sign=1,bool mature=true) {
  Input x;x.theta=sign*13727;x.omega=sign;x.observer=int16_t(-sign*150);x.fallback=int16_t(sign*350);
  for(int i=0;i<20;i++)transaction(x,"latch");
  check(rtl.guard_active,"20 transactions must latch");
  if(mature)for(int i=0;i<500;i++)transaction(x,"bounded rescue dwell");
 }
};

int main(int argc,char**argv) {
 Verilated::commandArgs(argc,argv);
 try {
  Bench b;
  // Exact opposition boundaries, zeros, and full signed limits.
  for(int sign:{-1,1}) {
   b.reset();Input x;x.theta=sign*13727;x.omega=sign;
   x.observer=int16_t(-sign*150);x.fallback=int16_t(sign*350);
   for(int i=1;i<=19;i++){b.transaction(x,"first 19");b.check(!b.rtl.guard_active,"early rescue");b.check(int16_t(b.rtl.duty)==x.observer,"early fallback");}
   b.transaction(x,"20th");b.check(b.rtl.guard_active&&int16_t(b.rtl.duty)==x.fallback,"trigger transaction must select fallback");
   x.theta=0;x.omega=0;x.velocity=0;
   for(int i=1;i<=99;i++){b.transaction(x,"first 99 quiet");b.check(b.rtl.guard_active&&!b.rtl.reblend,"early release");}
   b.transaction(x,"100th quiet");
   b.check(!b.rtl.guard_active&&b.rtl.reblend&&int16_t(b.rtl.duty)==x.fallback,"release must retain fallback and pulse reblend");
   b.transaction(x,"post release");b.check(!b.rtl.reblend&&int16_t(b.rtl.duty)==x.observer,"next transaction returns observer");
  }
  for(int sign:{-1,1}) {
   for(int kind=0;kind<5;kind++) {
    b.reset();Input x;x.theta=sign*13727;x.omega=sign;x.observer=int16_t(-sign);
    for(int i=0;i<19;i++)b.transaction(x,"pre bad opposition");
    Input bad=x;
    if(kind==0)bad.theta=sign*11438;
    if(kind==1)bad.omega=0;
    if(kind==2)bad.omega=-sign;
    if(kind==3)bad.observer=0;
    if(kind==4)bad.observer=int16_t(sign);
    b.transaction(bad,"opposition boundary");b.check(b.rtl.opposed_count==0&&!b.rtl.guard_active,"strict trigger boundary must clear run");
    b.transaction(x,"restart run");b.check(b.rtl.opposed_count==1,"run restarts at one");
   }
   for(int axis=0;axis<3;axis++) {
    b.reset();b.latch(sign);Input quiet;
    for(int i=0;i<99;i++)b.transaction(quiet,"quiet 99 before boundary");
    Input bad=quiet;
    if(axis==0)bad.theta=sign*6863;
    if(axis==1)bad.omega=sign*52429;
    if(axis==2)bad.velocity=sign*78643;
    b.transaction(bad,"strict release boundary");
    b.check(b.rtl.guard_active&&b.rtl.release_count==0,"equality cannot release");
    Input good;
    good.theta=sign*6862;good.omega=sign*52428;good.velocity=sign*78642;
    for(int i=0;i<100;i++)b.transaction(good,"inside all release bounds");
    b.check(!b.rtl.guard_active&&b.rtl.reblend,"strictly inside bounds must release");
   }
  }
  // Invalid prediction breaks both consecutive runs, but cannot clear rescue.
  b.reset();Input opposed;opposed.theta=20000;opposed.omega=100;opposed.observer=-150;
  for(int i=0;i<19;i++)b.transaction(opposed,"pre invalid");
  Input invalid=opposed;invalid.valid=false;b.transaction(invalid,"invalid opposition");
  b.check(b.rtl.opposed_count==0&&!b.rtl.guard_active,"prediction cannot latch");
  b.latch();Input quiet;
  for(int i=0;i<99;i++)b.transaction(quiet,"pre invalid release");
  invalid=quiet;invalid.valid=false;b.transaction(invalid,"invalid release");
  b.check(b.rtl.guard_active&&b.rtl.release_count==0&&b.rtl.opposed_count==0,"prediction resets counts only");
  for(int i=0;i<99;i++)b.transaction(quiet,"renewed release 99");
  b.check(b.rtl.guard_active,"release cannot include predicted samples");
  b.transaction(quiet,"renewed release 100");b.check(b.rtl.reblend,"renewed real 100 releases");
  // Idle clocks never learn. Inactive clocks clear state, but inactive starts
  // still pass observer through and complete, including back-to-back traffic.
  b.reset();b.latch();Input idle;idle.active=false;b.tick(idle,"inactive idle");
  b.check(!b.rtl.guard_active,"inactive clears without start");
  for(int i=0;i<40;i++){idle.start=true;idle.observer=int16_t(i-20);b.tick(idle,"inactive pipeline");}
  idle.start=false;b.tick(idle,"inactive drain");b.tick(idle,"inactive idle pulse clear");
  b.check(!b.rtl.done&&!b.rtl.reblend,"done/reblend must be pulses");
  b.reset();opposed.start=false;
  for(int i=0;i<150;i++)b.tick(opposed,"idle cannot count");
  b.check(b.rtl.opposed_count==0,"only start may count");
  // Reset/clear/STOP override starts and cancel already computed results,
  // including the release pulse awaiting its completion edge.
  for(int reason=0;reason<3;reason++) {
   b.reset();b.latch();Input x;for(int i=0;i<99;i++)b.transaction(x,"cancel preparation");
   x.start=true;b.tick(x,"pending release");
   if(reason==0)x.rst_n=false;if(reason==1)x.clear=true;if(reason==2)x.stop=true;
   b.tick(x,"cancel beats completion/start");
   b.check(!b.rtl.done&&!b.rtl.reblend&&!b.rtl.guard_active&&int16_t(b.rtl.duty)==0,"cancel must discard response");
   Input resume;b.tick(resume,"after cancel");b.check(!b.rtl.done,"no stale done after cancel");
  }
  // Force sign/absolute-value corner cases and prove data are selected unchanged,
  // even outside normal +/-350 duties: this block must not hide a new limiter.
  for(int32_t t:{INT32_MIN,INT32_MAX,-13726,13726,0}) {
   b.reset();Input x;x.theta=t;x.omega=t;x.observer=t<0?INT16_MAX:INT16_MIN;x.fallback=INT16_MIN;
   for(int i=0;i<25;i++)b.transaction(x,"signed limits");
  }
  for(int sign:{-1,1}){
   b.reset();b.latch(sign,false);Input x;
   x.theta=sign*10000;x.omega=-sign*150000;x.velocity=sign*300000;
   x.observer=sign*230;x.fallback=sign*250;
   for(int i=0;i<519;i++)b.transaction(x,"dwell plus compatible 19");
   b.check(b.rtl.guard_active&&!b.rtl.reblend,"new path must wait 500 valid dwell plus20");
   b.transaction(x,"dwell plus compatible20");b.check(b.rtl.reblend,"new path releases after bounded dwell and compatibility");
  }
  // R31: moving inward with compatible mature commands can hand back before
  // reaching the old near-zero-speed threshold. A prediction or conflict
  // resets the 20-frame qualification; no elapsed-time-only release.
  for(int sign:{-1,1}) {
   b.reset();b.latch(sign);Input x;
   x.theta=sign*10000;x.omega=-sign*150000;x.velocity=sign*300000;
   x.observer=sign*230;x.fallback=sign*250;
   for(int i=0;i<19;i++)b.transaction(x,"compatible 19");
   b.check(b.rtl.guard_active&&!b.rtl.reblend,"compatible release early");
   b.transaction(x,"compatible 20");
   b.check(!b.rtl.guard_active&&b.rtl.reblend&&int16_t(b.rtl.duty)==x.fallback,"compatible release retains fallback");
   for(int kind=0;kind<9;kind++) {
    b.reset();b.latch(sign);
    for(int i=0;i<19;i++)b.transaction(x,"before incompatible");
    Input bad=x;
    if(kind==0)bad.theta=sign*11438;
    if(kind==1)bad.omega=-sign*262144;
    if(kind==2)bad.velocity=sign*393216;
    if(kind==3)bad.omega=sign*150000;
    if(kind==4)bad.observer=-sign*200;
    if(kind==5)bad.observer=sign*229;
    if(kind==6)bad.valid=false;
    if(kind==7)bad.observer=0;
    if(kind==8){bad.observer=0;bad.fallback=0;}
    b.transaction(bad,"incompatible breaks qualification");
    if(kind==6)for(int i=0;i<500;i++)b.transaction(x,"renew valid rescue dwell");
    for(int i=0;i<19;i++)b.transaction(x,"renewed compatible 19");
    b.check(b.rtl.guard_active,"incompatible/predicted frame must break run");
    b.transaction(x,"renewed compatible 20");b.check(b.rtl.reblend,"new 20 releases");
   }
  }
  // Fixed-seed random bursts exercise latching/release, interruptions, bubbles,
  // input changes between accept/complete, and consecutive start transactions.
  std::mt19937 rng(20261020);
  for(int episode=0;episode<600;episode++) {
   Input x;int sign=(rng()&1)?1:-1;unsigned kind=rng()%8;
   unsigned n=1+rng()%180;
   for(unsigned k=0;k<n;k++) {
    x.start=(rng()%10)!=0;x.active=(rng()%80)!=0;x.valid=(rng()%90)!=0;
    x.clear=(rng()%600)==0;x.stop=(rng()%700)==0;x.rst_n=(rng()%900)!=0;
    x.observer=int16_t(int(rng()%701)-350);x.fallback=int16_t(int(rng()%701)-350);
    x.theta=int32_t(rng());x.omega=int32_t(rng());x.velocity=int32_t(rng());
    if(kind<=2){x.theta=sign*(13727+int(rng()%10000));x.omega=sign*(1+int(rng()%1000));x.observer=int16_t(-sign*(1+rng()%350));}
    if(kind==3||kind==4){x.theta=int(rng()%13725)-6862;x.omega=int(rng()%104857)-52428;x.velocity=int(rng()%157285)-78642;}
    if(kind==5)x.valid=false;
    if(kind==6)x.active=false;
    if(kind==7){x.theta=sign*13726;x.omega=0;}
    b.tick(x,"random burst");
   }
  }
  Input drain;b.tick(drain,"drain");b.tick(drain,"drain clear");
  std::cout<<"{\"status\":\"PASS\",\"seed\":20261020,\"clock_edges\":"<<b.clocks
   <<",\"accepted_transactions\":"<<b.transactions<<",\"completed_transactions\":"<<b.completions
   <<",\"rescue_triggers\":"<<b.triggers<<",\"release_pulses\":"<<b.releases
   <<",\"cancelled_pending_transactions\":"<<b.cancels<<",\"mismatches\":0}\n";
 } catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
