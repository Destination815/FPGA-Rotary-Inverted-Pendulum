#include "Vcomponent_top.h"
#include "verilated.h"
#include "r29_handoff_contract.hpp"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

struct X {
 bool rst=true,clear=false,stop=false,frame=false,manual=false,stationary=true;
 bool moving=false,observing=true,valid=true,start=false,publish=false,discard=false,gate=false,busy=false,new_request=false;
 int state=3,hold=2000,mode=2;
 int32_t command=350,cap=350,t=0,w=0,v=0;
 int16_t observer=100,fallback=230;
 bool mature()const{return ((state==3&&hold>=2000)||(state==2&&hold==1999))&&mode==2&&!manual&&stationary&&!moving;}
 bool hardcancel()const{return clear||manual||(state!=3&&state!=2)||mode!=2||moving||(!stationary&&!busy)||new_request;}
 bool active()const{return observing&&mode==2&&!manual;}
};
struct CounterGold {
 int n=0;bool fired=false,pending=false,inflight=false,prior=false;
 bool trigger(const X&x)const{return pending&&!inflight&&x.rst&&x.mature()&&!x.hardcancel()&&!x.stop&&!x.discard;}
 void edge(const X&x,int32_t command,int32_t cap,bool done){
  if(!x.rst||x.hardcancel()||x.stop||(x.frame&&!x.mature())){*this={};return;}
  const bool take=trigger(x);
  if(done&&inflight){pending=false;inflight=false;}
  else if(x.start&&take)inflight=true;
  if(x.frame){
   if(!fired){
    bool saturated=prior&&cap>0&&std::llabs(int64_t(command))>=cap;
    n=saturated?std::min(100,n+1):0;
    if(n==100){fired=true;pending=true;}
   }
   prior=x.mature();
  }
  if(x.discard){pending=false;inflight=false;}
 }
};
struct GuardGold {
 bool pending=false,pulse=false,done=false,reblend=false,guard=false;
 unsigned opposed=0,stable=0,compatible=0,rescue=0;
 int16_t duty=0,slot=0;
 void edge(const X&x,bool force){
  if(!x.rst||x.clear||x.stop){*this={};return;}
  done=pending;reblend=pending&&pulse;if(pending)duty=slot;
  pending=x.start;pulse=false;
  if(!x.active()){guard=false;opposed=stable=compatible=rescue=0;}
  if(!x.start)return;
  slot=x.observer;
  if(!x.active())return;
  if(force){guard=true;opposed=stable=compatible=rescue=0;}
  if(!x.valid){opposed=stable=compatible=rescue=0;if(guard)slot=x.fallback;return;}
  if(guard){
   slot=x.fallback;
   bool quiet=std::llabs(int64_t(x.t))<6863&&std::llabs(int64_t(x.w))<52429&&std::llabs(int64_t(x.v))<78643;
   bool ok=rescue>=500&&std::llabs(int64_t(x.t))<11438&&std::llabs(int64_t(x.w))<262144&&std::llabs(int64_t(x.v))<393216&&
           int64_t(x.t)*x.w<=0&&int64_t(x.observer)*x.fallback>0&&std::llabs(int64_t(x.observer)-x.fallback)<=20;
   stable=quiet?stable+1:0;compatible=ok?compatible+1:0;rescue=std::min(500u,rescue+1);
   if(stable==100||compatible==20){guard=false;opposed=stable=compatible=rescue=0;pulse=true;}
  }else{
   stable=compatible=rescue=0;
   bool conflict=std::llabs(int64_t(x.t))>13726&&int64_t(x.t)*x.w>0&&int64_t(x.t)*x.observer<0;
   opposed=conflict?opposed+1:0;
   if(opposed==20){guard=true;slot=x.fallback;}
  }
 }
};
struct Bench {
 Vcomponent_top t;CounterGold c;GuardGold g;RescueHandoff native;
 int32_t actual=0,cap=150;
 uint64_t clocks=0,checks=0,frames=0,transactions=0,forces=0,attempts=0,releases=0,bypass=0,cancel_cases=0,gate_pairs=0,gate_skips=0;
 bool must_legacy_match=true;
 void check(bool b,const std::string&why){++checks;if(!b)throw std::runtime_error(why+" edge="+std::to_string(clocks));}
 void edge(X x,const std::string&where=""){
  t.clk=0;t.rst_n=x.rst;t.clear=x.clear;t.stop=x.stop;t.frame_tick=x.frame;
  t.controller_state=x.state;t.hold_ticks=x.hold;t.swing_mode=x.mode;t.manual=x.manual;
  t.use_returning_gate=x.gate;
  t.ref_stationary=x.stationary;t.trajectory_running=x.moving;t.observing=x.observing;t.valid=x.valid;t.start=x.start;
  t.ref_busy=x.busy;t.new_request=x.new_request;
  t.publish=x.publish;t.discard_pending=x.discard;t.publish_command=uint32_t(x.command);t.publish_cap=uint32_t(x.cap);
  t.theta=uint32_t(x.t);t.omega=uint32_t(x.w);t.velocity=uint32_t(x.v);
  t.observer_duty=uint16_t(x.observer);t.fallback_duty=uint16_t(x.fallback);t.eval();
  const bool inward=(x.t>0&&x.w<0)||(x.t<0&&x.w>0);
  const uint64_t at=x.t<0?uint64_t(-int64_t(x.t)):uint64_t(x.t);
  const uint64_t aw=x.w<0?uint64_t(-int64_t(x.w)):uint64_t(x.w);
  const bool allowed=at!=0&&(!inward||2*at>aw);
  check(bool(t.returning_allowed)==allowed&&early_returning_gate(x.t,x.w)==allowed,where+" returning gate arithmetic");
  bool attempt=c.trigger(x)&&x.start&&x.rst&&!x.clear&&!x.stop;
  bool force=attempt&&(!x.gate||allowed);
  attempts+=attempt;
  if(attempt&&x.gate&&!allowed)++gate_skips;
  check(bool(t.early_trigger)==c.trigger(x),where+" pre trigger");
  if(force){++forces;must_legacy_match=false;}
  bool olddone=g.done;
  c.edge(x,actual,cap,olddone);g.edge(x,force);
  if(!x.rst||x.clear||x.stop||!x.active())native.reset();
  if(x.start&&x.rst&&!x.clear&&!x.stop){
   int selected=native.tick(x.mode,x.t,x.w,x.v,x.observing&&!x.manual,x.valid,x.observer,x.fallback,false,force);
   check(selected==g.slot&&native.latched==g.guard&&native.opposed_ticks==int(g.opposed)&&native.stable_ticks==int(g.stable)&&native.compatible_ticks==int(g.compatible)&&native.rescue_ticks==int(g.rescue)&&native.released==g.pulse,where+" production contract");
   if(force)check(!native.activated,where+" force must not fake activated");
   ++transactions;
  }
  if(!x.rst){actual=0;cap=150;}else if(x.publish){actual=x.command;cap=x.cap;}
  t.clk=1;t.eval();++clocks;frames+=x.frame;
  if(!x.rst||x.clear||x.stop)must_legacy_match=true;
  check(t.saturated_count==c.n&&bool(t.fired)==c.fired&&bool(t.pending)==c.pending&&bool(t.inflight)==c.inflight&&bool(t.prior_mature)==c.prior,where+" counter");
  check(int32_t(t.actual_command)==actual&&int32_t(t.actual_cap)==cap,where+" commit register");
  check(bool(t.done)==g.done&&bool(t.reblend)==g.reblend&&bool(t.guard_active)==g.guard&&int16_t(t.duty)==g.duty&&t.opposed_count==g.opposed&&t.release_count==g.stable&&t.compatible_count==g.compatible&&t.rescue_age==g.rescue,where+" guard");
  if(g.reblend)++releases;
  if(must_legacy_match){
   check(t.duty==t.legacy_duty&&t.done==t.legacy_done&&t.reblend==t.legacy_reblend&&t.guard_active==t.legacy_active,where+" early0 legacy exact");++bypass;
  }
  t.clk=0;t.eval();
 }
 void reset(){X x;x.rst=false;edge(x,"reset");x.rst=true;edge(x,"release reset");}
 void publish(int command,int limit=350){X x;x.publish=true;x.command=command;x.cap=limit;edge(x,"publish");}
 void frame(X x=X{}){x.frame=true;edge(x,"raw frame");x.frame=false;edge(x,"raw frame gap");}
 void tx(X x=X{}){x.start=true;edge(x,"accept");x.start=false;edge(x,"done");edge(x,"retire");}
 void prime(int command=350,int limit=350){reset();publish(command,limit);frame();for(int i=0;i<99;i++)frame();check(c.n==99&&!c.pending,"99 intervals");}
 void force(X x=X{}){prime();frame(x);check(c.n==100&&c.pending&&!c.inflight,"100 intervals pending");tx(x);}
};

int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv);
 try{
  Bench b;
  // Exactly100 preceding mature intervals; first mature frame contributes0.
  for(int sign:{-1,1})for(int cap:{150,350}){
   b.reset();b.publish(sign*cap,cap);b.frame();b.check(b.c.n==0,"first mature frame excludes previous domain");
   for(int i=1;i<=99;i++){b.frame();b.check(b.c.n==i&&!b.c.pending,"first99");}
   b.frame();b.check(b.c.n==100&&b.c.pending&&b.c.fired,"hundredth");
   for(int i=0;i<17;i++)b.edge(X{},"pending wait");b.check(b.c.pending,"pending survives no start");
   X x;x.start=true;b.edge(x,"force accept");b.check(b.c.pending&&b.c.inflight&&b.g.rescue==1&&b.g.stable==1,"pending not retired at start and first dwell1");
   b.edge(x,"consecutive start");b.check(b.c.pending&&b.c.inflight&&b.g.rescue==2,"force not repeated during inflight");
   x.start=false;b.edge(x,"done retire");b.check(!b.c.pending&&!b.c.inflight,"retire only done");b.edge(x);
   for(int i=0;i<98;i++)b.tx();b.check(!b.g.guard,"100 stable release");
   uint64_t f=b.forces;for(int i=0;i<400;i++){b.frame();b.tx();}b.check(b.forces==f&&b.c.fired,"no retrigger after original release");
  }
  // cap-1/zero break the run. Full signed limits are widened safely.
  for(int command:{149,-149,0,INT32_MIN,INT32_MAX}){
   b.prime(150,150);b.publish(command,150);b.frame();
   bool sat=std::llabs(int64_t(command))>=150;
   b.check(b.c.n==(sat?100:0),"magnitude and cap boundary");
  }
  // Same-edge publication must not contaminate the completed interval.
  b.prime(350,350);X x;x.frame=true;x.publish=true;x.command=0;x.cap=150;b.edge(x,"old saturated350 published0cap150");
  b.check(b.c.pending&&b.c.n==100,"old actual command/cap sampled");
  b.prime(350,350);b.publish(150,350);x.command=150;x.cap=150;b.edge(x,"old unsaturated350 publishedcap150");
  b.check(!b.c.pending&&b.c.n==0,"new cap must not be backdated");
  b.prime(350,350);x.command=-150;x.cap=150;b.edge(x,"old positive new negative");b.check(b.c.pending,"new sign ignored");
  // Reference computation busy is not a physical/reference-motion boundary.
  // Hold the raw-frame maturity decision throughout that transient.
  b.reset();b.publish(350);b.frame();for(int i=0;i<50;i++)b.frame();
  X computing;computing.stationary=false;computing.busy=true;
  for(int i=0;i<4320;i++)b.edge(computing,"static reference busy cannot reset count");
  b.check(b.c.n==50&&b.c.prior,"busy preserved learned intervals");
  b.frame();b.check(b.c.n==51,"first post-busy frame continues run");
  // A raw frame with unqualified maturity really does reset, unlike busy
  // between raw boundaries. New requests revoke pending immediately.
  b.frame(computing);b.check(!b.c.n&&!b.c.prior,"raw unqualified frame resets");
  b.prime();b.frame();X request;request.new_request=true;b.edge(request,"explicit request abort");
  b.check(!b.c.pending&&!b.c.fired,"new request cancels immediately");
  // Native post-run age2000 corresponds to raw CAPTURE1999 will-mature.
  b.reset();b.publish(350);X boundary;boundary.state=2;boundary.hold=1998;b.frame(boundary);
  boundary.hold=1999;b.frame(boundary);b.check(b.c.prior&&!b.c.n,"will-mature first frame count0");
  boundary.hold=2000;boundary.stationary=false;boundary.busy=true;
  b.edge(boundary,"will-mature compute interval");b.check(b.c.prior&&!b.c.n,"capture2000 midframe holds snapshot");
  boundary.state=3;boundary.stationary=true;boundary.busy=false;b.frame(boundary);
  b.check(b.c.n==1,"first interval after native age2000");
  // First force valid quiet/nonquiet/invalid, and already-latched reset.
  for(int kind=0;kind<3;kind++){
   X y;if(kind==1)y.t=20000;if(kind==2)y.valid=false;
   b.force(y);b.check(b.g.guard&&b.g.opposed==0&&b.g.compatible==0&&b.g.rescue==(kind==2?0u:1u)&&b.g.stable==(kind==0?1u:0u),"first force counters");
   b.check(int16_t(b.t.duty)==y.fallback&&!b.t.reblend,"force fallback no reblend");
  }
  b.reset();b.publish(350);X opposed;opposed.t=20000;opposed.w=100;opposed.observer=-150;
  for(int i=0;i<20;i++)b.tx(opposed);b.check(b.g.guard,"old opposed latch");
  b.frame();for(int i=0;i<100;i++)b.frame();b.tx();b.check(b.g.rescue==1&&b.g.stable==1&&b.g.opposed==0,"already latched force resets4");
  // First force rescue=1 means compatible20 releases on transaction520.
  X compat;compat.t=10000;compat.w=-150000;compat.v=300000;compat.observer=230;compat.fallback=250;
  b.force(compat);for(int i=0;i<518;i++)b.tx(compat);b.check(b.g.guard&&!b.t.reblend,"compatible19 at519");
  b.tx(compat);b.check(!b.g.guard&&b.t.reblend==0,"reblend pulse has retired");
  b.check(b.native.released,"native compatible20 release");b.tx(compat);b.check(int16_t(b.t.duty)==compat.observer,"next transaction observer");
  // Domain exit consumes the sole attempt; reentry cannot replay pending.
  b.prime();b.frame();X outside;outside.observing=false;outside.discard=true;
  b.edge(outside,"phase2 completed outside observer domain");
  b.check(b.c.fired&&!b.c.pending&&!b.c.inflight,"discard preserves spent attempt");
  uint64_t spent=b.forces;
  for(int i=0;i<250;i++){b.frame();b.tx();}
  b.check(b.forces==spent&&!b.g.guard,"domain reentry must not resurrect force");
  X leave;leave.state=6;b.edge(leave,"leave mature period");b.publish(350);b.frame();
  for(int i=0;i<100;i++)b.frame();b.tx();b.check(b.forces==spent+1,"new mature period may attempt again");
  // Actual shared controller function:10013 deterministic pairs include
  // theta=10013 and inward omega=20025/20026/20027 (strict > boundary), all
  // signs, zeros, and full signed extrema. No floating approximation.
  std::mt19937 gates(10013);
  for(int i=0;i<10013;i++){
   X q;q.gate=true;q.t=int32_t(gates());q.w=int32_t(gates());
   if(i<24){int sign=(i&1)?-1:1;q.t=sign*10013;q.w=-sign*(20025+(i/2)%3);}
   if(i==24)q.t=0;if(i==25)q.w=0;
   if(i==26){q.t=INT32_MIN;q.w=INT32_MIN;}
   if(i==27){q.t=INT32_MIN;q.w=INT32_MAX;}
   if(i==28){q.t=INT32_MAX;q.w=INT32_MIN;}
   if(i==29){q.t=1;q.w=INT32_MIN;}
   b.edge(q,"10013 returning gate pairs");++b.gate_pairs;
  }
  // Gatefalse spends an attempt but preserves every old-path output/edge.
  for(int sign:{-1,1})for(int extra:{0,1}){
   b.prime();b.frame();X q;q.gate=true;q.t=sign*10013;q.w=-sign*(20026+extra);
   uint64_t oldforce=b.forces,oldattempt=b.attempts;b.tx(q);
   b.check(b.attempts==oldattempt+1&&b.forces==oldforce&&!b.c.pending&&b.c.fired,"skip must spend once at done");
   b.check(!b.g.guard&&int16_t(b.t.duty)==q.observer,"skip must preserve observer exact");
   for(int i=0;i<180;i++){b.frame(q);b.tx(q);}
   b.check(b.attempts==oldattempt+1&&b.forces==oldforce,"skip cannot defer to later frame");
  }
  b.prime();b.frame();X zero;zero.gate=true;zero.t=0;zero.w=0;b.tx(zero);
  b.check(!b.g.guard&&b.c.fired&&!b.c.pending,"zero theta skip consumes");
  for(int sign:{-1,1}){
   b.prime();b.frame();X q;q.gate=true;q.t=sign*10013;q.w=-sign*20025;
   b.tx(q);b.check(b.g.guard&&b.g.rescue==1&&b.g.stable==0,"inside strict gate force first tick");
  }
  // Every state/domain bypass drives >100 saturated frames yet remains old RTL.
  for(int kind=0;kind<18;kind++){
   b.reset();b.publish(350);X y;
   if(kind<8)y.state=kind;
   else if(kind==8)y.manual=true;
   else if(kind==9)y.stationary=false;
   else if(kind==10)y.moving=true;
   else if(kind==11)y.mode=0;
   else if(kind==12)y.mode=1;
   else if(kind==13)y.hold=1999;
   else if(kind==14){y.state=4;y.stationary=true;}
   else if(kind==15){y.state=3;y.manual=true;y.moving=true;}
   else if(kind==16){y.state=2;y.hold=2000;}
   else {y.state=3;y.stationary=false;y.moving=true;}
   if(y.mature())continue;
   for(int i=0;i<130;i++){b.frame(y);b.tx(y);}
   b.check(!b.c.fired&&!b.c.pending&&!b.c.n,"bypass cannot learn");
  }
  // Force pending and its completion are canceled at each raw clock around
  // a 37-clock simulated observer wait and the guard's one-clock pipeline.
  for(int reason=0;reason<7;reason++)for(int offset=0;offset<43;offset++){
   b.prime();b.frame();X y;
   for(int i=0;i<offset;i++){y.start=(i==37);b.edge(y,"pre cancel raw clock");}
   y.start=true;
   if(reason==0)y.stop=true;if(reason==1)y.clear=true;if(reason==2)y.rst=false;
   if(reason==3)y.manual=true;if(reason==4)y.state=7;
   if(reason==5)y.state=4;if(reason==6)y.moving=true;
   b.edge(y,"cancel priority");
   b.check(!b.c.pending&&!b.c.inflight&&!b.c.fired&&!b.c.n,"cancel early state");
   uint64_t f=b.forces;y.start=false;y.stop=false;y.clear=false;y.rst=true;
   for(int i=0;i<50;i++)b.edge(y,"cancel drain");
   b.check(b.forces==f&&!b.c.pending,"no late force");++b.cancel_cases;
  }
  // Seeded arbitrary raw traffic checks counter, native contract, independent
  // oracle and old-path equivalence without a plant or fixture-label input.
  b.reset();std::mt19937 rng(20261024);
  for(int i=0;i<60000;i++){
   X y;y.frame=rng()%5==0;y.start=rng()%4==0;y.publish=rng()%3==0;
   y.command=int(rng()%801)-400;y.cap=int(rng()%201)+150;
   y.t=int(rng()%60001)-30000;y.w=int(rng()%600001)-300000;y.v=int(rng()%800001)-400000;
   y.observer=int(rng()%701)-350;y.fallback=int(rng()%701)-350;
   y.valid=rng()%17!=0;y.manual=rng()%101==0;y.moving=rng()%109==0;
   y.state=rng()%113==0?4:3;y.hold=rng()%127==0?1999:2000;
   y.stop=rng()%503==0;y.clear=rng()%601==0;y.rst=rng()%701!=0;
   b.edge(y,"random");
  }
  std::cout<<"{\"status\":\"PASS\",\"raw_clock_edges\":"<<b.clocks<<",\"assertions\":"<<b.checks
   <<",\"raw_frames\":"<<b.frames<<",\"guard_transactions\":"<<b.transactions<<",\"budget_trigger_accepts\":"<<b.forces
   <<",\"budget_attempt_accepts\":"<<b.attempts<<",\"returning_gate_pairs\":"<<b.gate_pairs<<",\"returning_skips\":"<<b.gate_skips
   <<",\"release_pulses\":"<<b.releases<<",\"old_path_equal_edges\":"<<b.bypass<<",\"cancel_clock_cases\":"<<b.cancel_cases
   <<",\"mismatches\":0,\"closed_loop_cases\":0}\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
