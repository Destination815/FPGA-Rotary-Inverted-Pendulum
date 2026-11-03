#include "r29_handoff_contract.hpp"
#include "legacy_contract.hpp"
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
int main(){
 try{
  std::mt19937 r(20261024);RescueHandoff n;RescueHandoffLegacy o;
  EarlySaturation e;int oldcount=0;bool oldfired=false,oldprior=false;
  uint64_t triggers=0;
  for(int i=0;i<160000;i++){
   bool mature=i%2001>0;int cap=150+r()%201,command=(i%300<170)?cap:int(r()%701)-350;
   if(!mature){oldcount=0;oldfired=false;}
   bool force=false;
   if(mature&&!oldfired){
    oldcount=oldprior&&cap>0&&std::llabs(int64_t(command))>=cap?std::min(100,oldcount+1):0;
    if(oldcount>=100){oldfired=true;force=true;}
   }
   oldprior=mature;
   bool actual=e.tick(mature,command,cap);
   if(force!=actual||oldcount!=e.count||oldfired!=e.fired||oldprior!=e.prior_mature)throw std::runtime_error("counter native manual code changed");
   triggers+=force;
   if(force){o.latched=true;o.opposed_ticks=o.stable_ticks=o.compatible_ticks=o.rescue_ticks=0;}
   int mode=(r()%201)?2:1;bool observe=r()%101!=0,valid=r()%97!=0,stop=r()%809==0;
   int32_t t=int(r()%60001)-30000,w=int(r()%600001)-300000,v=int(r()%800001)-400000;
   if(i%600<300)t=w=v=0;
   int mixed=int(r()%701)-350,fallback=int(r()%701)-350;
   int a=o.tick(mode,t,w,v,observe,valid,mixed,fallback,stop);
   int b=n.tick(mode,t,w,v,observe,valid,mixed,fallback,stop,force);
   if(a!=b||o.latched!=n.latched||o.eligible!=n.eligible||o.activated!=n.activated||o.released!=n.released||o.opposed_ticks!=n.opposed_ticks||o.stable_ticks!=n.stable_ticks||o.compatible_ticks!=n.compatible_ticks||o.rescue_ticks!=n.rescue_ticks)throw std::runtime_error("native optional force changed root pre-tick assignment semantics");
  }
  std::cout<<"{\"status\":\"PASS\",\"transactions\":160000,\"counter_triggers\":"<<triggers<<",\"mismatches\":0}\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
