#include "hybrid_fixed.hpp"
#include "uncompact.hpp"
#include "baseline.hpp"
#include "maps.hpp"
#include "layout.hpp"
#include "Vs23_compute.h"
#include "verilated.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
using hybrid_fixed::Machine;
static VerilatedContext ctx;static Vs23_compute rtl{&ctx};static unsigned tx=0,checks=0,unchecks=0,firstchecks=0,changed=0,maxcycles=0;
void need(bool q,const char*m){if(!q)throw std::runtime_error(m);}void ck(){rtl.clk=0;rtl.eval();rtl.clk=1;rtl.eval();}
void put(int a,int64_t v){rtl.ext_address=a;rtl.ext_data=v;rtl.ext_write=1;ck();rtl.ext_write=0;}
int64_t get(int a){rtl.ext_address=a;ck();ck();return int64_t(uint64_t(rtl.ext_read)<<16)>>16;}
void set(Machine&m,int a,double v){m.mem[a]=std::llround(v*hybrid_fixed::S);}
void run(Machine q,bool first){uncompact::Machine u;baseline::Machine b;for(auto&p:MAPS){u.mem[p[1]]=q.mem[p[0]];b.mem[p[2]]=q.mem[p[0]];}
for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE)put(MAP[k],q.mem[k]);q.run();u.run();b.run();
for(auto&p:MAPS){need(q.mem[p[0]]==u.mem[p[1]],"scratch live state");unchecks++;if(first){need(q.mem[p[0]]==b.mem[p[2]],"first acquisition changed");firstchecks++;}}
need(q.mem[hybrid_fixed::mode]==b.mem[baseline::mode],"capture predicate unchanged"); if(q.mem[hybrid_fixed::output]!=b.mem[baseline::output])changed++;
rtl.start=1;ck();rtl.start=0;unsigned cy=0;while(!rtl.done&&cy++<50000)ck();need(rtl.done&&!rtl.fault,"CPU completion");maxcycles=std::max(maxcycles,cy);
for(int k=0;k<OLD_WORDS;k++)if(MAP[k]<WRITABLE){need(get(MAP[k])==q.mem[k],"actual writable RAM");checks++;}tx++;}

int main(int argc,char**argv){try{using namespace hybrid_fixed;rtl.rst_n=0;ck();rtl.rst_n=1;rtl.initialize=1;ck();rtl.initialize=0;unsigned z=0;while(!rtl.ready&&z++<10000)ck();need(rtl.ready,"init");
for(int sign:{-1,1})for(int captured:{0,1})for(int valid:{0,1})for(int kind=0;kind<24;kind++){
Machine m;set(m,measurement_0,sign*(kind%4==0?.3:kind%4==1?1.21:kind%4==2?1.6:1.99));set(m,measurement_1,-sign*(kind%3==0?4.01:kind%3==1?7.:10.));set(m,measurement_2,0);set(m,measurement_3,sign*(kind%6==0?.3:kind%6==1?1.:kind%6==2?2.:kind%6==3?-2.:kind%6==4?-1.:0.));
set(m,mode,0);set(m,ever,captured);set(m,recovery_energy,1.4);set(m,vreq,-sign*.8);set(m,age,300);set(m,tick,1000);set(m,attempt,1500);set(m,phase_active,0);set(m,phase_enabled,0);set(m,counter,19);set(m,origin,.01);set(m,observer_ready_input,0);set(m,progress_measurement_valid,valid);set(m,applied,sign*.1);run(m,!captured);}
for(int sign:{-1,1})for(int valid:{0,1})for(int kind=0;kind<12;kind++){
Machine m;set(m,measurement_0,sign*.01);set(m,measurement_1,sign*.1);set(m,measurement_2,.02);set(m,measurement_3,sign*.1);set(m,mode,2);set(m,ever,1);set(m,age,2000+kind);set(m,tick,3000);set(m,counter,19);set(m,origin,.01);set(m,observer_ready_input,1);set(m,progress_measurement_valid,valid);set(m,applied,sign*.1);run(m,true);}
for(int file=1;file<argc;file++){Machine m;set(m,ever,1);set(m,mode,2);set(m,age,2000);set(m,tick,3000);set(m,phase_active,0);set(m,phase_enabled,0);std::ifstream in(argv[file],std::ios::binary);int64_t r[8];bool first=true;int last=0,brage=0,index=0;
 while(in.read((char*)r,sizeof r)){if(first){m.mem[origin]=r[2]*512;first=false;}for(int k=0;k<4;k++)m.mem[k]=r[k]*512;set(m,applied,r[4]/1000.);set(m,progress_measurement_valid,r[5]);if(r[6]==7){last=7;continue;}if(last==7)set(m,resume_elapsed,40);last=r[6];brage=(r[6]==6||r[6]==2)?brage+1:0;set(m,observer_ready_input,brage>200);if(index>150&&index%20==0)run(m,false);m.run();index++;}}
need(changed>0,"no new policy coverage");std::cout<<"{\"status\":\"PASS\",\"transactions\":"<<tx<<",\"actual_writable_checks\":"<<checks<<",\"uncompacted_named_checks\":"<<unchecks<<",\"unchanged_first_or_hold_named_checks\":"<<firstchecks<<",\"changed_recovery_outputs\":"<<changed<<",\"max_cpu_cycles\":"<<maxcycles<<"}\n";
}catch(const std::exception&e){std::cerr<<e.what()<<" tx="<<tx<<"\n";return 1;}}
