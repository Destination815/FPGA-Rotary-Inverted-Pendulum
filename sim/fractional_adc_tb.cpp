#include "Vadc_parallel.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <algorithm>
static uint64_t checks=0;
static void ck(bool v,const char*msg){++checks;if(!v)throw std::runtime_error(msg);}
static void clock(Vadc_parallel&t){t.clk=0;t.eval();t.clk=1;t.eval();}
static uint16_t datum(int kind,int n){switch(kind){case 0:return 740;case 1:return 740+(n&1);case 2:return 720+(n/128)%25;case 3:return n%127==0?0:740;case 4:return 1023;case 5:return 738+(n*73+19)%4;default:return n%2048==100?790:740;}}
static void run(Vadc_parallel&t,int kind,int samples){
 t.rst_n=0;clock(t);t.rst_n=1;
 ck(t.average_q4==0&&!t.ready_q4&&!t.valid_q4,"reset fractional outputs");
 int phase=0,warm=0,cnt=0,fcount=0;uint32_t sum=0,fsum=0;int low=0,high=0;bool good=false;int avg=0,favg=0;bool ready=false,fr=false,fv=false,ac=false;int locked=0;
 for(int cyc=0;cyc<samples*20;cyc++){
  int sample=datum(kind,cyc/20);t.data=phase==15?sample:((sample+131)%1024);
  if(phase==19)ac=true;else if(phase==9)ac=false;
  if(phase==15)locked=sample;
  if(phase==16){
   if(warm<16)warm++;
   else{
    if(fcount==0){low=high=locked;good=locked>3&&locked<1020;}
    else{low=std::min(low,locked);high=std::max(high,locked);good=good&&locked>3&&locked<1020;}
    fsum+=locked;
    if(fcount==511){favg=fsum>>5;fr=true;fv=good&&high-low<=32;fsum=0;fcount=0;}else fcount++;
    sum+=locked;if(cnt==15){avg=sum>>4;ready=true;sum=0;cnt=0;}else cnt++;
   }
  }
  clock(t);
  ck(t.adc_clk==ac,"ADC clock edge");ck(t.average==avg&&t.ready==ready,"legacy 16 mean changed");
  ck(t.average_q4==favg&&t.ready_q4==fr&&t.valid_q4==fv,"fractional 512 mean mismatch");
  phase=(phase+1)%20;
 }
}
int main(){try{Vadc_parallel t;for(int kind=0;kind<7;kind++)run(t,kind,6300);run(t,1,1000);run(t,0,3000);std::cout<<"PASS "<<checks<<" frontend edge/arithmetic/reset assertions\n";return 0;}catch(std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
