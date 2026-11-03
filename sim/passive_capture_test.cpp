// Historical/manual-session compatibility fixture: explicit automatic_start=0.
// R33 START coverage is in r33_passive_capture_test.cpp.
#include "Vpassive_capture.h"
#include "verilated.h"
#include <cassert>
#include <iostream>
Vpassive_capture d;unsigned checked=0;
void clk(){d.clk=0;d.eval();d.clk=1;d.eval();}
void begin(unsigned t){d.automatic_start=0;d.sample=0;d.stop=0;d.millis=t;d.begin_record=1;clk();d.begin_record=0;}
void pending_row(unsigned t,unsigned fault=0,unsigned state=3){d.millis=t;for(int k=0;k<11;k++)d.sample_data[k]=t+10000*k;d.sample_data[1]=(fault<<24)|(state<<16)|(150+t%51);d.sample=1;clk();d.sample=0;}
void row(unsigned t,unsigned fault=0,unsigned state=3){pending_row(t,fault,state);clk();}
void check(int n,unsigned first,int reason,unsigned trigger=4095){for(int k=0;k<5;k++)clk();assert(d.valid && !d.busy && d.count==n && d.start_ms==first && d.reason==reason && d.trigger_index==trigger);for(int i=0;i<n;i++){d.read_index=i;clk();clk();assert(d.read_data[0]==first+unsigned(i));assert((d.read_data[1]&65535)==150+(first+unsigned(i))%51);assert(d.read_data[10]==first+unsigned(i)+100000);checked++;}}
void stop(){d.stop=1;clk();d.stop=0;}
int main(){d.rst_n=0;clk();d.rst_n=1;
 // 30 second hold: final full five 5Hz cycles, not first motion window.
 begin(100);for(unsigned i=0;i<30000;i++){row(100+i);row(100+i);}assert(d.busy);stop();check(4096,100+30000-4096,1);
 row(99999,4);check(4096,100+30000-4096,1);
 // Early fault includes fault sample and stops without waiting for host STOP.
 begin(40000);for(unsigned i=0;i<1081;i++)row(40000+i,i==1080?4:0);check(1081,40000,3);assert((d.read_data[1]>>24)==4);
 stop();row(99999,0);check(1081,40000,3);
 begin(50000);for(unsigned i=0;i<40;i++)row(50000+i,i==39?4:0);check(40,50000,3);
 begin(60000);for(unsigned i=0;i<32000;i++)row(60000+i);check(4096,60000+32000-4096,4);
 // Time wrap and simultaneous STOP + sample: command stop wins; no extra row.
 begin(0xfffffe00u);for(unsigned i=0;i<1200;i++)row(0xfffffe00u+i);d.stop=1;d.sample=1;d.millis++;clk();d.stop=0;d.sample=0;check(1200,0xfffffe00u,1);
 begin(1000);stop();for(int k=0;k<5;k++)clk();assert(!d.busy&&!d.valid&&d.count==0);
 // Auto capture after a wrapped prefix: exact1+3072 post-trigger retention.
 // Duplicate sample times must not consume the post-trigger budget.
 begin(100000);for(unsigned i=0;i<7000;i++)row(100000+i,0,i<6000?1:6);
 row(107000,0,2);for(unsigned i=1;i<3072;i++){row(107000+i,0,i<2000?2:3);row(107000+i,0,2);}
 assert(d.busy&&!d.valid);pending_row(110072,0,3);assert(!d.busy&&!d.valid);
 check(4096,105977,5,1023); // last pending write must be committed before valid.
 stop();row(200000,4,5);check(4096,105977,5,1023);
 // Early capture has fewer pre-trigger records, no invented padding.
 begin(200000);for(unsigned i=0;i<10;i++)row(200000+i,0,7);row(200010,0,2);
 for(unsigned i=1;i<=3072;i++)row(200010+i,0,3);check(3083,200000,5,10);
 // Fault takes precedence even on the planned final recording sample.
 begin(300000);for(unsigned i=0;i<2000;i++)row(300000+i,0,1);row(302000,0,2);
 for(unsigned i=1;i<3072;i++)row(302000+i,0,3);pending_row(305072,2,5);
 check(4096,300977,3,1023);assert((d.read_data[1]>>24)==2);
 // An earlier fault retains its sample and computes the shifted capture index.
 begin(400000);for(unsigned i=0;i<5000;i++)row(400000+i,0,6);row(405000,0,2);
 for(unsigned i=1;i<31;i++)row(405000+i,0,2);pending_row(405031,1,5);
 check(4096,400936,3,4064);
 // STOP while a capture row is pending excludes the simultaneous later sample.
 begin(500000);for(unsigned i=0;i<5000;i++)row(500000+i,0,1);pending_row(505000,0,2);
 d.stop=1;d.sample=1;d.millis=505001;clk();d.stop=0;d.sample=0;
 check(4096,500905,1,4095);assert(((d.read_data[1]>>16)&7)==2);
 // A second automatic takeover in the window must not extend the first window.
 begin(600000);row(600000,0,1);row(600001,0,2);
 for(unsigned i=2;i<=3073;i++)row(600000+i,0,i==500?1:i==501?2:3);
 check(3074,600000,5,1);
 // Clock wrap affects timestamps only, not count or trigger relocation.
 begin(0xfffffc00u);for(unsigned i=0;i<1500;i++)row(0xfffffc00u+i,0,1);
 row(0xfffffc00u+1500,0,2);for(unsigned i=1;i<=3072;i++)row(0xfffffc00u+1500+i,0,3);
 check(4096,0xfffffc00u+477,5,1023);
 // Manual CAPTURE begins directly at2 and must retain its original tail behavior.
 begin(700000);for(unsigned i=0;i<10000;i++)row(700000+i,0,i<2000?2:3);
 assert(d.busy);stop();check(4096,705904,1);
 // A reset/cancelled session must not leak a prior automatic trigger.
 begin(800000);row(800000,0,1);row(800001,0,2);d.rst_n=0;clk();d.rst_n=1;
 begin(900000);for(unsigned i=0;i<5000;i++)row(900000+i,0,2);assert(d.busy);stop();check(4096,900904,1);
 //32second recorder timeout still wins if capture arrives at the end of that budget.
 begin(1000000);for(unsigned i=0;i<31900;i++)row(1000000+i,0,1);row(1031900,0,2);
 for(unsigned i=31901;i<32000;i++)row(1000000+i,0,3);check(4096,1027904,4,3996);
 // Not a direct automatic transition:1 ->3 ->2 cannot forge a capture marker.
 begin(1100000);row(1100000,0,1);row(1100001,0,3);
 for(unsigned i=2;i<4500;i++)row(1100000+i,0,2);assert(d.busy);stop();check(4096,1100404,1);
 // Reset at every stage from queued final RAM write through valid publication.
 for(unsigned phase=0;phase<5;phase++){
  begin(1200000);row(1200000,0,1);row(1200001,0,2);
  for(unsigned i=2;i<3073;i++)row(1200000+i,0,3);pending_row(1203073,0,3);
  for(unsigned i=0;i<phase;i++)clk();d.rst_n=0;clk();d.rst_n=1;
  assert(!d.busy&&!d.valid&&d.count==0&&d.trigger_index==4095);
  begin(1300000);for(unsigned i=0;i<40;i++)row(1300000+i,0,2);stop();check(40,1300000,1);
 }
 // The shared bank is explicitly invalidated before a legacy session.
 d.legacy_select=1;clk();assert(!d.valid&&!d.busy&&d.count==0);
 for(int i=0;i<1024;i++){d.legacy_write=1;d.legacy_address=i;for(int j=0;j<10;j++)d.legacy_data[j]=i+j*12345;clk();}d.legacy_write=0;
 for(int i=0;i<1024;i++){d.read_index=i;clk();clk();for(int j=0;j<10;j++)assert(d.read_data[j]==unsigned(i+j*12345));assert(d.read_data[10]==0);checked++;}
 d.legacy_select=0;begin(90000);for(int i=0;i<4200;i++)row(90000+i);stop();check(4096,90104,1);
 begin(2000);d.rst_n=0;clk();assert(!d.busy&&!d.valid&&d.count==0);
 std::cout<<"PASS manual tail; auto capture3072-post window; duplicate ms, ring/time wrap, early/final fault, STOP pending write, repeated capture, reset, legacy bank; "<<checked<<" rows checked\n";
}
