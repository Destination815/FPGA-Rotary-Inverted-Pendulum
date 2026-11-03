#include "Vpassive_capture.h"
#include "verilated.h"
#include <cassert>
#include <iostream>
Vpassive_capture d;unsigned checked=0,scenarios=0;
void clk(){d.clk=0;d.eval();d.clk=1;d.eval();}
void begin(unsigned t,bool automatic){d.sample=0;d.stop=0;d.millis=t;d.automatic_start=automatic;d.begin_record=1;clk();d.begin_record=0;d.automatic_start=0;assert(d.start_window==automatic);}
void pending(unsigned t,unsigned state=1,unsigned fault=0){d.millis=t;for(int k=0;k<11;k++)d.sample_data[k]=t+10000*k;d.sample_data[1]=(fault<<24)|(state<<16)|(150+t%51);d.sample=1;clk();d.sample=0;}
void row(unsigned t,unsigned state=1,unsigned fault=0){pending(t,state,fault);clk();}
void stop(){d.stop=1;clk();d.stop=0;}
void check(unsigned n,unsigned first,unsigned reason,bool automatic,unsigned trigger=4095){for(int k=0;k<5;k++)clk();assert(!d.busy&&d.valid&&d.count==n&&d.start_ms==first&&d.reason==reason&&d.start_window==automatic&&d.trigger_index==trigger);for(unsigned i=0;i<n;i++){d.read_index=i;clk();clk();assert(d.read_data[0]==first+i);assert((d.read_data[1]&65535)==150+(first+i)%51);assert(d.read_data[10]==first+i+100000);checked++;}scenarios++;}
int main(){d.rst_n=0;clk();d.rst_n=1;
 // Explicit START, independent of first measured state; repeated timestamps do not consume capacity.
 begin(1000,true);for(unsigned i=0;i<4095;i++){row(1000+i,3);row(1000+i,3);}assert(d.busy&&!d.valid);pending(5095,3);assert(!d.busy&&!d.valid);check(4096,1000,6,true);
 for(unsigned i=5096;i<7000;i++)row(i,3);stop();check(4096,1000,6,true); // buffer freeze is not STOP; later state remains accepted by the surrounding system.
 // Capture marker retained, but it cannot move/extend/shorten a START window.
 begin(10000,true);row(10000,1);row(10001,2);for(unsigned i=2;i<4096;i++)row(10000+i,i==500?1:i==501?2:3);check(4096,10000,6,true,1);
 // Last-slot trigger and no-trigger sentinel share4095; actual state transition disambiguates them.
 begin(20000,true);for(unsigned i=0;i<4095;i++)row(20000+i,6);pending(24095,2);check(4096,20000,6,true,4095);assert(((d.read_data[1]>>16)&7)==2);
 // STOP wins over simultaneous final/fault sample; pending earlier RAM write remains committed.
 begin(30000,true);for(unsigned i=0;i<4094;i++)row(30000+i);pending(34094);d.sample=1;d.stop=1;d.millis=34095;d.sample_data[1]=(4<<24)|(5<<16);clk();d.sample=0;d.stop=0;check(4095,30000,1,true);
 // Fault wins over capacity and includes its sample.
 begin(40000,true);for(unsigned i=0;i<4095;i++)row(40000+i);pending(44095,5,4);check(4096,40000,3,true);assert(d.read_data[1]>>24==4);
 begin(50000,true);for(unsigned i=0;i<39;i++)row(50000+i);pending(50039,5,2);check(40,50000,3,true);
 begin(60000,true);for(unsigned i=0;i<21;i++)row(60000+i,i<10?1:2);stop();check(21,60000,1,true,10);
 begin(70000,true);stop();for(int k=0;k<5;k++)clk();assert(!d.busy&&!d.valid&&d.count==0&&d.start_window);scenarios++;
 //32-bit clock wrap preserves4096 consecutive milliseconds.
 begin(0xfffffc00u,true);for(unsigned i=0;i<4096;i++)row(0xfffffc00u+i);check(4096,0xfffffc00u,6,true);
 // Accepted repeated BEGIN resets the window and increments session identity, including after frozen data.
 unsigned id=d.capture_id;begin(80000,true);for(unsigned i=0;i<50;i++)row(80000+i);begin(90000,true);assert(d.capture_id==id+2);for(unsigned i=0;i<4096;i++)row(90000+i);check(4096,90000,6,true);
 begin(100000,false);for(unsigned i=0;i<4500;i++)row(100000+i,i<2000?2:3);assert(d.busy&&!d.start_window);stop();check(4096,100404,1,false);
 // A START strobe without BEGIN cannot reinterpret an active manual capture session.
 begin(110000,false);d.automatic_start=1;clk();d.automatic_start=0;for(unsigned i=0;i<4500;i++)row(110000+i,2);assert(d.busy&&!d.start_window);stop();check(4096,110404,1,false);
 // Manual-session legacy recovery/event-window contract remains1+3072post samples.
 begin(120000,false);for(unsigned i=0;i<1500;i++)row(120000+i,6);row(121500,2);for(unsigned i=1;i<=3072;i++)row(121500+i,3);check(4096,120477,5,false,1023);
 begin(130000,false);for(unsigned i=0;i<32000;i++)row(130000+i,3);check(4096,157904,4,false);
 // Reset at each freeze pipeline stage removes valid/session/trigger; next manual capture is clean.
 for(unsigned phase=0;phase<5;phase++){begin(200000,true);for(unsigned i=0;i<4095;i++)row(200000+i);pending(204095);for(unsigned i=0;i<phase;i++)clk();d.rst_n=0;clk();d.rst_n=1;assert(!d.busy&&!d.valid&&!d.start_window&&d.count==0&&d.trigger_index==4095);begin(300000,false);for(unsigned i=0;i<40;i++)row(300000+i,2);stop();check(40,300000,1,false);}
 // Shared legacy bank still works and clears START-session status.
 d.legacy_select=1;clk();assert(!d.valid&&!d.start_window);for(unsigned i=0;i<1024;i++){d.legacy_write=1;d.legacy_address=i;for(int j=0;j<10;j++)d.legacy_data[j]=i+j*12345;clk();}d.legacy_write=0;for(unsigned i=0;i<1024;i++){d.read_index=i;clk();clk();for(int j=0;j<10;j++)assert(d.read_data[j]==i+j*12345);assert(d.read_data[10]==0);checked++;}scenarios++;
 std::cout<<"PASS scenarios="<<scenarios<<" read_rows="<<checked<<" explicit START first4096, manual legacy windows, trigger boundaries, STOP/fault precedence, repeat/reset/timewrap/duplicate and shared RAM\n";
}
