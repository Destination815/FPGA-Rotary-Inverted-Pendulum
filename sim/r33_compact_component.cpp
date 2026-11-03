#include "Vcompact_test_top.h"
#include "verilated.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using Row=std::array<uint32_t,6>;
Vcompact_test_top d;
uint64_t clocks=0,checked=0;unsigned cases=0;
std::string output;
void tick(){d.clk=0;d.eval();d.clk=1;d.eval();++clocks;}
void idle(int n=4){d.sample=0;d.begin_record=0;d.stop=0;while(n--)tick();}
void reset(){d.rst_n=0;d.legacy_select=0;d.legacy_write=0;d.sample=0;d.stop=0;d.begin_record=0;tick();tick();d.rst_n=1;idle();}
void begin(uint32_t ms=100,bool automatic=true){d.millis=ms;d.begin_record=1;d.automatic_start=automatic;tick();d.begin_record=0;assert(d.busy&&!d.valid);}
void setbits(Row& r,unsigned at,unsigned n,uint32_t x){for(unsigned b=0;b<n;b++)if(x&(1u<<b))r[(at+b)/32]|=1u<<((at+b)%32);}
Row expected(){Row r{};setbits(r,0,32,d.millis);setbits(r,32,14,d.adc_q4);setbits(r,46,32,d.encoder);setbits(r,78,10,d.command_pwm);setbits(r,88,10,d.applied_pwm);setbits(r,98,16,d.flags);setbits(r,114,4,d.state);setbits(r,118,9,d.limit);setbits(r,127,1,d.adc_fine_valid);setbits(r,128,21,d.theta_q16);setbits(r,149,23,d.dtheta_q16);setbits(r,172,4,d.fault);return r;}
Row sample(uint32_t ms,unsigned i,unsigned fault=0){
 d.millis=ms;d.adc_q4=(i*731u)&16383;d.encoder=uint32_t(0x80000000u+i*104729u);
 d.command_pwm=int(i%1024)-512;d.applied_pwm=511-int(i%1024);d.flags=uint16_t(i*257u);
 d.state=(i<200?1:i==200?2:3);d.limit=i%512;d.adc_fine_valid=i&1;
 d.theta_q16=int(i%2097152)-1048576;d.dtheta_q16=4194303-int(i%8388608);d.fault=fault;
 Row r=expected();d.sample=1;tick();d.sample=0;return r;
}
Row read(unsigned i){d.read_index=i;tick();assert(d.read_valid);Row r{};for(int k=0;k<6;k++)r[k]=d.read_data[k];return r;}
uint16_t crc(const std::array<uint8_t,64>& b){uint16_t c=65535;for(unsigned i=2;i<62;i++){c^=uint16_t(b[i])<<8;for(int j=0;j<8;j++)c=c&0x8000?(c<<1)^0x1021:c<<1;}return c;}
template<class T>void writeframe(std::ofstream& f,T& words){std::array<uint8_t,64>b{};for(int i=0;i<62;i++)b[i]=words[i/4]>>(8*(i%4));uint16_t c=crc(b);b[62]=c;b[63]=c>>8;f.write((char*)b.data(),64);}
void check(const std::string& name,const std::vector<Row>& full,unsigned reason,unsigned gaps=0,bool dump=false){
 idle();assert(!d.busy);assert(d.valid==!full.empty());assert(d.reason==reason);assert(d.gaps==gaps);
 unsigned n=std::min(size_t(8192),full.size()),h=std::min(size_t(4096),full.size()),t=n-h;
 assert(d.count==n&&d.head_count==h&&d.tail_count==t&&d.total_samples==full.size());
 assert(d.tail_base==(full.size()>8192?(full.size()-4096)%4096:0));
 std::vector<Row> retained;
 for(unsigned i=0;i<n;i++){unsigned ordinal=i<h?i:full.size()-t+i-h;auto r=read(i);if(r!=full[ordinal]){std::cerr<<name<<" row "<<i<<" ordinal "<<ordinal<<" mismatch\n";std::abort();}retained.push_back(r);++checked;}
 if(n<8192){d.read_index=n;tick();assert(!d.read_valid);}
 if(dump&&n){std::ofstream f(output+"/"+name+".wire",std::ios::binary);writeframe(f,d.info_payload);for(unsigned i=0;i<n;i+=2){d.first_index=i;d.row_count=std::min(2u,n-i);for(unsigned k=0;k<6;k++){d.row0[k]=retained[i][k];d.row1[k]=i+1<n?retained[i+1][k]:0;}d.eval();writeframe(f,d.data_payload);}}
 ++cases;
}
void stop(){d.stop=1;tick();d.stop=0;}
int main(int argc,char**argv){Verilated::commandArgs(argc,argv);output=argc>1?argv[1]:".";std::filesystem::create_directories(output);
 for(unsigned n:{0,1,4095,4096,4097,8191,8192,8193,10000,30000}){
  reset();begin(100,n!=30000);std::vector<Row> rows;
  for(unsigned i=0;i<n;i++)rows.push_back(sample(101+i,i));
  assert(d.busy);stop();check("size_"+std::to_string(n),rows,1,0,n==1||n==8192||n==10000||n==30000);
  if(n>200)assert(d.trigger_index==200);
 }
 for(unsigned n:{1,4096,8192,12000}){
  reset();begin();std::vector<Row> rows;for(unsigned i=0;i<n;i++)rows.push_back(sample(101+i,i,i==n-1?4:0));
  check("fault_"+std::to_string(n),rows,3,0,n==12000);assert(d.freeze_fault==4);
 }
 // STOP simultaneously with incoming fault/sample/START retains only earlier sample.
 reset();begin();std::vector<Row> r{sample(101,0)};d.sample=1;d.stop=1;d.begin_record=1;d.fault=9;d.millis=102;tick();d.sample=0;d.stop=0;d.begin_record=0;check("stop_priority",r,1,0,true);assert(d.capture_id==1&&d.freeze_fault==9);
 // Duplicate/missing/backward times remain explicit. A natural uint32 wrap is continuous.
 reset();begin(0xfffffffeu);r.clear();for(unsigned i=0;i<6;i++)r.push_back(sample(0xffffffffu+i,i));stop();check("millis_wrap",r,1,0,true);
 reset();begin();r.clear();for(uint32_t ms:{101,102,102,105,104,105})r.push_back(sample(ms,r.size()));stop();check("gaps",r,1,3,true);assert(d.first_gap_ms==102);
 // New session clears all metadata; old pending RAM word cannot leak into valid count.
 reset();begin();sample(101,12);begin(500,false);r={sample(501,88)};stop();check("restart_pending",r,1,0,true);assert(d.capture_id==2&&!d.automatic_session);
 reset();begin();sample(101,12,4);begin(600);r={sample(601,99)};stop();check("restart_freezing",r,1);assert(d.capture_id==2);
 reset();begin();sample(101,12);reset();assert(!d.valid&&!d.busy&&!d.count&&d.capture_id==0);begin();r={sample(101,55)};stop();check("reset_pending",r,1);
 // Width rejection checks full source ports, never silently truncates.
 for(unsigned which=0;which<12;which++){
  reset();begin();r={sample(101,1)};d.millis=102;d.fault=0;
  switch(which){case 0:d.command_pwm=512;break;case 1:d.command_pwm=-513;break;case 2:d.applied_pwm=512;break;case 3:d.applied_pwm=-513;break;case 4:d.limit=512;break;case 5:d.limit=0xffffffff;break;case 6:d.theta_q16=1048576;break;case 7:d.theta_q16=-1048577;break;case 8:d.dtheta_q16=4194304;break;case 9:d.dtheta_q16=-4194305;break;case 10:d.fault=16;break;case 11:d.fault=255;break;}
  d.sample=1;tick();d.sample=0;check("range_"+std::to_string(which),r,8);
 }
 // Inclusive signed limits, state15/flagsFFFF/ADC16383/limit511 survive one row.
 reset();begin();sample(101,0);d.millis=102;d.adc_q4=16383;d.encoder=0x7fffffff;d.command_pwm=511;d.applied_pwm=-512;d.flags=65535;d.state=15;d.limit=511;d.adc_fine_valid=1;d.theta_q16=1048575;d.dtheta_q16=-4194304;d.fault=15;r={};r.push_back(expected());d.sample=1;tick();d.sample=0;
 // Retain the initial opposite signed endpoints too.
 std::vector<Row> bounds;d.millis=101;d.adc_q4=0;d.encoder=0x80000000;d.command_pwm=-512;d.applied_pwm=511;d.flags=0;d.state=1;d.limit=0;d.adc_fine_valid=0;d.theta_q16=-1048576;d.dtheta_q16=4194303;d.fault=0;bounds.push_back(expected());bounds.push_back(r[0]);check("signed_bounds",bounds,3,0,true);
 // Capture trigger in the rolling tail, then aged out: sentinel != a false retained index.
 for(unsigned n:{7000,14000}){reset();begin();r.clear();for(unsigned i=0;i<n;i++){d.millis=101+i;d.state=i==5000?2:1;d.adc_q4=0;d.encoder=i;d.command_pwm=0;d.applied_pwm=0;d.flags=0;d.limit=350;d.fault=0;d.theta_q16=0;d.dtheta_q16=0;auto row=expected();d.sample=1;tick();d.sample=0;r.push_back(row);}stop();check("trigger_"+std::to_string(n),r,1,0,true);assert(d.trigger_index==(n==7000?5000:65535));assert(d.trigger_ms==5101);}
 // Existing D02 legacy rows share the same physical RAM, two 176-bit words.
 reset();d.legacy_select=1;std::array<std::array<uint32_t,10>,1024> legacy{};
 for(unsigned i=0;i<1024;i++){for(unsigned k=0;k<10;k++){legacy[i][k]=0x9e3779b9u*i+0xabcdef01u*k;d.legacy_data[k]=legacy[i][k];}d.legacy_address=i;d.legacy_write=1;tick();d.legacy_write=0;tick();tick();assert(!d.legacy_overrun);}
 for(unsigned i=0;i<1024;i++){d.legacy_read_index=i;bool saw=false;for(unsigned j=0;j<6;j++){tick();if(d.legacy_read_valid){for(unsigned k=0;k<10;k++)assert(d.legacy_read_data[k]==legacy[i][k]);saw=true;}}assert(saw);}
 ++cases;d.legacy_select=0;begin();r={sample(101,123)};stop();check("after_legacy",r,1);
 reset();d.legacy_select=1;d.legacy_write=1;tick();tick();assert(d.legacy_overrun);++cases;
 std::ofstream f(output+"/rtl_result.json");f<<"{\"status\":\"PASS\",\"scenarios\":"<<cases<<",\"rows_bit_exact\":"<<checked<<",\"clock_edges\":"<<clocks<<"}\n";std::cout<<"PASS "<<cases<<" scenarios, "<<checked<<" rows bit-exact, "<<clocks<<" RTL clocks\n";
}
