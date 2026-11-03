#include "Vmotion_config_stage.h"
#include "verilated.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
static unsigned checks=0;
static void need(bool b,const char*s){++checks;if(!b)throw std::runtime_error(s);}
static void edge(Vmotion_config_stage&t){t.clk=0;t.eval();t.clk=1;t.eval();}
static void reset(Vmotion_config_stage&t){t.in_valid=0;t.new_command=0;t.stop_level=0;t.in_overflow=0;t.rst_n=0;edge(t);t.rst_n=1;edge(t);}
int main(){try{
 VerilatedContext c;Vmotion_config_stage t{&c};reset(t);uint32_t rng=0x23912345;
 for(unsigned op=0;op<256;++op){
  rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
  t.in_valid=1;t.in_opcode=op;t.in_value=rng;t.in_seq=op;t.admission=op%4;t.in_overflow=op%2;t.eval();
  need(!t.out_valid,"unregistered ordinary output");edge(t);
  need(t.out_valid&&t.out_opcode==op&&t.out_seq==op&&t.out_value==rng&&t.out_overflow==op%2,"registered command misalignment");
  if(op==0x36||op==0x38)need(t.configuration==rng&&t.saved_admission==op%4,"configuration admission mismatch");
  t.in_valid=0;t.in_value=~rng;t.in_opcode=~op;t.in_seq=~op;t.in_overflow=!(op%2);t.eval();
  need(t.out_value==rng&&t.out_opcode==op&&t.out_overflow==op%2,"raw input propagates to output");edge(t);need(!t.out_valid,"command duplicated");
 }
 for(unsigned i=0;i<4096;++i){
  rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
  t.in_valid=1;t.in_opcode=0x36;t.in_value=rng;t.in_seq=i;t.admission=i%4;edge(t);
  need(t.out_valid&&t.out_value==rng&&t.configuration==rng&&t.saved_admission==i%4,"configuration snapshot mismatch");
  t.in_valid=0;t.in_value=~rng;t.admission=(i+1)%4;t.eval();
  need(t.configuration==rng&&t.saved_admission==i%4,"configuration snapshot changed without edge");
  edge(t);need(!t.out_valid,"configuration duplicated");
 }
 for(unsigned op=0;op<256;++op){
  t.in_valid=1;t.in_opcode=op;t.new_command=1;edge(t);need(!t.out_valid,"new raw command failed to cancel old stage");
  t.new_command=0;t.in_valid=0;edge(t);need(!t.out_valid,"cancelled stage reappeared");
  t.in_valid=1;t.in_opcode=op;t.stop_level=1;edge(t);need(!t.out_valid,"STOP failed to cancel stage");
  t.stop_level=0;t.in_valid=0;edge(t);need(!t.out_valid,"STOP-cancelled stage reappeared");
 }
 t.in_valid=1;edge(t);t.rst_n=0;t.eval();need(!t.out_valid,"reset did not cancel output");
 std::cout<<"PASS registered_command_stage_checks="<<checks<<" opcodes=256 config_payloads=4096 cancellations=512\n";
 }catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
