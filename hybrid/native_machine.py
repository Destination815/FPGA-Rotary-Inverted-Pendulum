"""Emit executable integer oracle for an ISA image, including operation profiling."""
import json
from pathlib import Path
from arithmetic import TABLES,PI
from machine import OPS
D=Path(__file__).resolve().parent

def generate(file,namespace):
 a=json.loads(Path(file).read_text());h='''#pragma once
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>




'''+f'namespace {namespace} {{\n'
 h+='static const int64_t INITIAL[]={'+','.join(map(str,a['initial']))+'};\n'
 h+='static const int CODE[][4]={'+','.join('{'+','.join(map(str,c))+'}'for c in a['code'])+'};\n'
 for n,t in zip(['SINROM','EXPROM','TANHROM'],TABLES):h+=f'static const int64_t {n}[]={{'+','.join(map(str,t))+'};\n'
 h+=f'''static constexpr int Q=25;static constexpr int64_t S=1LL<<Q, PI={PI},TAU={2*PI};
struct Machine {{
 std::vector<int64_t> mem;uint64_t instructions=0,cycles=0;uint64_t counts[37]={{}};
 bool sram=false;
 static int mul_cycles(int64_t a,int64_t b){{uint64_t x=std::abs(a),y=std::abs(b);int nx=((x&0xffffff)!=0)+(int)((x>>24)!=0),ny=((y&0xffffff)!=0)+(int)((y>>24)!=0);return 7;}}
 static int div_cycles(int64_t a,int64_t b){{if(!b)return 4;uint64_t x=std::abs(a),y=std::abs(b);int bx=0,by=0;while(x){{bx++;x>>=1;}}while(y){{by++;y>>=1;}}int bits=a?std::max(0,bx+25-by+1):0;return 6+(bits+1)/2;}}

 Machine():mem(std::begin(INITIAL),std::end(INITIAL)){{}}
 int64_t alu(int op,int64_t a,int64_t b){{__int128 r=0;int cyc=1;switch(op){{
'''
 exprs=dict(SHR='a>>(b/S)',SHL='(__int128)a<<(b/S)',MOV='a',ADD='(__int128)a+b',SUB='(__int128)a-b',MUL='((__int128)a*b)>>Q',DIV='b?((__int128)a*S)/b:throw std::runtime_error("zero divisor")',NEG='-(__int128)a',ABS='a<0?-(__int128)a:a',MIN='std::min(a,b)',MAX='std::max(a,b)',LT='(a<b)*S',LE='(a<=b)*S',GT='(a>b)*S',GE='(a>=b)*S',EQ='(a==b)*S',NE='(a!=b)*S',AND='(a!=0&&b!=0)*S',OR='(a!=0||b!=0)*S')
 for op,expr in exprs.items():
  cost={'MUL':'mul_cycles(a,b)','DIV':'div_cycles(a,b)'}.get(op,1)
  h+=f'case {OPS.index(op)}:r={expr};cyc={cost};break;\n'
 h+='''case 10:case 11:case 12:case 13:{
 int64_t phase,t=a;bool negative=false;const int64_t*table;
 if(op==10||op==11){if(std::abs(a)>1024*S)throw std::runtime_error("trig bound");if(op==11)t+=PI/2;int wraps=0;while(t>PI){t-=TAU;wraps++;}while(t< -PI){t+=TAU;wraps++;}negative=t<0;t=std::abs(t);if(t>PI/2)t=PI-t;phase=t*262144/(PI/2);table=SINROM;cyc=15+wraps;}
 else if(op==12){if(a>0)throw std::runtime_error("exp bound");phase=std::min(-a,32*S)*8192/S;table=EXPROM;cyc=11;}
 else{negative=a<0;phase=std::min(std::abs(a),8*S)*32768/S;table=TANHROM;cyc=11;}
 if(phase>=262144)cyc-=7;int idx=std::min(int64_t(1024),phase>>8);r=table[idx];if(idx<1024)r+=((table[idx+1]-table[idx])*(phase&255))>>8;if(negative)r=-r;break;
 }
 default:throw std::runtime_error("unsupported opcode");}
 cycles+=cyc+(sram?1:0);if(r<-(1LL<<47)||r>((1LL<<47)-1))throw std::runtime_error("overflow");return (int64_t)r;}
 void run(unsigned entry=0){cycles+=1;std::vector<unsigned> stack;for(unsigned pc=entry,guard=0;guard++<10000000;){const int*c=CODE[pc++];instructions++;counts[c[0]]++;
 if(c[0]==29||c[0]==30||c[0]==0){if(stack.empty()){cycles+=sram?2:1;return;}pc=stack.back();stack.pop_back();cycles+=sram?3:2;continue;}
 if(c[0]==22){if(!mem[c[2]]){pc=c[1];cycles+=1;}cycles+=sram?2:1;}
 else if(c[0]==23){pc=c[1];cycles+=sram?3:2;}
 else if(c[0]==24){stack.push_back(pc);pc=c[1];cycles+=sram?3:2;}
 else if(c[0]==25){if(stack.empty()){cycles+=sram?3:2;return;}pc=stack.back();stack.pop_back();cycles+=sram?3:2;}
 else if(c[0]==26){mem[c[1]]=mem.at(c[2]+mem[c[3]]/S);cycles+=sram?3:1;}
 else if(c[0]==27){mem.at(c[1]+mem[c[2]]/S)=mem[c[3]];cycles+=sram?2:1;}
 else if(c[0]==28){cycles+=sram?2:1;}
 else mem[c[1]]=alu(c[0],mem[c[2]],mem[c[3]]);
 }throw std::runtime_error("watchdog");}
};
'''
 for n,i in a['names'].items():
  if not n.startswith(('tmp','constant')):h+='static constexpr int '+n.replace('[','_').replace(']','')+'='+str(i)+';\n'
 for n,i in a['labels'].items():h+='static constexpr unsigned ENTRY_'+n+'='+str(i)+';\n'
 if namespace in ['planner','ucompute']:
  if namespace=='planner':names=a['names']
  else:
   source=json.loads((D/'planner_program.json').read_text());names={k:a['maps']['planner'][str(v)]for k,v in source['names'].items()if str(v)in a['maps']['planner']}
  from kernel_interfaces import KERNELS
  hook=''
  for op,ns,keys,outputs in KERNELS:
   hook+='if(c[0]=='+str(OPS.index(op))+'){int64_t input['+str(len(keys))+']={'+','.join('mem['+str(names[k])+']'for k in keys)+'},output[4];unsigned elapsed='+ns+'_accel::run(*this,input,output);'
   hook+=''.join('mem['+str(names[k])+']=output['+str(i)+'];'for i,k in enumerate(outputs))
   hook+='cycles+=elapsed+'+str(4+len(keys)*2+len(outputs))+';continue;}\n'
  h=h.replace('if(c[0]==29||c[0]==30||c[0]==0)',hook+'if(c[0]==29||c[0]==30||c[0]==0)')
 h+='}\n';out=D/(namespace+'.hpp');out.write_text(h);return out
if __name__=='__main__':generate(D/'planner_program.json','planner')
