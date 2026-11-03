"""Test the exact registered lookup stages extracted from production RTL."""
from pathlib import Path
import re,random,math,subprocess,json
P=Path(__file__).resolve().parents[1];B=P/'build/lookup';B.mkdir(parents=True,exist_ok=True)
s=(P/'fpga/src/r10_observer_core.v').read_text();decl=s[s.index('reg [31:0] lut_abs;'):s.index('always @(posedge clk')]
ops=[]
for i in range(1,8):
 t=re.search(r"9'd"+str(i)+r':begin (.*?) end\n',s).group(1).replace('z3','value').replace('tf_old','answer');ops.append(t)
rtl='module lookup(input clk,input signed[31:0] value,input[3:0] step,output reg signed[31:0] answer);\n'+decl+'always @(posedge clk)case(step)\n'+'\n'.join(f"4'd{i+1}:begin {v} end"for i,v in enumerate(ops))+'\ndefault:begin end\nendcase\nendmodule\n'
(B/'lookup.v').write_text(rtl)
v={0,1,-1,2**31-1,-2**31}
# Every table bin and both sides of every round-to-nearest transition.
for i in range(1026):
 threshold=(i*8192-4096)//5
 for delta in range(-2,4):
  for sign in [-1,1]:v.add(sign*(threshold+delta))
r=random.Random(61006);v.update(r.randrange(-2**31,2**31)for _ in range(20000));v=sorted(x for x in v if -2**31<=x<2**31)
table=[round(math.tanh(i/1024/.1)*(1<<24))for i in range(1025)]
rows=[]
for x in v:
 old=(abs(x)*10240+8388608)>>24;new=(abs(x)*5+4096)>>13;assert old==new
 expected=table[old]if old<=1024 else 1<<24;rows.append(f'{x} {expected if x>=0 else -expected}')
(B/'vectors').write_text('\n'.join(rows))
(B/'test.cpp').write_text('''#include "Vlookup.h"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){Vlookup t;std::ifstream f(argv[1]);long long v,e;int n=0;while(f>>v>>e){t.value=v;for(int j=1;j<=7;j++){t.step=j;t.clk=0;t.eval();t.clk=1;t.eval();if(j==1)t.value=123456789;}if(int32_t(t.answer)!=e){std::cerr<<v<<" mismatch\\n";return 1;}n++;}std::cout<<"PASS exact table transactions="<<n<<" seven registered stages and snapshotted sign/value\\n";}''')
with (B/'build.log').open('w')as f:subprocess.run(['verilator','--cc','--exe','--build','-j','2','--Mdir',str(B/'obj'),'--top-module','lookup',str(B/'lookup.v'),str(B/'test.cpp')],stdout=f,stderr=subprocess.STDOUT,check=True)
a=subprocess.check_output([str(B/'obj/Vlookup'),str(B/'vectors')],text=True);print(a);(P/'evidence/lookup_pipeline.json').write_text(json.dumps(dict(status='PASS',transactions=len(v),result=a,table_values_unchanged=True),indent=2))
