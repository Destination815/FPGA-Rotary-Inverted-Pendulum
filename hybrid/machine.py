"""R17 scheduled fixed-point ISA builder. All numbers signed Q25 in 48 bits.
No plant-state access. Instructions are source for both integer oracle and RTL.
"""
from contextlib import contextmanager
import json, math
from pathlib import Path
Q=25; W=48; SCALE=1<<Q
OPS=['HALT','MOV','ADD','SUB','MUL','DIV','NEG','ABS','MIN','MAX','SIN','COS','EXP','TANH','LT','LE','GT','GE','EQ','NE','AND','OR','JZ','JMP','CALL','RET','LOAD','STORE','YIELD','ENDFAST','ENDPLAN','SHR','SHL','MASS','EFFECTIVE','POLICY','COST']
class V:
 def __init__(self,p,n):self.p=p;self.n=n
 def op(self,o,b=None):return self.p.expr(o,self,b)
 def __add__(self,b):return self.op('ADD',b)
 def __sub__(self,b):return self.op('SUB',b)
 def __mul__(self,b):
  if isinstance(b,(int,float)) and b!=0:
   f,e=math.frexp(abs(b))
   if f==.5:
    v=self if e==1 else self.op('SHL' if e>1 else 'SHR',abs(e-1))
    return -v if b<0 else v
  return self.op('MUL',b)
 def __truediv__(self,b):return self.op('DIV',b)
 def __neg__(self):return self.op('NEG')
 def __abs__(self):return self.op('ABS')
 def __radd__(self,b):return self.p.val(b)+self
 def __rsub__(self,b):return self.p.val(b)-self
 def __rmul__(self,b):return self*b
 def __rtruediv__(self,b):return self.p.val(b)/self
 def __lt__(self,b):return self.op('LT',b)
 def __le__(self,b):return self.op('LE',b)
 def __gt__(self,b):return self.op('GT',b)
 def __ge__(self,b):return self.op('GE',b)
 def __and__(self,b):return self.op('AND',b)
 def __or__(self,b):return self.op('OR',b)
 def eq(self,b):return self.op('EQ',b)
 def ne(self,b):return self.op('NE',b)
 def set(self,b):self.p.emit('MOV',self,self.p.val(b));return self
 def minimum(self,b):return self.op('MIN',b)
 def maximum(self,b):return self.op('MAX',b)
 def clip(self,a,b):return self.maximum(a).minimum(b)
 def sin(self):return self.op('SIN')
 def cos(self):return self.op('COS')
 def exp(self):return self.op('EXP')
 def tanh(self):return self.op('TANH')
 def __bool__(self):raise TypeError('Use p.if_(), never Python truth on a runtime value')
class Program:
 def __init__(self):self.names={};self.init=[];self.code=[];self.consts={};self.labels={};self.fix=[];self.tmp=0
 def reg(self,name,value=0):
  if name in self.names:raise ValueError(name)
  n=len(self.init);self.names[name]=n;self.init.append(round(value*SCALE));return V(self,n)
 def array(self,name,size,values=None):return [self.reg(f'{name}[{i}]',0 if values is None else values[i])for i in range(size)]
 def val(self,x):
  if isinstance(x,V):return x
  raw=round(float(x)*SCALE)
  if raw not in self.consts:self.consts[raw]=self.reg(f'constant_{raw}',x)
  return self.consts[raw]
 def emit(self,op,d=0,a=0,b=0):
  self.code.append([OPS.index(op),*(v.n if isinstance(v,V)else v for v in (d,a,b))]);return len(self.code)-1
 def expr(self,op,a,b=None):
  r=self.reg(f'tmp{self.tmp}');self.tmp+=1;self.emit(op,r,self.val(a),self.val(0 if b is None else b));return r
 def mark(self,name):self.labels[name]=len(self.code)
 def jump(self,name,op='JMP',cond=0):self.fix.append((self.emit(op,0,cond),name))
 def call(self,name):self.jump(name,'CALL')
 @contextmanager
 def if_(self,condition):
  i=self.emit('JZ',0,condition);yield;self.code[i][1]=len(self.code)
 @contextmanager
 def loop(self,counter,start,stop):
  counter.set(start);head=len(self.code);i=self.emit('JZ',0,counter<stop);yield
  counter.set(counter+1);self.emit('JMP',head);self.code[i][1]=len(self.code)
 def load(self,base,index):
  out=self.reg(f'tmp{self.tmp}');self.tmp+=1;self.emit('LOAD',out,base,index);return out
 def store(self,base,index,value):self.emit('STORE',base,index,self.val(value))
 def finish(self,path):
  for i,name in self.fix:self.code[i][1]=self.labels[name]
  # Exact peephole: a single-use temporary immediately copied to its final
  # destination can be written there directly. No operation/order/rounding changes.
  reads={};targets={c[1]for c in self.code if c[0]in [22,23,24]and c[1]>=0}|{v for v in self.labels.values()if v>=0}
  for op,d,a,b in self.code:
   operands=[a] if op==22 else [] if op in [0,23,24,25,28,29,30] else [b] if op==26 else [a,b]
   for x in operands:reads[x]=reads.get(x,0)+1
  names={v:k for k,v in self.names.items()};removed=set()
  for i in range(len(self.code)-1):
   op,d,a,b=self.code[i];n=self.code[i+1]
   if op not in [0,22,23,24,25,27,28,29,30] and n[0]==1 and n[2]==d and reads.get(d)==1 and names[d].startswith('tmp') and i+1 not in targets:
    self.code[i][1]=n[1];removed.add(i+1)
  mapping={};kept=[]
  for i,c in enumerate(self.code):
   mapping[i]=len(kept)
   if i not in removed:kept.append(c)
  mapping[len(self.code)]=len(kept)
  for c in kept:
   if c[0]in [22,23,24]and c[1]>=0:c[1]=mapping[c[1]]
  self.labels={k:mapping[v]if v>=0 else v for k,v in self.labels.items()};self.code=kept
  used=set()
  for op,d,a,b in self.code:
   used.update([a,b])
   if op not in [22,23,24]:used.add(d)
  rm={};newinit=[]
  for name,old in list(self.names.items()):
   if name.startswith('tmp')and old not in used:del self.names[name];continue
   rm[old]=len(newinit);self.names[name]=len(newinit);newinit.append(self.init[old])
  for c in self.code:
   if c[0]not in [22,23,24]:c[1]=rm[c[1]]
   c[2]=rm[c[2]];c[3]=rm[c[3]]
  self.init=newinit
  # Liveness allocate temporary registers; persistent variables/arrays retain addresses.
  # Kept explicit initially: branch-safe compacting is a separate checked optimization.
  obj=dict(word_bits=W,fraction_bits=Q,ops=OPS,initial=self.init,names=self.names,code=self.code,labels=self.labels)
  Path(path).write_text(json.dumps(obj,separators=(',',':')));return obj
