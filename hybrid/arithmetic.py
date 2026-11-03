"""Independent integer semantics used by the R17 ISA transaction tests."""
import math
W=48;Q=25;S=1<<Q;LO=-(1<<(W-1));HI=(1<<(W-1))-1
PI=round(math.pi*S);TAU=2*PI
# Quarter-wave sine, exp(-x) and tanh(x); 1025 entries each, linear interpolation.
TABLES=[[round(math.sin(i*math.pi/2048)*S) for i in range(1025)],
 [round(math.exp(-i/32)*S) for i in range(1025)],
 [round(math.tanh(i/128)*S) for i in range(1025)]]
def truncdiv(a,b):return (abs(a)//abs(b))*(-1 if (a<0)!=(b<0)else 1)
def lut(op,a):
 if op in ('SIN','COS'):
  if abs(a)>1024*S:raise ArithmeticError('trigonometric input bound')
  t=(a+(PI//2 if op=='COS'else 0))%TAU
  if t>PI:t-=TAU
  neg=t<0;t=abs(t)
  if t>PI//2:t=PI-t
  phase=(t*1024*256)//(PI//2);which=0
 elif op=='EXP':
  if a>0:raise ArithmeticError('EXP input must be nonpositive')
  phase=min(-a,32*S)*32*256//S;which=1;neg=False
 else:phase=min(abs(a),8*S)*128*256//S;which=2;neg=a<0
 idx=min(1024,phase>>8);f=phase&255;v=TABLES[which][idx]
 if idx<1024:v+=((TABLES[which][idx+1]-v)*f)>>8
 return -v if neg else v

def alu(op,a,b=0):
 if op=='MOV':r=a
 elif op=='ADD':r=a+b
 elif op=='SUB':r=a-b
 elif op=='MUL':r=(a*b)>>Q
 elif op=='DIV':
  if not b:raise ArithmeticError('divide by zero')
  r=truncdiv(a*S,b)
 elif op=='NEG':r=-a
 elif op=='ABS':r=abs(a)
 elif op=='MIN':r=min(a,b)
 elif op=='MAX':r=max(a,b)
 elif op in ('SIN','COS','EXP','TANH'):r=lut(op,a)
 elif op in ('LT','LE','GT','GE','EQ','NE'):r=int(dict(LT=a<b,LE=a<=b,GT=a>b,GE=a>=b,EQ=a==b,NE=a!=b)[op])*S
 elif op=='AND':r=int(bool(a)and bool(b))*S
 elif op=='OR':r=int(bool(a)or bool(b))*S
 elif op=='SHR':r=a>>(b//S)
 elif op=='SHL':r=a<<(b//S)
 else:raise ValueError(op)
 if not LO<=r<=HI:raise ArithmeticError('overflow')
 return r
