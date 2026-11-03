"""Frozen v2 measured controller: integer Q18 reference and Verilog generator.

All run-time state uses signed32 Q18; products are signed64 rounded-nearest
(ties toward +infinity). Constants have independently chosen >=Q22 precision.
Time is exactly counted at1kHz. Inputs use radians/radians-second, not degrees.
"""
from pathlib import Path
import ast, math, json
Q=1<<18
ROOT=Path(__file__).resolve().parent
CFG=json.loads((ROOT/'selected_candidate_v2.json').read_text())
# Kept literal to freeze this controller independently of import path/sys.modules.
DZCOMP=.015
SIN=[round(math.sin(i/256)*Q) for i in range(806)]
COS=[round(math.cos(i/256)*Q) for i in range(806)]
TANH=[round(math.tanh(i/64)*Q) for i in range(513)]
def q(x): return round(x*Q)
def clip(x,lo,hi):return min(max(x,lo),hi)
def sign(x):return (x>0)-(x<0)
def sinlut(x):return sign(x)*SIN[clip((abs(x)+512)>>10,0,805)]
def coslut(x):return COS[clip((abs(x)+512)>>10,0,805)]
def tanhlut(x):return sign(x)*TANH[clip((abs(x)+2048)>>12,0,512)]
def const(x):
    frac=min(30, int(math.floor(math.log2((2**30-1)/abs(x)))) if x else 30)
    return round(x*(1<<frac)),frac
V=json.loads((ROOT/'velocity_candidate.json').read_text())
PROGRAM=[]
def a(name,expr):PROGRAM.append(('assign',name,str(expr)))
def m(name,left,right,shift=18):PROGRAM.append(('mul',name,str(left),str(right),shift))
def k(name,left,value):
    c,f=const(value);m(name,left,c,f)
# R16: do not brake an under-energy pass merely because it enters 30 degrees.
a('s','sinlut(theta)');a('c','coslut(theta)')
m('w2','omega','omega');k('ekin','w2',.5/94.50762226949593);a('catch_energy','ekin+c')
a('good',f"abs(theta)<{q(math.radians(V.get('capture_deg',8)))} and abs(omega)<{q(V.get('capture_omega',.7))} and abs(velocity)<{q(V.get('capture_velocity',1))} and abs(prevu)<=150")
a('ready','min(ready+1,30) if good else 0')
a('newcap','mode<2 and ready>=30')
a('mode','2 if newcap else mode')
a('reference','phi if newcap else reference')
a('vreq','velocity if newcap else vreq')
a('integral','0 if newcap else integral')
a('age','0 if newcap else age')
a('progress','min(progress,abs(theta)) if ticks>0 else abs(theta)')
a('fallback',f"mode==0 and (fallback!=0 or (ticks>=6000 and progress>{q(math.radians(60))}))")
a('newcatch',f"mode==0 and abs(theta)<{q(math.radians(V.get('catch_deg',20)))} and abs(omega)<{q(6)} and sign(theta)*sign(omega)<0 and catch_energy>={q(V.get('catch_energy',.98))}")
a('mode','1 if newcatch else mode');a('reference','phi if newcatch else reference')
a('vreq','velocity if newcatch else vreq');a('integral','0 if newcatch else integral');a('age','0 if newcatch else age')
a('release_flag',f"mode==1 and (abs(theta)>{q(math.radians(V.get('release_deg',36)))} or age>1000)")
a('mode','0 if release_flag else mode');a('vreq','velocity if release_flag else vreq');a('age','0 if release_flag else age')
a('limitq',f'{q(.15)} if mode>=2 else {q(.35)}');a('limitp','150 if mode>=2 else 350')
a('s','sinlut(theta)');a('c','coslut(theta)')
m('w2','omega','omega');k('ekin','w2',.5/94.50762226949593);a('energy',f'ekin+c-{q(1.1)}')
# Velocity-closed energy pump; selected in measured-response model family.
k('energy_early','energy',V['energy']);k('energy_late','energy',V['energy_late']);a('energyforce','energy_late if ticks>=6000 else energy_early');k('fallback_energy','energy',80);a('energyforce','fallback_energy if fallback else energyforce')
a('energyforce','energyforce if sign(omega)==sign(c) and omega!=0 and c!=0 else (-energyforce if omega!=0 and c!=0 else 0)')
k('center','phi',V['center']);k('vdamp','velocity',V['damping'])
a('accel',f"clip(energyforce-center-vdamp+({q(2)} if kick_sign>0 else -{q(2)}) if ticks<150 else energyforce-center-vdamp,-{q(V['acceleration'])},{q(V['acceleration'])})")
k('fallback_center','phi',3);a('accel',f'clip(energyforce-fallback_center-velocity,-{q(20)},{q(20)}) if fallback else accel')
k('dv','accel',.001);a('vbase',f"{q(V.get('late_vmax',V['vmax']))} if ticks>=6000 else {q(V['vmax'])}")
a('catch_bound',str(q(V['catch_vmax'])) if 'catch_vmax' in V else 'vbase')
a('vbound',f"catch_bound if mode==1 else (vbase if mode==0 else {q(.75)})")
a('vbound',f"{q(1.75)} if fallback else vbound")
a('vreq','clip(vreq+dv,-vbound,vbound) if mode==0 else vreq')
# Taper the requested speed at 44.72 degrees; measured speed is closed below.
k('arm_lo',f"-{q(V['bound'])}-phi",8);k('arm_hi',f"{q(V['bound'])}-phi",8)
k('fallback_lo',f'-{q(math.radians(50))}-phi',8);k('fallback_hi',f'{q(math.radians(50))}-phi',8);a('arm_lo','fallback_lo if fallback else arm_lo');a('arm_hi','fallback_hi if fallback else arm_hi')
a('arm_lo','clip(arm_lo,-vbound,vbound)');a('arm_hi','clip(arm_hi,-vbound,vbound)')
a('vreq','clip(vreq,arm_lo,arm_hi) if mode==0 else vreq')
# Frozen local balance during braking and capture.
k('leanp','phi-reference',-CFG['op']);k('leand','velocity',-CFG['od']);a('lean',f'clip(leanp+leand,-{q(.08)},{q(.08)})')
k('ap','theta-lean',CFG['kp']);k('ad','omega',CFG['kd']);a('accel',f"clip(ap+ad,-{q(V.get('catch_accel',20))},{q(V.get('catch_accel',20))})");k('dv','accel',.001)
a('vreq','clip(vreq+dv,-vbound,vbound) if mode==1 or mode==2 else vreq');a('ev','vreq-velocity')
k('di_swing','ev',V['ki']*.001);k('di_hold','ev',CFG['vi']*.001);a('di','di_swing')
a('ibound',f'{q(.2)} if mode<2 else {q(.1)}');a('trial','clip(integral+di,-ibound,ibound)')
k('feed_swing','vreq',V['feed']);k('feed_hold','vreq',.2);a('feed','feed_swing')
k('vp_swing','ev',V['kp']);k('vp_hold','ev',CFG['vp']);a('vp','vp_swing')
k('tanharg','vreq',12.5);a('tanhv','tanhlut(tanharg)');k('dead','tanhv',DZCOMP)
a('raw','feed+vp+trial+dead');a('integral','trial if not(abs(raw)>limitq and sign(raw)==sign(ev)) else integral')
M=.06;L=.075;R=.132;ALPHA=94.50762226949593;G=.647
J=M*9.81*L/ALPHA;C=M*R*L
m('s2','s','s');m('c2','c','c');m('sc','s','c');m('v2','velocity','velocity');m('vw','velocity','omega')
k('desired','vreq-velocity',50);a('desired',f'clip(desired,-{q(40)},{q(40)})')
k('inertia','s2',(J+C*C/J)/G);a('inertia',f'inertia+{q(.0016)}');m('inv1','inertia','desired')
k('inv2','sc',C*M*9.81*L/J/G)
m('tmp','s','c2');m('tmp','tmp','v2');k('inv3','tmp',C/G)
m('tmp','c','omega');k('inv4','tmp',C*.15/G)
k('inv5','velocity',.2)
m('tmp','sc','vw');k('inv6','tmp',2*J/G)
m('tmp','s','w2');k('inv7','tmp',C/G)
a('inverse','inv1+inv2+inv3-inv4+inv5+inv6-inv7');k('tanharg','inverse',G/.05);a('tanhv','tanhlut(tanharg)');k('invc','tanhv',.015)

a('u','clip(inverse+invc if fallback else feed+vp+integral+dead,-limitq,limitq)')
# Output quantization and slew are performed in permille. Round-nearest ties-even
# of fixed u*1000/Q, then clamp slew and magnitude. Integer slew boundaries make
# this equivalent to floating algorithm clip then round, except threshold ties.
m('scaled','u',1000,0)
a('rounded','(abs(scaled)>>18)+(1 if (abs(scaled)&262143)>131072 or ((abs(scaled)&262143)==131072 and ((abs(scaled)>>18)&1)==1) else 0)')
a('rounded','-rounded if scaled<0 else rounded');a('slew_low','prevu-20');a('slew_high','prevu+20')
a('slew_value','clip(rounded,slew_low,slew_high)')
a('limit_low','-limitp');a('limited','clip(slew_value,limit_low,limitp)')
a('prevu','0 if mode==3 else limited')
a('age','min(age+1,12001)');a('ticks','min(ticks+1,12001)')

COMPILED=[(op[0],op[1],compile(op[2],'<expr>','eval'),*( [compile(op[3],'<expr>','eval'),op[4]] if op[0]=='mul' else [])) for op in PROGRAM]

class IntegerSwing:
    def __init__(self,kick_sign=1,manual_capture=False,phi=0,velocity=0):
        self.reset(kick_sign,manual_capture,phi,velocity)
    def reset(self,kick_sign=1,manual_capture=False,phi=0,velocity=0):
        self.e={name:0 for _,name,*_ in PROGRAM};self.e.update(theta=0,phi=0,omega=0,velocity=0,kick_sign=1 if kick_sign>=0 else -1,mode=2 if manual_capture else 0,reference=phi if manual_capture else 0,vreq=velocity if manual_capture else 0)
        self.fault=False
    @property
    def mode(self):return self.e['mode']
    @property
    def reference(self):return self.e['reference']
    def stop(self):self.e['mode']=3;self.e['prevu']=0
    def update(self,theta,phi,omega,velocity):
        self.e.update(theta=int(theta),phi=int(phi),omega=int(omega),velocity=int(velocity))
        if self.mode==3:return 0
        # Physical domain enforced ahead of arithmetic to avoid any 32-bit wrap.
        if abs(theta)>q(math.pi+.02) or abs(phi)>q(4) or abs(omega)>q(40) or abs(velocity)>q(40):
            self.fault=True;self.stop();return 0
        g={'__builtins__':{},'abs':abs,'min':min,'max':max,'clip':clip,'sign':sign,'sinlut':sinlut,'coslut':coslut,'tanhlut':tanhlut}
        for op in COMPILED:
            if op[0]=='assign':value=int(eval(op[2],g,self.e))
            else:
                _,name,left,right,shift=op
                prod=eval(left,g,self.e)*eval(right,g,self.e)
                value=(prod+(1<<(shift-1)))>>shift if shift else prod
            if not -2**31<=value<2**31: self.fault=True;self.stop();return 0
            self.e[op[1]]=value
        return self.e['prevu']

# Tiny Python-expression->Verilog compiler. Program above is single source of
# executable specification and scheduled RTL arithmetic; no division operators.
def verilog(s):
    def v(n):
        if isinstance(n,ast.Constant):return f"32'sd{n.value}" if n.value>=0 else f"-32'sd{-n.value}"
        if isinstance(n,ast.Name):return n.id
        if isinstance(n,ast.BinOp):return '('+v(n.left)+' '+{ast.Add:'+',ast.Sub:'-',ast.Mult:'*',ast.RShift:'>>>',ast.LShift:'<<<',ast.BitAnd:'&'}[type(n.op)]+' '+v(n.right)+')'
        if isinstance(n,ast.UnaryOp):return '('+{ast.USub:'-',ast.Not:'!',ast.UAdd:'+'}[type(n.op)]+v(n.operand)+')'
        if isinstance(n,ast.BoolOp):return '('+(' && ' if isinstance(n.op,ast.And) else ' || ').join(v(x) for x in n.values)+')'
        if isinstance(n,ast.Compare):
            assert len(n.ops)==1
            return '('+v(n.left)+' '+{ast.Lt:'<',ast.LtE:'<=',ast.Gt:'>',ast.GtE:'>=',ast.Eq:'==',ast.NotEq:'!='}[type(n.ops[0])]+' '+v(n.comparators[0])+')'
        if isinstance(n,ast.IfExp):return '(('+v(n.test)+') != 0 ? '+v(n.body)+' : '+v(n.orelse)+')'
        if isinstance(n,ast.Call):
            name=n.func.id;args=[v(x) for x in n.args]
            if name=='abs':return f'(({args[0]} < 0) ? -({args[0]}) : ({args[0]}))'
            if name=='sign':return f'(({args[0]} > 0) ? 32\'sd1 : (({args[0]} < 0) ? -32\'sd1 : 32\'sd0))'
            if name=='clip':return f'(({args[0]} < {args[1]}) ? {args[1]} : (({args[0]} > {args[2]}) ? {args[2]} : {args[0]}))'
            if name=='min':return f'(({args[0]} < {args[1]}) ? {args[0]} : {args[1]})'
            return name+'('+','.join(args)+')'
        raise ValueError(ast.dump(n))
    return v(ast.parse(s,mode='eval').body)

def generate():
    from compact_rtl import generate as compact_generate
    return compact_generate()
if __name__=='__main__':print('Generated instructions',generate())
