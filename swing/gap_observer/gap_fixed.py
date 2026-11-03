"""Q24 gap-state bridge, fixed nominal measured dynamics, no plant state input.
Normal AB updates remain the T03 algorithm in state_estimator_gap.v.
"""
from pathlib import Path
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swing_fixed import verilog,const,clip,sign,Q
# Gap bridge uses its original finer 1/1024 rad ROM, independent of swing VM LUT.
SIN=[round(math.sin(i/1024)*(1<<18)) for i in range(3219)]
COS=[round(math.cos(i/1024)*(1<<18)) for i in range(3219)]
def sinlut(x): return (-1 if x<0 else 1)*SIN[min(3218,(abs(x)+128)>>8)]
def coslut(x): return COS[min(3218,(abs(x)+128)>>8)]
ROOT=Path(__file__).resolve().parent
PI=205887*256;TWO_PI=411775*256;SCALE=386*256
PROGRAM=[]
def a(n,e):PROGRAM.append(('assign',n,str(e)))
def m(n,l,r,shift=18):PROGRAM.append(('mul',n,str(l),str(r),shift))
def k(n,l,c):cc,s=const(c);m(n,l,cc,s)
def wrap(n,e):a(n,e);a(n,f'{n}-{TWO_PI} if {n}>{PI} else ({n}+{TWO_PI} if {n}<-{PI} else {n})')
def phase(n,e):a(n,e);a(n,f'{n}-{TWO_PI} if {n}>={TWO_PI} else ({n}+{TWO_PI} if {n}<0 else {n})')
m('offset','zero',50593792,13);wrap('pred','position+velocity');phase('phase','pred+offset')
a('needgap',f'(allow!=0 and initialized!=0 and (phase>{1016*SCALE} or phase<{7*SCALE})) or age>0 or (allow!=0 and initialized!=0 and episode_age>0 and clean_age<5 and (adc<=3 or adc>=1020))')
a('entry','age==0 and needgap!=0');a('age','min(age+1,181) if needgap!=0 else 0')
a('theta','position>>6');a('s','sinlut(theta)');a('c','coslut(theta)')
k('omega','velocity',1000/64);a('armv','current_v<<2');a('darmv','(current_v-previous_v)<<2')
m('sc','s','c');m('v2','armv','armv');m('centrif','sc','v2');k('grav','s',94.50762226949593*.001);k('centrif','centrif',.001)
m('coupled','c','darmv');k('coupled','coupled',1.27166)
k('damp','omega',.15*.001);a('dw','grav+centrif-damp-coupled');a('clean_age','min(clean_age+1,29) if needgap==0 else clean_age')
k('lag_n','dw',.5)
m('lagfix','lag_n','clean_age',0)
a('clean_age','0 if needgap!=0 else clean_age')
# Entry latency compensation is a model-based correction, not a measured velocity.
# Carry its magnitude as bounded velocity uncertainty through this gap.
k('lag_unc_candidate','abs(lagfix)',.064)
a('lag_unc_per_ms','(lag_unc_candidate if lag_unc_candidate>lag_unc_per_ms else lag_unc_per_ms) if entry!=0 and episode_age>0 else (lag_unc_candidate if entry!=0 else lag_unc_per_ms)')
a('omega',f'clip(omega+lagfix,-{30*Q},{30*Q}) if entry!=0 else omega')
k('damp','omega',.15*.001);a('dw','grav+centrif-damp-coupled');a('wn',f'clip(omega+dw,-{30*Q},{30*Q})')
k('dn','omega+dw/2',.064) if False else None
# Divide-by2 arithmeticshift matchesRTL; <0.000002rad/s truncationeffect.
k('dn','omega+(dw>>1)',.064);wrap('pn','position+dn');k('vn','wn',.064)
phase('phase_n','pn+offset');wrap('err','(wrapped<<8)-pn')
a('unc_age','min(episode_age+1,181)')
m('age2','unc_age','unc_age',0)
k('unc_v','unc_age',.003*(1<<24))
k('unc_a','age2',.000025*(1<<24))
m('unc_lag','lag_unc_per_ms','unc_age',0)
a('unc',f'clip({4*SCALE}+unc_v+unc_a+unc_lag,{23*SCALE},{round(.35*(1<<24))})')
# Expanded re-acquisition requires five continuous measured samples, physical motion,
# matching direction, sector-edge location, and bounded prediction innovation.
wrap('raw_step','(wrapped-raw1)<<8')
wrap('raw_motion','(wrapped-raw4)<<8')
a('return_good',f'needgap!=0 and adc>3 and adc<1020 and abs(raw_step)<{round(.04*(1<<24))} and abs(err)<{round(.35*(1<<24))}')
a('return_age','min(return_age+1,5) if return_good!=0 else 0')
a('return_pending','5 if return_good!=0 else (return_pending-1 if return_pending>0 and needgap!=0 else 0)')
a('return_guard',f'return_pending>0 and abs(err)<{round(.35*(1<<24))}')
a('measured_exit',f'return_age>=5 and age<=180 and (adc<96 or adc>928) and abs(raw_motion)>{SCALE} and sign(raw_motion)==sign(vn)')
a('raw4','raw3');a('raw3','raw2');a('raw2','raw1');a('raw1','wrapped')
a('exitgood',f'needgap!=0 and adc>7 and adc<1016 and phase_n>{8*SCALE} and phase_n<{1015*SCALE} and abs(err)<{round(.12*(1<<24))}')
# OriginalT03 signed half-away correction error/64, preservingQ24rad/ms.
# Keep normal causal correction; rail bounce after provisional exit re-enters
# the same bounded episode until five clean samples have been seen.
a('exitgood','exitgood!=0 or measured_exit!=0')
a('correction','((abs(err)+32)>>6) if err>=0 else -((abs(err)+32)>>6)')
a('pout','pn+(48*correction if measured_exit!=0 else 15*correction) if exitgood!=0 else pn');a('vout',f'clip(vn+correction,-503316,503316) if exitgood!=0 else vn')
a('return_dv',f'clip((raw_motion>>2)-vn,-{round(.002*(1<<24))},{round(.002*(1<<24))})')
a('vout','clip(vn+(return_dv>>1),-503316,503316) if measured_exit!=0 else vout')
a('episode_age','min(episode_age+1,181) if needgap!=0 or (episode_age>0 and clean_age<5) else 0')
a('age','0 if exitgood!=0 else age');a('ok',f'not(needgap!=0 and (allow==0 or age>180 or episode_age>180 or (exitgood==0 and return_guard==0 and phase_n>unc and phase_n<{1016*SCALE}-unc)))')
# R20 v2: minimum correction only when the original dynamic refusal would fire.
S=SCALE; LIMIT=round(.35*(1<<24))
pr=PROGRAM
# Track valid edge evidence independently from predicted sector entry.
pre=[('assign','edge_age',f'0 if adc>3 and adc<1020 and (adc<32 or adc>992) else min(edge_age+1,6)')]
i=next(i for i,o in enumerate(pr)if o[1]=='entry');pr[i+1:i+1]=[('assign','attested','1 if entry!=0 and edge_age<=5 else (0 if needgap==0 and episode_age==0 else attested)'),('assign','project_total','0 if entry!=0 and episode_age==0 else project_total')]
# Delay innovation calculation until *after* the dynamic uncertainty and minimal
# projection. Keep the same original uncertainty budget and refusal interval.
i=next(i for i,o in enumerate(pr)if o[1]=='err')
err_ops=pr[i:i+2];del pr[i:i+2]
i=next(i for i,o in enumerate(pr)if o[1]=='unc')+1
extra=[
 ('assign','project_high_bound',f'{1016*S}-unc'),
 ('assign','project_low_delta','unc-phase_n'),
 ('assign','project_high_delta','project_high_bound-phase_n'),
 ('assign','project_delta',f'(project_low_delta if phase_n<{PI} else project_high_delta) if needgap!=0 and attested!=0 and (adc<=3 or adc>=1020) and (phase_n<{96*S} or phase_n>{928*S}) and phase_n>unc and phase_n<project_high_bound else 0'),
 ('assign','project_abs','abs(project_delta)'),
 ('assign','project_total','project_total+project_abs'),
 ('assign','pn',f'pn+project_delta if project_total<={LIMIT} else pn'),
 ('assign','pn',f'pn-{TWO_PI} if pn>{PI} else (pn+{TWO_PI} if pn<-{PI} else pn)'),
 ('assign','phase_n',f'phase_n+project_delta if project_total<={LIMIT} else phase_n')]
pr[i:i]=extra+err_ops;pr[:0]=pre
# Measurement re-acquisition correction can itself cross the signed angle seam.
i=next(i for i,o in enumerate(pr)if o[1]=='pout')+1
pr[i:i]=[('assign','pout',f'pout-{TWO_PI} if pout>{PI} else (pout+{TWO_PI} if pout<-{PI} else pout)')]
# Preserve dynamic innovation rejection for unattested episodes; projection only
# receives <=0.35rad lifetime correction. No timer / output limit modification.
pr += [('assign','ok',f'ok!=0 and project_total<={LIMIT}')]


# R21: reacquire from eight continuous, moving edge observations even when
# model phase has drifted moderately. No rail sample qualifies; 180ms remains.
i=next(i for i,o in enumerate(PROGRAM)if o[1]=='raw4')
PROGRAM[i:i]=[
 ('assign','edge_good',f'needgap!=0 and attested!=0 and age<=180 and adc>7 and adc<1016 and (adc<96 or adc>928) and abs(raw_step)<{round(.04*(1<<24))} and abs(err)<{round(.6*(1<<24))} and abs(raw_motion)>{SCALE} and sign(raw_motion)==sign(vn)'),
 ('assign','edge_confirm','min(edge_confirm+1,8) if edge_good!=0 else 0'),
 ('assign','measured_exit','measured_exit!=0 or edge_confirm>=8')]

# R22: allow an attested returning edge to supply its consecutive evidence.
# This countdown is per blind episode, never renewable by intermittent noise;
# no measurement correction occurs until the existing exit test succeeds.
i=next(i for i,o in enumerate(pr)if o[1]=='ok')
pr[i:i]=[
 ('assign','edge_candidate',f'needgap!=0 and attested!=0 and age<=180 and adc>7 and adc<1016 and (adc<96 or adc>928) and abs(err)<{round(.6*(1<<24))}'),
 ('assign','reacquire_window','min(reacquire_window+1,10) if needgap!=0 and (edge_candidate!=0 or reacquire_window>0) else 0'),
 ('assign','return_guard','return_guard!=0 or (edge_candidate!=0 and reacquire_window<10)')]

COMPILED=[(op[0],op[1],compile(op[2],'<gap>','eval'),*([compile(op[3],'<gap>','eval'),op[4]] if op[0]=='mul' else [])) for op in PROGRAM]
class GapFixed:
 def __init__(self):self.state={k:0 for k in ['age','clean_age','raw1','raw2','raw3','raw4','return_age','lag_unc_per_ms','episode_age','return_pending','edge_age','attested','project_total','project_delta','edge_confirm','reacquire_window']};self.state['edge_age']=6;self.age=0;self.clean_age=0
 def update(self,position,velocity,current_v,previous_v,adc,zero,wrapped,initialized,allow):
  e=dict(position=position,velocity=velocity,current_v=current_v,previous_v=previous_v,adc=adc,zero=zero,wrapped=wrapped,initialized=int(initialized),allow=int(allow),age=self.age,clean_age=self.clean_age)
  e.update(self.state)
  g={'__builtins__':{},'abs':abs,'min':min,'max':max,'clip':clip,'sign':sign,'sinlut':sinlut,'coslut':coslut}
  for op in COMPILED:
   if op[0]=='assign':value=int(eval(op[2],g,e))
   else:
    prod=eval(op[2],g,e)*eval(op[3],g,e);sh=op[4];value=(prod+(1<<(sh-1)))>>sh if sh else prod
   if not -2**31<=value<2**31:raise OverflowError((op,value))
   e[op[1]]=value
  self.state={k:e[k] for k in self.state};self.age=e['age'];self.clean_age=e['clean_age'];return e

def generate():
 regs=sorted({x[1] for x in PROGRAM}|{'position','velocity','current_v','previous_v','adc','zero','wrapped','initialized','allow'})
 lines=['// Generated measured-only blind-sector bridge; integerQ24state.',
'module r01_gap_bridge(input wire clk,rst_n,clear,start,gap_allow,ab_initialized,',
'input wire signed[31:0] ab_position,ab_velocity,current_arm_velocity,previous_arm_velocity,raw_wrapped,',
'input wire[9:0] adc_raw,input wire[13:0] zero_q4,output reg busy,done,use_gap,gap_active,measurement_ok,',
'output reg signed[31:0] position_out,velocity_out,output reg[7:0] gap_ms);',
'reg [9:0] pc;reg signed[31:0] mul_a,mul_b;reg signed[63:0] mul_product,mul_rounded,mul_bias;reg[5:0] mul_shift;']
 # Reachable counter ranges: 0..181, 0..29 and 0..5; signed widths retain comparisons.
 widths={'age':9,'episode_age':9,'unc_age':9,'clean_age':6,'return_age':4,'return_pending':4,'edge_age':4,'attested':2,'edge_confirm':5,'reacquire_window':5}
 lines += [f'reg signed[{widths.get(r,32)-1}:0] {r};' for r in regs]
 for name,values in [('sinrom',SIN),('cosrom',COS)]:
  lines += [f'function signed [31:0] {name}; input [11:0] idx; begin case(idx)']
  lines += [f"12'd{i}: {name}={v};" for i,v in enumerate(values)]
  lines += [f'default:{name}={values[-1]};endcase end endfunction']
 for name,rom,sgn in [('sinlut','sinrom',True),('coslut','cosrom',False)]:
  lines += [f'function signed[31:0] {name};input signed[31:0] x;reg signed[31:0] ix,y;begin ix=((x<0 ? -x:x)+128) >>> 8;if(ix>3218)ix=3218;y={rom}(ix[11:0]);{name}='+('x<0 ? -y:y;' if sgn else 'y;')+'end endfunction']
 lines += ['always @(posedge clk or negedge rst_n) begin','if(!rst_n) begin busy<=0;done<=0;pc<=0;use_gap<=0;gap_active<=0;measurement_ok<=0;gap_ms<=0;position_out<=0;velocity_out<=0;mul_a<=0;mul_b<=0;mul_product<=0;mul_rounded<=0;mul_bias<=0;mul_shift<=0;']
 lines += [f'{r}<={6 if r=="edge_age" else 0};' for r in regs]
 lines += ['end else begin done<=0;mul_product<=mul_a*mul_b;mul_rounded<=(mul_product+mul_bias) >>> mul_shift;',
'if(clear)begin reacquire_window<=0;edge_confirm<=0;edge_age<=6;attested<=0;project_total<=0;project_delta<=0;busy<=0;pc<=0;age<=0;clean_age<=0;raw1<=0;raw2<=0;raw3<=0;raw4<=0;return_age<=0;lag_unc_per_ms<=0;episode_age<=0;return_pending<=0;use_gap<=0;gap_active<=0;measurement_ok<=0;gap_ms<=0;end',
'else if(start && !busy)begin busy<=1;pc<=0;position<=ab_position;velocity<=ab_velocity;current_v<=current_arm_velocity;previous_v<=previous_arm_velocity;adc<=$signed({22\'d0,adc_raw});zero<=$signed({18\'d0,zero_q4});wrapped<=raw_wrapped;initialized<=ab_initialized ? 1:0;allow<=gap_allow ? 1:0;end',
'else if(busy)begin case(pc)']
 pc=0
 for op in PROGRAM:
  if op[0]=='assign':
   expr=verilog(op[2]);name=op[1]
   if name in('needgap','entry','exitgood','ok','return_good','return_guard','measured_exit'):expr=f"{expr} ? 32'sd1:32'sd0"
   lines += [f"10'd{pc}:begin {name}<={expr};pc<=10'd{pc+1};end"];pc+=1
  else:
   _,name,l,r,sh=op
   lines += [f"10'd{pc}:begin mul_a<={verilog(l)};mul_b<={verilog(r)};mul_shift<=6'd{sh};mul_bias<=64'sd{(1<<(sh-1)) if sh else 0};pc<=10'd{pc+1};end",f"10'd{pc+1}:pc<=10'd{pc+2};",f"10'd{pc+2}:pc<=10'd{pc+3};",f"10'd{pc+3}:begin {name}<=mul_rounded[31:0];pc<=10'd{pc+4};end"]
   pc+=4
 lines += [f"10'd{pc}:begin busy<=0;done<=1;pc<=0;use_gap<=needgap!=0;gap_active<=age!=0;gap_ms<=age[7:0];measurement_ok<=ok!=0;position_out<=pout;velocity_out<=vout;end",'default:begin busy<=0;done<=1;measurement_ok<=0;end','endcase end end end endmodule']
 # R39T1 storage-only transform; exact original table/phase contract is checked.
 sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/r39t1_gap'))
 from gap_prefetch import prefetch_sine
 (ROOT/'r01_gap_bridge.v').write_text(prefetch_sine('\n'.join(lines)+'\n'));return pc+1
if __name__=='__main__':print(generate())

class FixedObserver:
 """Bit-level T03 estimator + bridge. Output dict fields are Q16 except filteredQ18."""
 def __init__(self,zero=746):
  self.zero=zero;self.enczero=0;self.last_phi=self.last_theta=0;self.initialized=self.previous_good=self.ab_initialized=False
  self.history=[0]*16;self.index=self.fill=0;self.position=self.velocity=self.dphi=0;self.bridge=GapFixed();self.outputs={}
 def calibrate(self,encoder=0):self.__init__(self.zero);self.enczero=encoder
 def update(self,adc,encoder,gap_allow=True,adc_ready=True):
  wrap16=lambda v:v-411775 if v>205887 else(v+411775 if v< -205887 else v)
  wrap24=lambda v:v-TWO_PI if v>PI else(v+TWO_PI if v< -PI else v)
  rawgood=adc_ready and 3<adc<1020
  wrapped=wrap16((adc-self.zero)*386);phi=clip((encoder-self.enczero)*396,-2000000,2000000)
  pred=wrap24(self.position+self.velocity);delta=wrap16(wrapped-self.last_theta);err=wrap24((wrapped<<8)-pred)
  full=self.fill>=16;phidelta=phi-(self.history[self.index] if full else self.last_phi)
  product=phidelta*(125 if full else 1000)
  rate=int(abs(product)//2)*sign(product) if full else product;rate=clip(rate,-1966080,1966080)
  dn=self.dphi+((rate-self.dphi)>>2);jump=-9830<delta<9830
  corr=sign(err)*((abs(err)+32)>>6);vn=clip(self.velocity+corr,-503316,503316);pn=pred+15*corr
  b=self.bridge.update(self.position,self.velocity,dn,self.dphi,adc,self.zero*16,wrapped,self.ab_initialized,gap_allow and adc_ready)
  if b['needgap']:
   position=b['pout'];velocity=b['vout'];abi=bool(b['ok']);filtered=position>>6;theta=position>>8;measurement_ok=bool(b['ok']);previousgood=bool(b['ok'])
   dtheta=sign(velocity)*((abs(velocity)*1000+128)>>8) if self.initialized else 0
  else:
   if not self.ab_initialized or not rawgood or not jump:position=wrapped<<8;velocity=0;abi=rawgood;filtered=wrapped<<2
   else:position=pn;velocity=vn;abi=self.ab_initialized;filtered=pn>>6
   theta=wrapped;measurement_ok=rawgood and self.initialized and jump;previousgood=rawgood and(jump or not self.initialized)
   dtheta=sign(vn)*((abs(vn)*1000+128)>>8) if self.initialized and rawgood and self.previous_good and jump and self.ab_initialized else 0
  dphi=dn if self.initialized else 0
  self.history[self.index]=phi;self.index=(self.index+1)%16;self.fill=min(16,self.fill+1)
  self.position=position;self.velocity=velocity;self.ab_initialized=abi;self.initialized=True;self.previous_good=previousgood;self.last_phi=phi;self.last_theta=theta;self.dphi=dphi
  self.outputs=dict(theta=theta,dtheta=dtheta,phi=phi,dphi=dphi,filtered_theta_q18=filtered,measurement_ok=bool(measurement_ok),gap_active=self.bridge.age>0,gap_ms=self.bridge.age)
  return self.outputs
