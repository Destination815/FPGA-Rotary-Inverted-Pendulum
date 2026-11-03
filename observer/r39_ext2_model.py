"""Independent integer transaction oracle; arbitrary precision Python sums.
32-bit state saturation, Q24 product truncation, LUT and ties-to-even output.
"""
from pathlib import Path
import json,math
C=json.loads((Path(__file__).with_name('constants.json')).read_text());Q=C['Q'];LUT=[round(math.tanh(i/1024/.1)*Q)for i in range(1025)]
def sat(x):return max(-2**31,min(2**31-1,x))
def lut(x):
 i=(abs(x)*10240+Q//2)>>24
 return (1 if x>=0 else -1)*(LUT[i]if i<1025 else Q)
LOWGAIN=json.loads(Path(__file__).with_name('r39_r22_feedback_profile.json').read_text())
BASE8=json.loads(Path(__file__).with_name('r39_disturbance_baseline.json').read_text())
def smooth_round_div(value,denominator):
 magnitude=abs(value);quotient,remainder=divmod(magnitude,denominator)
 quotient+=int(2*remainder>denominator or (2*remainder==denominator and quotient&1))
 return -quotient if value<0 else quotient
class Observer:
 def __init__(self,disturbance_observer=False,disturbance_regulator=False,config=None,disturbance_feedforward=False,manual_low_gain=False,manual_encoder_feedback=False,manual_filtered_motion=False,manual_output_shape=False,shape_profile=None,manual_damped_feedback=False,manual_sine_feedforward=False,manual_sine_damping=False,manual_smooth_velocity=False,manual_smooth_continuity=False):
  self.manual_smooth_continuity=manual_smooth_continuity
  self.manual_smooth_velocity=manual_smooth_velocity
  self.manual_sine_damping=manual_sine_damping
  self.manual_sine_feedforward=manual_sine_feedforward
  self.manual_damped_feedback=manual_damped_feedback
  self.manual_output_shape=manual_output_shape
  self.shape_profile=shape_profile or dict(source='feedback',fast_shift=4,slow_shift=6,gain_shift=2,bound_q24=671088)
  self.manual_filtered_motion=manual_filtered_motion
  self.manual_encoder_feedback=manual_encoder_feedback
  self.manual_low_gain=manual_low_gain
  self.config=config or (json.loads(Path(__file__).with_name('r39_disturbance_constants.json').read_text()) if disturbance_observer else C);self.compensate=(disturbance_regulator or disturbance_feedforward) and len(self.config['matrix'])==8;self.clear()
 def clear(self):self.smooth_position=0;self.smooth_velocity_step=0;self.smooth_valid=False;self.smooth_velocity_normalized=0;self.smooth_previous_feedback=0;self.smooth_correction=0;self.smooth_bound=0;self.smooth_innovation=0;self.smooth_sine_active=False;self.sine_bias=0;self.sine_valid=False;self.sine_position_ff=0;self.sine_reference=0;self.shape_fast=0;self.shape_slow=0;self.shape_valid=False;self.shape_correction=0;self.raw_request=0;self.shaped_request=0;self.feedback_lp=0;self.drift_latched=False;self.blend=0;self.integral=0;self.lp=0;self.age=0;self.bias_clipped=False;self.z=[0]*len(self.config['matrix']);self.samples=0;self.fault=0;self.ready=0
 def tick(self,t,p,previous,cap,reference_offset=0,reference_velocity=0,automatic=False,hold=True,fallback=0,brake=False,motion=False,hold_integral=False,velocity_ff_trim=False,motion_position_boost=False,sine_tracking=False,sine_reference=0,prediction_previous=None,output_slew=20):
  C=BASE8 if automatic and len(self.z)==8 else self.config
  reference_velocity=(((reference_velocity<<8)+2**31)%2**32-2**31)>>8
  sine_active=bool(self.manual_sine_feedforward and self.manual_low_gain and not automatic and motion and sine_tracking)
  if self.fault:return 0,self.ready,1
  if abs(t)>20000 or abs(p)>823550 or abs(previous)>350 or not 150<=cap<=350 or abs(reference_offset)>823550 or (sine_active and not -(1<<23)<=sine_reference<(1<<23)):self.fault=1;return 0,self.ready,1
  prior=(previous if prediction_previous is None else prediction_previous)*16777;e=max(0,prior-C['dead'])if prior>=0 else min(0,prior+C['dead']);e=sat((e*(C['ratio']if e<0 else Q))>>24)
  old=self.z.copy()
  if len(old)==8 and automatic:old[7]=0
  v=old+[e,lut(old[3]),t<<8,p<<8];n=[sat(sum((a*b)>>24 for a,b in zip(v,row)))for row in C['matrix']]
  if len(n)==8:
   bound=C.get('bias_bound_q24',5033165);self.bias_clipped=not automatic and abs(n[7])>bound;n[7]=0 if automatic else max(-bound,min(bound,n[7]))
  self.drift_latched=self.drift_latched or (automatic and abs(p-reference_offset)>5719)
  drift=automatic and self.drift_latched
  oldlp=self.lp;self.lp=sat(self.lp+(((p<<8)-(reference_offset<<8)-self.lp)*(133682 if drift else 16760)>>24));self.lp=(oldlp if (automatic or abs(reference_offset)>2288) and abs(reference_velocity)<6554 else 0) if not sine_active and not drift and (abs(n[1])>838861 or abs(n[3])>838861) else self.lp
  if motion and not sine_active and not automatic and ((p>=reference_offset and oldlp<0) or (p<=reference_offset and oldlp>0)):self.lp=0
  istep=self.lp*(1048 if drift or sine_active else 4194)>>24
  low=self.manual_low_gain and not automatic
  gain=(([82569382,58376238,5555072,45579777,-388103,-3056810,-3363586] if brake else [67937104,38753879,3568142,30194271,-268429,-2104864,-2148228]) if automatic else (LOWGAIN['gain'].copy()if low else C['gain'].copy()))
  if not automatic and (abs(n[1])>2516582 or abs(n[0])>1464088):gain[0]=LOWGAIN['theta_fast_gain']if low else C['theta_fast_gain']
  if self.manual_damped_feedback and low:gain[0]=74306208
  if sine_active:gain[1]=58130819
  pe=((n[2]-(reference_offset<<8)+2**31)%2**32)-2**31
  operand=sat(pe+(max(-5856353,min(5856353,pe))>>3))if motion_position_boost and motion and not automatic else pe
  feedback_v=sat(n[3]+max(-335544,min(335544,1*((p<<8)-n[2]))))if self.manual_encoder_feedback and low else n[3]
  if self.manual_filtered_motion and self.manual_encoder_feedback and low:
   target=max(-335544,min(335544,(2 if motion else 1)*((p<<8)-n[2])))
   self.feedback_lp=self.feedback_lp+((target-self.feedback_lp)>>2)if motion else target
   feedback_v=sat(n[3]+self.feedback_lp)
  else:self.feedback_lp=0
  if self.manual_smooth_velocity and low and self.manual_encoder_feedback:
   old_feedback=feedback_v
   self.smooth_previous_feedback=old_feedback
   self.smooth_bound=(167772 if self.manual_smooth_continuity else min(167772,abs(reference_velocity<<8)>>6)) if sine_active else 0
   if not self.smooth_valid:
    self.smooth_position=p<<8
    self.smooth_velocity_step=smooth_round_div(old_feedback,100)
    self.smooth_innovation=0
    self.smooth_velocity_normalized=sat(100*self.smooth_velocity_step)
    self.smooth_correction=0
   else:
    prediction=self.smooth_position+self.smooth_velocity_step
    residual=(p<<8)-prediction
    innovation=residual-50688 if residual>50688 else residual+50688 if residual< -50688 else 0
    self.smooth_position=sat(prediction+smooth_round_div(15*innovation,64))
    self.smooth_velocity_step=sat(self.smooth_velocity_step+smooth_round_div(innovation,64))
    self.smooth_velocity_normalized=sat(100*self.smooth_velocity_step)
    self.smooth_innovation=sat(innovation)
    target=max(-self.smooth_bound,min(self.smooth_bound,smooth_round_div(self.smooth_velocity_normalized-old_feedback,2)))
    self.smooth_correction=(max(self.smooth_correction-8192,min(self.smooth_correction+8192,target)) if sine_active else 0) if self.manual_smooth_continuity else target
   self.smooth_valid=True;self.smooth_sine_active=sine_active
   feedback_v=sat(old_feedback+self.smooth_correction)
  else:
   self.smooth_position=0;self.smooth_velocity_step=0;self.smooth_valid=False
   self.smooth_velocity_normalized=0;self.smooth_previous_feedback=0;self.smooth_correction=0;self.smooth_bound=0;self.smooth_innovation=0;self.smooth_sine_active=False
  terms=[(a*b)>>24 for a,b in zip([operand if i==2 else feedback_v if i==3 else n[i] for i in range(7)],gain)]
  feedback_source=sat(sum(terms));rod_source=sat(sum(terms[:2]))
  request=sat(sum(terms)+((lut(n[3])*C['friction'])>>24)+self.integral+(((n[7]*C['disturbance_gain_q24'])>>24)if self.compensate and not automatic else 0)+((((reference_velocity<<8)-((max(-4194304,min(4194304,reference_velocity<<8))>>3)if velocity_ff_trim and motion and not automatic and not low else 0))*(-2554777 if sine_active else LOWGAIN['velocity_ff_q24']if low else -Q//2))>>24))
  self.raw_request=request
  if self.manual_output_shape and low:
   profile=self.shape_profile
   source=rod_source if profile.get('source')=='rod' else feedback_source if profile.get('source')=='feedback' else request
   if self.shape_valid:
    self.shape_fast+=((source-self.shape_fast)>>profile['fast_shift'])
    self.shape_slow+=((source-self.shape_slow)>>profile['slow_shift'])
   else:self.shape_fast=source;self.shape_slow=source;self.shape_valid=True
   # Exact A36 F/L states; only the active-sine correction scale changes.
   correction_shift=0 if self.manual_sine_damping and sine_active else profile['gain_shift']
   self.shape_correction=max(-profile['bound_q24'],min(profile['bound_q24'],(self.shape_fast-self.shape_slow)>>correction_shift))
   request=sat(request-self.shape_correction)
  else:self.shape_fast=0;self.shape_slow=0;self.shape_valid=False;self.shape_correction=0
  # Reference-only correction is after F/L; original feedback source is unchanged.
  if sine_active:
   r=sine_reference<<8
   self.sine_bias=r if not self.sine_valid else 0 if abs(self.sine_bias)<=31 else self.sine_bias-(self.sine_bias>>5)
   self.sine_position_ff=((r-self.sine_bias)>>3)*2
   self.sine_reference=r
   request=sat(request+self.sine_position_ff)
  else:self.sine_bias=0;self.sine_position_ff=0;self.sine_reference=0
  self.sine_valid=sine_active
  self.shaped_request=request
  req=sat(((request*(C['inverse_ratio']if request<0 else Q))>>24)+(-C['dead']if request<0 else C['dead']if request>0 else 0))
  val=req*1000;mag=abs(val);rnd=(mag>>24)+int((mag%Q>Q//2)or(mag%Q==Q//2 and (mag>>24)&1));wanted=-rnd if val<0 else rnd
  # R41T2: a reduced current hard cap overrides the previous-output slew interval.
  bounded=max(-cap,min(cap,wanted));u=max(-cap,min(cap,max(previous-output_slew,min(previous+output_slew,bounded))));err=req-u*16777
  if (sine_active or not (hold_integral and motion and not automatic)) and self.age>=1000 and abs(t<<8)<2348810 and (sine_active or (abs(n[1])<838861 and abs(n[3])<838861)) and (abs(err)<50332 or istep*err<0):self.integral=max(-5872026,min(5872026,self.integral+istep))
  self.age=min(1000,self.age+1);self.z=n;self.ready=int(self.samples>=200);self.samples=min(200,self.samples+1)
  if automatic:
   self.blend=min(Q,self.blend+83886) if self.age>=200 and hold else max(0,self.blend-1048576)
   v=(u-fallback)*self.blend;mag=abs(v);rounded=(mag>>24)+int((mag%Q>Q//2)or(mag%Q==Q//2 and (mag>>24)&1));u=fallback+(-rounded if v<0 else rounded)
  return u,self.ready,0

# Complete A39 recursive state. Request/diagnostic values are retained where the
# A model API specified them, never inferred from missing CP words.
NATIVE_RESTORE_FIELDS=('z','integral','lp','age','samples','fault','ready',
 'bias_clipped','feedback_lp','shape_fast','shape_slow','shape_valid',
 'sine_bias','sine_valid','sine_position_ff','sine_reference',
 'smooth_position','smooth_velocity_step','smooth_valid','smooth_correction')
def _snapshot_native_state(self):
 import copy
 return {n:copy.deepcopy(getattr(self,n))for n in NATIVE_RESTORE_FIELDS}
def _restore_native_state(self,state):
 import copy
 if not isinstance(state,dict)or set(state)!=set(NATIVE_RESTORE_FIELDS):raise ValueError('R39 ext2 restore requires exactly20 fields including smooth_correction')
 boolean={'bias_clipped','shape_valid','sine_valid','smooth_valid'}
 for n,v in state.items():
  if n=='z':
   if type(v)is not list or len(v)!=8 or any(type(x)is not int or not -(1<<31)<=x<(1<<31)for x in v):raise ValueError('Invalid R39 ext2 state vector')
  elif n in boolean:
   if type(v)is not bool:raise ValueError('Invalid R39 ext2 flag '+n)
  elif type(v)is not int or not -(1<<31)<=v<(1<<31):raise ValueError('Invalid R39 ext2 word '+n)
 if not 0<=state['age']<=1000 or not 0<=state['samples']<=200 or state['fault']not in(0,1)or state['ready']not in(0,1):raise ValueError('Invalid R39 ext2 readiness')
 if abs(state['smooth_correction'])>167772 or(not state['smooth_valid']and state['smooth_correction']):raise ValueError('Invalid R39 recursive delta')
 for n,v in state.items():setattr(self,n,copy.deepcopy(v))
Observer.snapshot_native_state=_snapshot_native_state
Observer.restore_native_state=_restore_native_state
