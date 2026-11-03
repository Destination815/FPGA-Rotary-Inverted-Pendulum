"""Independent integer transaction oracle; arbitrary precision Python sums.
32-bit state saturation, Q24 product truncation, LUT and ties-to-even output.
"""
from pathlib import Path
import json,math
import importlib.util as _importlib_util
_ext2_spec=_importlib_util.spec_from_file_location('j280_r39_ext2_integer',Path(__file__).with_name('r39_ext2_model.py'));_ext2_module=_importlib_util.module_from_spec(_ext2_spec);_ext2_spec.loader.exec_module(_ext2_module);Ext2Observer=_ext2_module.Observer
C=json.loads((Path(__file__).with_name('constants.json')).read_text());Q=C['Q'];LUT=[round(math.tanh(i/1024/.1)*Q)for i in range(1025)]
AUTO_GAIN=[67937104,38753879,3568142,30194271,-268429,-2104864,-2148228,-3019427]
MANUAL_GAIN=[126466217,117243313,11515862,91736295,-747125,-5912648,-7009661,-8388608]
MOTION_GAINS=[[round(a+(b-a)*phase/7)for a,b in zip(AUTO_GAIN,MANUAL_GAIN)]for phase in range(8)]
def i32(x):return ((int(x)+(1<<31))%(1<<32))-(1<<31)
def sat(x):return max(-2**31,min(2**31-1,x))
def lut(x):
 i=(abs(x)*10240+Q//2)>>24
 return (1 if x>=0 else -1)*(LUT[i]if i<1025 else Q)
class DampingGate:
 def __init__(self):self.clear()
 def clear(self):self.quiet=0;self.sign=0;self.elapsed=0;self.halves=0;self.active=False;self.weight=0;self.ramp=0;self.mode=False
 def tick(self,eligible,theta,postmove=False):
  if not eligible or self.mode!=postmove:self.clear();self.mode=postmove;return self.weight
  lower,upper=(60,500)if postmove else(80,180)
  if self.quiet<1000:self.quiet+=1;return self.weight
  was=self.active
  if not self.active:
   if self.sign:self.elapsed=min(upper+1,self.elapsed+1)
   hit=1 if theta>=1144 else -1 if theta<=-1144 else 0
   if hit and hit!=self.sign:
    if self.sign and lower<=self.elapsed<=upper:self.halves=min(4,self.halves+1)
    else:self.halves=0
    self.sign=hit;self.elapsed=0
    if self.halves>=4:self.active=True
   elif self.elapsed>upper:self.sign=0;self.elapsed=0;self.halves=0
  if was and self.weight<3:
   self.ramp+=1
   if self.ramp>=20:self.ramp=0;self.weight+=1
  return self.weight

class Observer:
 def __init__(self):self.mature_static_hold=False;self.clear()
 def clear(self):self.r40_unwind_active=False;self.ext2=Ext2Observer(disturbance_observer=True,disturbance_regulator=True,manual_low_gain=True,manual_encoder_feedback=True,manual_filtered_motion=True,manual_output_shape=True,manual_damped_feedback=True,manual_sine_feedforward=True,manual_sine_damping=True,manual_smooth_velocity=True,manual_smooth_continuity=True);self.ext2_mode=False;self.transfer_ticks=0;self.rate_replacement_delta=0;self.rate_good=False;self.fast_good=0;self.fast_correction=0;self.stasis_error=0;self.stasis_samples=0;self.stasis_reference=0;self.damping_gate=DampingGate();self.manual_moved=False;self.motion_watch=False;self.motion_bad_ticks=0;self.motion_latched=False;self.motion_phase=0;self.motion_ticks=0;self.drift_latched=False;self.blend=0;self.integral=0;self.lp=0;self.age=0;self.z=[0]*7;self.samples=0;self.fault=0;self.ready=0
 def tick(self,t,p,previous,cap,reference_offset=0,reference_velocity=0,automatic=False,hold=True,fallback=0,brake=False,prediction_previous=None,motion_assist_permission=False,measured_omega=0,measurement_current=False,rate_hold=False,ext2_warm=False,ext2_enable=False,ext2_motion=False,ext2_reference=0):
  if ext2_warm or not (automatic and not brake and motion_assist_permission and measurement_current) or self.fault:self.r40_unwind_active=False
  if bool(ext2_warm)!=self.ext2_mode:
   self.transfer_ticks=128
   if ext2_warm:
    self.ext2.z[7]=0;self.ext2.shape_valid=False;self.ext2.smooth_valid=False;self.ext2.sine_valid=False;self.ext2.feedback_lp=0
  elif self.transfer_ticks:self.transfer_ticks-=1
  if ext2_warm:
   # The RTL shares old z/I/LP/age/ready; copy those exact states at the gate.
   self.ext2.z=self.z.copy()+[self.ext2.z[7]];self.ext2.integral=self.integral;self.ext2.lp=self.lp;self.ext2.age=self.age;self.ext2.samples=self.samples;self.ext2.ready=self.ready;self.ext2.fault=self.fault
   r=self.ext2.tick(t,p,previous,cap,reference_offset,reference_velocity,False,hold,fallback,brake,motion=ext2_motion,hold_integral=True,velocity_ff_trim=True,motion_position_boost=True,sine_tracking=ext2_enable,sine_reference=ext2_reference,prediction_previous=prediction_previous if automatic else None,output_slew=20)
   self.z=self.ext2.z[:7];self.integral=self.ext2.integral;self.lp=self.ext2.lp;self.age=self.ext2.age;self.samples=self.ext2.samples;self.ready=self.ext2.ready;self.fault=self.ext2.fault;self.ext2_mode=True
   return r
  self.ext2_mode=False
  if not automatic and abs(reference_velocity)>64:self.manual_moved=True
  # Mirror the RTL's existing previous reference_s latch, not new hardware.
  if self.stasis_samples and self.stasis_reference!=reference_offset:self.stasis_samples=0
  self.stasis_reference=reference_offset
  if self.fault:return 0,self.ready,1
  if abs(t)>20000 or abs(p)>823550 or abs(previous)>350 or not 150<=cap<=350 or abs(reference_offset)>823550:self.fault=1;self.r40_unwind_active=False;return 0,self.ready,1
  if automatic and not brake and motion_assist_permission and abs(reference_velocity)>64:self.motion_watch=True
  postmove_mode=self.mature_static_hold and automatic and not brake and reference_velocity==0 and motion_assist_permission and self.motion_latched and self.motion_phase==3
  damping_eligible=(measurement_current and abs(measured_omega)<163840 and abs(t)<=9151)if postmove_mode else(self.mature_static_hold and automatic and not brake and reference_velocity==0 and not self.motion_latched and abs(t)<=9151 and abs(p-reference_offset)<=5719)
  self.damping_gate.tick(damping_eligible,t,postmove_mode)
  if automatic and not brake and motion_assist_permission and abs(t)>1144:
   self.motion_bad_ticks=min(20,self.motion_bad_ticks+1)
   if self.motion_bad_ticks>=20:self.motion_latched=True
  else:self.motion_bad_ticks=0
  prior=(prediction_previous if automatic and prediction_previous is not None else previous)*16777;e=max(0,prior-C['dead'])if prior>=0 else min(0,prior+C['dead']);e=sat((e*(C['ratio']if e<0 else Q))>>24)
  v=self.z+[e,lut(self.z[3]),t<<8,p<<8];n=[sat(sum((a*b)>>24 for a,b in zip(v,row)))for row in C['matrix']]
  self.drift_latched=self.drift_latched or (automatic and abs(p-reference_offset)>5719)
  drift=automatic and self.drift_latched
  active=not automatic and self.manual_moved and reference_velocity==0 and abs(t<<8)<2348810
  quiet=abs(n[1])<838861 and abs(n[3])<838861
  stasis_eligible=self.mature_static_hold and automatic and not brake and reference_velocity==0 and not self.motion_latched and abs(t)<1144 and abs(p-reference_offset)<=5719 and quiet
  endpoint_stasis_eligible=self.mature_static_hold and automatic and not brake and motion_assist_permission and reference_velocity==0 and measurement_current and abs(p-reference_offset)>1144 and abs(t)<3432 and quiet
  stasis_error=p-reference_offset
  if stasis_eligible or endpoint_stasis_eligible:self.stasis_samples=min(201,self.stasis_samples+1) if self.stasis_samples and stasis_error==self.stasis_error else 1
  else:self.stasis_samples=0
  self.stasis_error=stasis_error
  static_i_hold=stasis_eligible and not endpoint_stasis_eligible and self.stasis_samples>=201
  oldlp=self.lp;self.lp=sat(self.lp+(((p<<8)-(reference_offset<<8)-self.lp)*(133682 if drift else 16760)>>24))
  if not active and not drift and (abs(n[1])>838861 or abs(n[3])>838861):self.lp=oldlp if (automatic or abs(reference_offset)>2288) and abs(reference_velocity)<6554 else 0
  explicit_endpoint=endpoint_stasis_eligible  # R41: encoder dithering must not indefinitely defer explicit endpoint correction
  # R41: accumulate bounded correction before arrival only when the arm
  # leads the moving reference; do not integrate acceleration-phase lag.
  r41_moving_lead_i=automatic and not brake and motion_assist_permission and measurement_current and ((reference_velocity>64 and p>reference_offset and self.lp>0) or (reference_velocity< -64 and p<reference_offset and self.lp<0))
  istep=self.lp*(1048 if r41_moving_lead_i or (drift and not explicit_endpoint) else 4194)>>24
  r40_scope=automatic and not brake and motion_assist_permission and measurement_current
  r40_opposed=self.lp*self.integral<0
  r40_trigger=r40_scope and r40_opposed and abs(self.lp)>1218492 and abs(self.integral)>=838861
  self.r40_unwind_active=r40_scope and self.integral!=0 and (self.r40_unwind_active or r40_trigger)
  r40_unwind=self.r40_unwind_active and r40_opposed
  if r40_unwind:istep=max(-16777,min(16777,(self.lp*8389)>>24))
  gain=(([67937104,38753879,3568142,30194271,-268429,-2104864,-2148228] if brake else MOTION_GAINS[self.motion_phase][:7]) if automatic else C['gain'].copy())
  if not automatic and (abs(n[1])>2516582 or abs(n[0])>1464088):gain[0]=C['theta_fast_gain']
  if self.mature_static_hold and automatic and not brake and reference_velocity==0 and cap==150 and not self.motion_latched:gain=gain.copy();gain[1]=gain[1]*(20+self.damping_gate.weight)//20
  rate_eligible=(self.mature_static_hold or rate_hold) and reference_velocity==0 and automatic and not brake and measurement_current and abs(measured_omega)<=1966080
  previous_rate_good=self.rate_good;self.rate_good=rate_eligible
  fast_eligible=self.mature_static_hold and automatic and not brake and reference_velocity==0 and not self.motion_latched and not motion_assist_permission and measurement_current and abs(measured_omega)<=1966080
  self.fast_good=min(201,self.fast_good+1) if fast_eligible else 0
  innovation=(measured_omega<<8)-10*n[1]
  shaped=innovation-13421773 if innovation>13421773 else innovation+13421773 if innovation< -13421773 else 0
  boost=abs(innovation)>13421773 and abs(measured_omega)<163840 and abs(t)<9830
  strong_shaped=shaped # R43: continuous at the existing 0.8 rad/s authorization boundary.
  bound=2097152 if boost else 1048576
  self.fast_correction=max(-bound,min(bound,((strong_shaped>>2 if motion_assist_permission else shaped*4) if boost else shaped>>4))) if rate_eligible and previous_rate_good else 0
  feedback_v=max(-50332,min(50332,n[3])) if static_i_hold and self.fast_good>=201 else n[3]
  feedback_states=[n[i]-(reference_offset<<8) if i==2 else feedback_v if i==3 else n[i] for i in range(7)]
  self.rate_replacement_delta=max(-243012,min(243012,(innovation>>6)*self.damping_gate.weight))if postmove_mode and measurement_current and self.motion_latched and self.motion_phase==3 else 0
  feedback_states[1]=sat(n[1]+self.rate_replacement_delta)
  request=sat(self.fast_correction+sum((a*b)>>24 for a,b in zip(feedback_states,gain))+((lut(feedback_v)*C['friction'])>>24)+self.integral+(((reference_velocity<<8)*(MOTION_GAINS[self.motion_phase][7] if automatic and self.motion_latched and not brake else -Q//2))>>24))
  req=sat(((request*(C['inverse_ratio']if request<0 else Q))>>24)+(-C['dead']if request<0 else C['dead']if request>0 else 0))
  val=req*1000;mag=abs(val);rnd=(mag>>24)+int((mag%Q>Q//2)or(mag%Q==Q//2 and (mag>>24)&1));wanted=-rnd if val<0 else rnd
  slew_quiet=abs(n[1])<838861 if reference_velocity==0 else measurement_current and abs(measured_omega)<32768
  bounded=max(-cap,min(cap,wanted));slew=5 if automatic and self.motion_latched and not brake and abs(t)<2621 and slew_quiet else 20;u=max(-cap,min(cap,max(previous-slew,min(previous+slew,bounded))));err=req-u*16777
  if self.age>=1000 and abs(t<<8)<2348810 and (quiet or r41_moving_lead_i or r40_unwind or (active and istep*self.integral<0)) and (abs(err)<50332 or istep*err<0):
   next_i=max(-5872026,min(5872026,self.integral+istep))
   if r40_unwind or (active and not quiet):next_i=max(0,next_i) if self.integral>=0 else min(0,next_i)
   # On an already balanced, stationary automatic HOLD, position error
   # under static friction must not keep winding the position integrator.
   # Permit unwind toward zero; the proportional position path remains.
   if static_i_hold:
    next_i=max(0,min(self.integral,next_i)) if self.integral>=0 else min(0,max(self.integral,next_i))
   self.integral=next_i
  if self.integral==0:self.r40_unwind_active=False
  if automatic and self.motion_latched and not brake and self.age>=199 and self.motion_phase<3:
   if self.motion_ticks>=49:self.motion_ticks=0;self.motion_phase+=1
   else:self.motion_ticks+=1
  self.age=min(1000,self.age+1);self.z=n;self.ready=int(self.samples>=200);self.samples=min(200,self.samples+1)
  if automatic:
   self.blend=min(Q,self.blend+83886) if self.age>=200 and (hold or self.motion_latched) else max(0,self.blend-1048576)
   v=(u-fallback)*self.blend;mag=abs(v);rounded=(mag>>24)+int((mag%Q>Q//2)or(mag%Q==Q//2 and (mag>>24)&1));u=fallback+(-rounded if v<0 else rounded)
  return u,self.ready,0

# R39 exposes only a complete extension-state restore; never zero-fills CP data.
def _r39_snapshot_ext2(self):
 return self.ext2.snapshot_native_state()
def _r39_restore_ext2(self,state):
 self.ext2.restore_native_state(state)
 self.z=self.ext2.z[:7];self.integral=self.ext2.integral;self.lp=self.ext2.lp
 self.age=self.ext2.age;self.samples=self.ext2.samples;self.ready=self.ext2.ready;self.fault=self.ext2.fault
 self.ext2_mode=True
Observer.snapshot_ext2_state=_r39_snapshot_ext2
Observer.restore_ext2_state=_r39_restore_ext2

# R40 model-only full continuation. Native A snapshots stay the historical
# 20-field format; this wrapper also carries the preserved ordinary-mode state.
def _r40_snapshot_seamless(self):
 import copy
 ordinary={k:copy.deepcopy(v.__dict__ if k=='damping_gate' else v) for k,v in self.__dict__.items() if k!='ext2'}
 return dict(format='R40-seamless-v2',ordinary=ordinary,ext2=self.ext2.snapshot_native_state())
def _r40_restore_seamless(self,state):
 import copy
 if not isinstance(state,dict) or set(state)!={'format','ordinary','ext2'} or state['format']!='R40-seamless-v2':raise ValueError('Expected complete R40 seamless snapshot')
 fresh=Observer(); ordinary=state['ordinary']; expected=fresh.snapshot_seamless_state()['ordinary']
 if not isinstance(ordinary,dict) or set(ordinary)!=set(expected):raise ValueError('Incomplete R40 ordinary state')
 for key,value in ordinary.items():
  prototype=expected[key]
  if type(value)is not type(prototype):raise ValueError('Invalid R40 state type '+key)
  if key=='z' and (len(value)!=7 or any(type(x)is not int or not -(1<<31)<=x<(1<<31) for x in value)):raise ValueError('Invalid R40 shared state')
  if key=='damping_gate' and (set(value)!=set(prototype) or any(type(value[k])is not type(prototype[k])for k in prototype)):raise ValueError('Invalid R40 damping state')
 if not 0<=ordinary['transfer_ticks']<=128:raise ValueError('Invalid R40 transfer counter')
 if ordinary['r40_unwind_active'] and (ordinary['ext2_mode'] or ordinary['fault'] or ordinary['integral']==0):raise ValueError('Invalid retained R40 unwind authorization')
 fresh.ext2.restore_native_state(state['ext2'])
 for key,value in ordinary.items():
  if key=='damping_gate':fresh.damping_gate.__dict__.update(copy.deepcopy(value))
  else:setattr(fresh,key,copy.deepcopy(value))
 self.__dict__.clear();self.__dict__.update(fresh.__dict__)
Observer.snapshot_seamless_state=_r40_snapshot_seamless
Observer.restore_seamless_state=_r40_restore_seamless
