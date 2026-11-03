"""Measured-state gap observer; its gap model knows no true simulated state."""
import math
SCALE=386/65536;ENC=396/65536;Q=1<<24
wrap=lambda x:(x+math.pi)%(2*math.pi)-math.pi
class Observer:
 def __init__(self,zero=746,max_gap_ms=180):
  self.zero=zero;self.max_gap_ms=max_gap_ms;self.pos=0.;self.w=0.;self.v=0.;self.prev_v=0.;self.last_raw=None;self.gap_age=0;self.initialized=False;self.hist=[0.]*16;self.index=0;self.fill=0;self.fault='';self.bridge_samples=0;self.max_gap_age=0
 def update(self,adc,enc,gap_allow=True):
  phi=enc*ENC
  old=self.hist[self.index] if self.fill>=16 else (self.hist[(self.index-1)%16] if self.fill else phi)
  rate=(phi-old)/(.016 if self.fill>=16 else .001);rate=max(-30,min(30,rate))
  self.hist[self.index]=phi;self.index=(self.index+1)%16;self.fill=min(self.fill+1,16)
  vel=self.v+.25*(rate-self.v) if self.initialized else 0.
  theta=wrap((adc-self.zero)*SCALE)
  rawgood=3<adc<1020
  pred=wrap(self.pos+self.w*.001)
  phase=(pred+self.zero*SCALE)%(2*math.pi)
  # Pad4codes beyond physical endpoints to cover predictor drift/ADC quantization.
  in_gap=(phase>1016*SCALE or phase<7*SCALE)
  innovation=wrap(theta-pred)
  onset=gap_allow and self.initialized and in_gap
  if onset or self.gap_age:
   if not gap_allow:self.fault='gap_disabled'
   if not self.gap_age:
    acc=94.50762226949593*math.sin(self.pos)+math.sin(self.pos)*math.cos(self.pos)*vel*vel-.15*self.w-1.27166*math.cos(self.pos)*(vel-self.v)/.001
    self.w=max(-30,min(30,self.w+.0145*acc))
   self.gap_age+=1;self.max_gap_age=max(self.max_gap_age,self.gap_age)
   sin=math.sin(self.pos);cos=math.cos(self.pos)
   # Integrate the measured arm-velocity increment, rather than divide then
   # multiply by dt. Baseline ABQ24 precision is retained after every sample.
   dw=(94.50762226949593*sin+sin*cos*vel*vel-.15*self.w)*.001-1.27166*cos*(vel-self.v)
   wn=max(-30,min(30,self.w+dw));pn=wrap(self.pos+.001*(self.w+.5*dw))
   phase_n=(pn+self.zero*SCALE)%(2*math.pi)
   # Reacquire only off endpoint rails AND outside the known gap, with innovation
   # consistency. A held endpoint value cannot masquerade as an exit.
   outside=8*SCALE<phase_n<1015*SCALE
   goodexit=rawgood and outside and abs(wrap(theta-pn))<.12 and 7<adc<1016
   if goodexit:
    err=wrap(theta-pn);self.pos=wrap(pn+15/64*err);self.w=max(-30,min(30,wn+err/.064));self.gap_age=0
   else:self.pos=pn;self.w=wn;self.bridge_samples+=1
   if self.gap_age>self.max_gap_ms:self.fault='gap_timeout'
   outtheta=self.pos
  elif not self.initialized:
   self.pos=theta;self.w=0.;self.initialized=rawgood;outtheta=theta
  elif not rawgood or abs(innovation)>.15:
   self.fault='unexpected_adc';outtheta=theta
  else:
   self.pos=wrap(pred+15/64*innovation);self.w=max(-30,min(30,self.w+innovation/.064));outtheta=theta
  self.pos=round(self.pos*Q)/Q;self.w=round(self.w*.001*Q)/Q*1000
  self.prev_v=self.v;self.v=vel;self.last_raw=adc
  return outtheta,self.w,phi,self.v,self.pos,not self.fault

def sensor(theta,zero=746,behavior='zero',last=None):
 phase=(theta+zero*SCALE)%(2*math.pi)
 if phase<=1023*SCALE:return min(1023,max(0,round(phase/SCALE)))
 if behavior=='zero':return 0
 if behavior=='max':return 1023
 if behavior=='hold':return last if last is not None else 0
 if behavior=='sweep':return round((phase-1023*SCALE)/(2*math.pi-1023*SCALE)*1023)
 raise ValueError(behavior)
