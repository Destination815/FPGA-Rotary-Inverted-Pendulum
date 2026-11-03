"""Integer bit model and independent double reference used by Verilator validation."""
import math
Q=1<<18
qr=lambda x:round(x*Q)
mul=lambda a,b:(int(a)*int(b))>>18
BINS=(2000,4000,6000,8000,12000,16000,20000,24000,32000,48000,64000,96000)
NUMERIC_BINS=BINS+(192000,384000,768000,1536000)
LUT=[qr(math.sin(2*math.pi*i/256)) for i in range(256)]
def choose(delta,speed=0,speed_limit=0):
 if speed_limit>0:
  for ms in NUMERIC_BINS:
   t=ms//1000
   if abs(delta)*15<=speed_limit*8*t and abs(delta)<=min(2147483647,qr(math.radians(30)*t*t/5.774)):return ms
  return NUMERIC_BINS[-1]
 for ms in BINS:
  t=ms/1000
  if abs(delta)<=qr(min(math.radians([10,20,30,60][speed])*t/1.875,math.radians(30)*t*t/5.774)):return max(4000,ms) if speed==0 else ms
 return 96000
class Reference:
 def __init__(self):
  self.centered=False;self.center=0;self.stop_pending=False;self.ending=False;self.mode=0;self.age=0;self.duration=2000;self.phase=0;self.origin=self.goal=0;self.c=[0]*5;self.out=[0]*4;self.reject=False;self.floatout=[0.]*4
 def tick(self,reseed=0,seed=0,target_valid=0,target=0,shuttle_valid=0,shuttle_on=0,amplitude=qr(math.radians(15)),speed_level=0,period_seconds=20,target_bound=qr(math.pi),speed_limit=0,ext2_centered=0,ext2_phase_step=0,anchor_from_seed=0):
  self.reject=False
  if reseed:
   self.centered=False;self.center=0;self.stop_pending=self.ending=False;self.mode=0;self.origin=self.goal=seed;self.out=[seed,0,0,0];self.floatout=[seed/Q,0,0,0];return self.out
  if target_valid:
   if self.mode!=0 or abs(target)>target_bound:self.reject=True;return self.out
   self.mode=1;self.origin=seed if anchor_from_seed else self.out[0];self.goal=target;d=target-self.origin;self.c=[0,0,10*d,-15*d,6*d];self.age=0;self.duration=choose(d,speed_level,speed_limit)
  elif shuttle_valid and shuttle_on:
   if self.mode!=0 or ((amplitude<=0 or (seed if anchor_from_seed else self.out[0])+amplitude>target_bound or (seed if anchor_from_seed else self.out[0])-amplitude < -target_bound) if ext2_centered else abs(self.out[0])>23334):self.reject=True;return self.out
   self.centered=bool(ext2_centered);self.center=(seed if anchor_from_seed else self.out[0]) if self.centered else 0;self.stop_pending=self.ending=False;self.amplitude=amplitude;self.period=3.5 if period_seconds==35 else period_seconds;self.mode=2;self.origin=self.out[0];self.age=0;self.phase=0;self.duration=2000
   self.configured_step=ext2_phase_step if self.centered else 0
   if self.configured_step:self.period=(1<<32)/(1000*self.configured_step)
  elif shuttle_valid and not shuttle_on and self.mode==2 and self.centered:
   self.stop_pending=True
  elif shuttle_valid and not shuttle_on and self.mode==2:
   p,v,a,j=self.out;self.mode=3;self.origin=p;self.goal=p+v;self.c=[2*v,2*a,-2*v-6*a,v+6*a,-2*a];self.age=0;self.duration=2000
  if self.mode==2 and self.centered and self.ending and self.age>=2000:
   self.mode=0;self.stop_pending=self.ending=False;self.out=[self.center,0,0,0];self.floatout=[self.center/Q,0,0,0];return self.out
  if self.mode==2 and self.centered and self.stop_pending and not self.ending and self.age>=2000:
   self.ending=True;self.age=0;self.origin=self.center
  if self.mode==0:return self.out
  if self.mode!=2 and self.age>=self.duration:
   self.mode=0;self.out=[self.goal,0,0,0];self.floatout=[self.goal/Q,0,0,0];return self.out
  q=min(Q,(self.age*round((1<<32)/self.duration))>>14)
  q2=mul(q,q);q3=mul(q2,q);q4=mul(q3,q);q5=mul(q4,q)
  if self.mode!=2:
   c1,c2,c3,c4,c5=self.c;T=self.duration/1000
   p=self.origin+sum(mul(a,b) for a,b in zip(self.c,[q,q2,q3,q4,q5]))
   v=mul(c1+2*mul(c2,q)+3*mul(c3,q2)+4*mul(c4,q3)+5*mul(c5,q4),qr(1/T))
   a=mul(2*c2+6*mul(c3,q)+12*mul(c4,q2)+20*mul(c5,q3),qr(1/T**2))
   j=mul(6*c3+24*mul(c4,q)+60*mul(c5,q2),qr(1/T**3))
   z=self.age/self.duration;c=[self.origin]+self.c
   self.floatout=[sum(c[k]/Q*math.prod(range(k-der+1,k+1))*z**(k-der) for k in range(der,6))/T**der for der in range(4)]
   self.age+=1
  else:
   ix=self.phase>>24;f=(self.phase>>6)&(Q-1);s=LUT[ix]+mul(LUT[(ix+1)%256]-LUT[ix],f);c=LUT[(ix+64)%256]+mul(LUT[(ix+65)%256]-LUT[(ix+64)%256],f)
   w=2*math.pi/self.period;A=self.amplitude/Q
   if self.configured_step:
    vq=(self.configured_step*6433982)>>24;aq=mul(vq,vq);jq=mul(aq,vq)
   else:vq,aq,jq=qr(w),qr(w*w),qr(w**3)
   y=self.center+mul(s,qr(A));yd=mul(c,mul(self.amplitude,vq));ydd=mul(s,-mul(self.amplitude,aq));yddd=mul(c,-mul(self.amplitude,jq))
   if self.age>=2000:h=Q;hd=hdd=hddd=0
   else:h=10*q3-15*q4+6*q5;hd=(30*q2-60*q3+30*q4)>>1;hdd=(60*q-180*q2+120*q3)>>2;hddd=(60*Q-360*q+360*q2)>>3
   if self.ending:h=Q-h;hd=-hd;hdd=-hdd;hddd=-hddd
   p=self.origin+mul(h,y-self.origin);v=mul(hd,y-self.origin)+mul(h,yd)
   a=mul(hdd,y-self.origin)+2*mul(hd,yd)+mul(h,ydd)
   j=mul(hddd,y-self.origin)+3*mul(hdd,yd)+3*mul(hd,ydd)+mul(h,yddd)
   z=min(1.,self.age/2000);r=self.phase/(1<<32)*2*math.pi;yf=self.center/Q+A*math.sin(r);ydf=A*w*math.cos(r);yddf=-A*w*w*math.sin(r);ydddf=-A*w**3*math.cos(r)
   hf=10*z**3-15*z**4+6*z**5;hdf=(30*z*z-60*z**3+30*z**4)/2 if self.age<2000 else 0
   hddf=(60*z-180*z*z+120*z**3)/4 if self.age<2000 else 0;hdddf=(60-360*z+360*z*z)/8 if self.age<2000 else 0
   if self.ending:hf=1-hf;hdf=-hdf;hddf=-hddf;hdddf=-hdddf
   d=yf-self.origin/Q;self.floatout=[self.origin/Q+hf*d,hdf*d+hf*ydf,hddf*d+2*hdf*ydf+hf*yddf,hdddf*d+3*hddf*ydf+3*hdf*yddf+hf*ydddf]
   self.phase=(self.phase+round((1<<32)/(self.period*1000)))&0xffffffff;self.age=min(2000,self.age+1)
  self.out=[p,v,a,j];return self.out
