"""Digital golden ADC frontend: same 2.5MHz sequence feeds both implementations.
Block sums are exact integers. No hardware claim; noise/time model explicit.
Warmup follows source: discard first16 conversions. Analogue position interpolated over the just-completed1ms plant step.
"""
import numpy as np
class SampledADC:
 def __init__(self,noise='white',seed=4217):
  self.noise=noise;self.rng=np.random.default_rng(seed);self.ticks=0;self.warmup=16;self.last_code=None
  self.c16=0;self.s16=0;self.average=0
  self.c512=0;self.s512=0;self.lo=1023;self.hi=0;self.q4=0;self.valid=False;self.ready=False
 def push(self,seq):
  seq=np.asarray(seq,dtype=np.int64)
  if self.warmup:
   n=min(self.warmup,len(seq));seq=seq[n:];self.warmup-=n
  # The consumer reads at1kHz, so retain only last completed16 block.
  # Skipping intermediate output assignments is exact at each push boundary.
  j=0
  if self.c16:
   n=min(16-self.c16,len(seq));self.s16+=int(seq[:n].sum());self.c16+=n;j=n
   if self.c16==16:self.average=self.s16>>4;self.c16=0;self.s16=0
  full=(len(seq)-j)//16
  if full:
   j+=16*full;self.average=int(seq[j-16:j].sum())>>4
  if j<len(seq):self.c16+=len(seq)-j;self.s16+=int(seq[j:].sum())
  j=0
  while j<len(seq):
   n=min(512-self.c512,len(seq)-j);chunk=seq[j:j+n];j+=n
   self.s512+=int(chunk.sum());self.c512+=n;self.lo=min(self.lo,int(chunk.min()));self.hi=max(self.hi,int(chunk.max()))
   if self.c512==512:
    self.q4=self.s512>>5;self.valid=self.lo>3 and self.hi<1020 and self.hi-self.lo<=32;self.ready=True
    self.s512=0;self.c512=0;self.lo=1023;self.hi=0
  return self.average,self.q4,self.valid and self.ready
 def sample(self,ideal_code,gap=False):
  t=(self.ticks+np.arange(2500)/2500)*.001;self.ticks+=1
  if self.noise=='white':noise=self.rng.normal(0,.6,2500)
  elif self.noise=='held':noise=np.full(2500,self.rng.uniform(-1,1))
  elif self.noise=='mains':noise=.8*np.sin(2*np.pi*50*t)+self.rng.normal(0,.15,2500)
  elif self.noise=='none':noise=np.zeros(2500)
  else:raise ValueError(self.noise)
  ideal=np.full(2500,float(ideal_code)) if self.last_code is None or gap or abs(ideal_code-self.last_code)>32 else np.linspace(self.last_code,ideal_code,2500,endpoint=True)
  self.last_code=float(ideal_code)
  seq=np.clip(np.rint(ideal+(0 if gap else noise)),0,1023).astype(np.int64)
  return self.push(seq)
