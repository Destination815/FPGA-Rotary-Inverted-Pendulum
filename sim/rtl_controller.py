"""FFI to the complete generated RTL supervisor/reference/swing/tracking cores."""
from pathlib import Path
import ctypes as ct
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
class RTL:
 def __init__(self):
  self.lib=ct.CDLL(str(ROOT/('build/r01/libr01_control.dylib' if sys.platform=='darwin' else 'build/r01/libr01_control.so')))
  self.lib.r01_new.restype=ct.c_void_p
  for name in ('r01_delete','r01_stop'):getattr(self.lib,name).argtypes=[ct.c_void_p]
  self.lib.r01_input.argtypes=[ct.c_void_p]+[ct.c_int32]*6
  self.lib.r01_raw_input.argtypes=[ct.c_void_p]+[ct.c_int32]*4
  self.lib.r01_fraction_input.argtypes=[ct.c_void_p]+[ct.c_int32]*6
  self.lib.r01_calibrate.argtypes=[ct.c_void_p]
  self.lib.r01_begin.argtypes=[ct.c_void_p,ct.c_int]
  self.lib.r01_command.argtypes=[ct.c_void_p,ct.c_int,ct.c_int32]
  self.lib.r01_tick.argtypes=[ct.c_void_p,ct.POINTER(ct.c_int32)]
  self.lib.r20_commit_info.argtypes=[ct.c_void_p,ct.POINTER(ct.c_int32)];self.lib.r20_commit_info.restype=ct.c_int
  self.lib.r20_idle.argtypes=[ct.c_void_p,ct.c_int];self.lib.r20_idle.restype=ct.c_int
  if hasattr(self.lib,'r29_guard_status'):self.lib.r29_guard_status.argtypes=[ct.c_void_p];self.lib.r29_guard_status.restype=ct.c_int
  self.ptr=self.lib.r01_new();self.out=(ct.c_int32*17)()
 def configure(self,workspace=60,amplitude=15,speed=0,period=20,full_turn_recovery=False):
  import math
  if hasattr(self.lib,"r26_recovery_config"):
   self.lib.r26_recovery_config.argtypes=[ct.c_void_p,ct.c_int];self.lib.r26_recovery_config(self.ptr,int(full_turn_recovery))
  elif full_turn_recovery:raise RuntimeError("RTL library does not implement recovery boundary opt-in")
  self.lib.r14_config.argtypes=[ct.c_void_p]+[ct.c_int32]*4
  self.lib.r14_config(self.ptr,round(math.radians(workspace)*65536),round(math.radians(amplitude)*65536),speed,period)
 def measure(self,theta,omega,phi,vel,filtered,valid=True):
  self.lib.r01_input(self.ptr,*[round(x*65536) for x in (theta,omega,phi,vel)],round(filtered*262144),int(valid))
 def numeric_speed(self,physical_deg_s=None,legacy_rad_s=None):
  import math
  if (physical_deg_s is None)==(legacy_rad_s is None):raise ValueError('Supply exactly one numeric speed coordinate')
  if physical_deg_s is not None:
   sys.path.insert(0,str(ROOT/'host'))
   from motion_config import speed_internal_q16
   value=speed_internal_q16(float(physical_deg_s))
  else:
   value=round(float(legacy_rad_s)*65536)
   if not 1186<=value<=71165:raise ValueError('Legacy speed outside admitted physical1..60deg/s')
  self.lib.r36_numeric_speed.argtypes=[ct.c_void_p,ct.c_int32]
  self.lib.r36_numeric_speed(self.ptr,value)
 def sine_parameters(self,amplitude_deg,period_s):
  sys.path.insert(0,str(ROOT/'host'))
  from sine_config import settings
  s=settings(amplitude_deg,period_s)
  self.lib.r41t2_sine.argtypes=[ct.c_void_p,ct.c_int32,ct.c_uint32]
  self.lib.r41t2_sine(self.ptr,s['amplitude_internal_q16'],s['phase_step'])
 def raw(self,adc,encoder,zero=746,valid=True):self.lib.r01_raw_input(self.ptr,adc,encoder,zero,int(valid))
 def fraction(self,adc,adc_q4,encoder,zero_q4=11936,valid=True,fractional_valid=True):self.lib.r01_fraction_input(self.ptr,adc,adc_q4,encoder,zero_q4,int(valid),int(fractional_valid))
 def calibrate(self):self.lib.r01_calibrate(self.ptr)
 def begin(self,manual=False):self.lib.r01_begin(self.ptr,int(manual))
 def command(self,op,value):self.lib.r01_command(self.ptr,op,round(value*65536) if op==6 else int(value))
 def tick(self):
  if not self.lib.r01_tick(self.ptr,self.out):raise RuntimeError('RTL missed harness 49000-clock deadline')
  x=list(self.out)
  result=dict(u=x[0]/1000,state=x[1],fault=x[2],ref=x[3]/65536,ref_v=x[4]/65536,ready=bool(x[5]),trajectory=bool(x[6]),cycles=x[7],limit=x[8]/1000,theta=x[9]/65536,omega=x[10]/65536,phi=x[11]/65536,vel=x[12]/65536,filtered=x[13]/262144,gap=bool(x[14]),gap_ms=x[15],measurement_ok=bool(x[16]))
  changes=(ct.c_int32*16)();n=self.lib.r20_commit_info(self.ptr,changes)
  if n>8:raise RuntimeError('command changes exceed diagnostic capacity')
  result['commits']=[(changes[i*2],changes[i*2+1]/1000)for i in range(n)]
  result['handoff_recovery']=bool(self.lib.r29_guard_status(self.ptr)) if hasattr(self.lib,'r29_guard_status') else False
  return result
 def idle(self,n=64):
  if self.lib.r20_idle(self.ptr,n):raise RuntimeError('command changed while idle')

 def stop(self):self.lib.r01_stop(self.ptr)
 def ext2(self,enabled=True):
  self.lib.r40_ext2.argtypes=[ct.c_void_p,ct.c_int]
  self.lib.r40_ext2(self.ptr,int(enabled))
 def ext2_active(self):
  self.lib.r40_ext2_status.argtypes=[ct.c_void_p];self.lib.r40_ext2_status.restype=ct.c_int
  return bool(self.lib.r40_ext2_status(self.ptr))
 def close(self):
  if self.ptr:self.lib.r01_delete(self.ptr);self.ptr=None
