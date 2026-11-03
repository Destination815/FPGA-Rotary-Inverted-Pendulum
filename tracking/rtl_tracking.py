"""Verilator RTL-in-the-loop adapter; actual clock-stepped HDL produces each duty."""
from pathlib import Path
import ctypes
Q=1<<18
HERE=Path(__file__).resolve().parent
class RtlController:
 def __init__(self,parameters=None):
  self.lib=ctypes.CDLL(str(HERE/'build/libtracking.dylib'))
  self.lib.tracking_create.restype=ctypes.c_void_p
  self.lib.tracking_destroy.argtypes=[ctypes.c_void_p]
  self.lib.tracking_clear.argtypes=[ctypes.c_void_p]
  self.lib.tracking_update.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
  self.handle=self.lib.tracking_create();self.reset()
 def reset(self,theta=None,vreq=0.,integral=0.,previous_duty=0.):
  self.lib.tracking_clear(self.handle);self.pending_init=(theta is not None);self.prior=round(previous_duty*1000);self.diagnostics={}
 def tick_q(self,*q,init_state=False,prior_duty_permille=0):
  x=(ctypes.c_int32*11)(*q,int(init_state),prior_duty_permille);y=(ctypes.c_int32*7)()
  self.lib.tracking_update(self.handle,x,y)
  if y[5]:raise RuntimeError('RTL input fault/timeout '+str(y[5]))
  self.diagnostics=dict(lean=y[1],acceleration_request=y[2],vreq=y[3],vi=y[4],cycles=y[6])
  return y[0]
 def tick(self,measured,reference=(0.,0.,0.,0.),*,angle_filtered=None):
  if angle_filtered is None:raise ValueError('Coherent filtered theta required')
  vals=[round(float(v)*Q) for v in (*measured,angle_filtered,*reference)]
  u=self.tick_q(*vals,init_state=self.pending_init,prior_duty_permille=self.prior)
  self.pending_init=False;return u/1000.
 def __del__(self):
  if getattr(self,'handle',None):self.lib.tracking_destroy(self.handle);self.handle=None
