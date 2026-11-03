"""Explicit complete-control FFI boundary: harness is legacy internal, public reports physical.
Only simulation tooling. Does not modify frozen RTL or claim full j280_top/UART ACK.
"""
from pathlib import Path
import ctypes as C,hashlib,importlib.util,math,re
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_coordinates(project):
 path=Path(project)/'host/coordinates.py';spec=importlib.util.spec_from_file_location('_full_rtl_coordinates_'+sha(path)[:12],path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def tick_exact(r):
 out=r.tick();idle=50000-(out['cycles']+1)
 if idle<0:raise RuntimeError('Control transaction exceeded 1ms clock budget')
 r.idle(idle)
 out['idle_clocks']=idle
 return out

def physical_output(out,coordinates,encoder=None):
 result=dict(out)
 for key in ('phi','vel','ref','ref_v'):
  result[key+'_internal']=out[key];result[key]=out[key]*coordinates.SCALE
 if encoder is not None:
  expected=max(-2000000,min(2000000,coordinates.signed32(encoder)*396))
  assert round(out['phi']*65536)==expected,'Estimator/raw count mismatch'
  result['phi']=coordinates.physical_q16(encoder)/65536
 result['coordinate_note']='FFI returns legacy internal values; this report adapter converts once. Public phi is host C101 arithmetic, not extra calibrated_position RTL in the harness.'
 return result

def configure_from_wire(r,T,K,word=310941374):
 w=(word&255)|(((word>>29)&1)<<8);a=(word>>8)&2047;speed=(word>>19)&3;period=(word>>21)&127;recovery=bool(word&(1<<28))
 assert word>>30==0 and 20<=w<=360 and 1<=a<=min(2047,10*(w-10)) and speed<=3 and period in(8,10,12,16,20,30,40,60,90,120)
 assert 9*a<=100*[1,2,3,6][speed]*period,'Config rejected by real board rate bound'
 top=(T/'fpga/src/j280_top.v').read_text()
 def table(name,key):
  text=top.split('function ',1)[1]if False else top[top.index(' '+name+';'):]
  text=text[:text.index('endfunction')]
  match=re.search(r'\b'+str(key)+r':'+name+r"=\d+'d(\d+);",text);assert match,(name,key)
  return int(match.group(1))
 hi=table('amp_hi_table',a>>5);lo=table('amp_lo_table',a&31);ampq=(hi>>3)+(lo>>3)+int((hi&7)+(lo&7)>=5);rangeq=w*1144
 r.configure(full_turn_recovery=recovery)
 r.lib.r14_config(r.ptr,rangeq,ampq,speed,period)
 readback=None
 if hasattr(r.lib,'r33_config_readback'):
  r.lib.r33_config_readback.argtypes=[C.c_void_p,C.POINTER(C.c_int32)];values=(C.c_int32*5)();r.lib.r33_config_readback(r.ptr,values);readback=list(values)
  assert readback==[rangeq,ampq,speed,period,int(recovery)],'FFI pin config readback mismatch'

 return dict(harness_pin_readback=readback,configuration_readback_available=readback is not None,wire_word=word,wire_hex=hex(word),workspace_internal_code_deg=w,amplitude_internal_code_tenths_deg=a,speed_code=speed,period_s=period,full_turn_recovery=recovery,workspace_internal_q16=rangeq,amplitude_internal_q16=ampq,workspace_physical_rad=rangeq/65536*K,amplitude_physical_rad=ampq/65536*K,source_j280_top_sha256=sha(T/'fpga/src/j280_top.v'),source_note='Exact wire configuration passed to this replay (legacy default is310941374); board integer LUT and range shift arithmetic, no double physical conversion.')

