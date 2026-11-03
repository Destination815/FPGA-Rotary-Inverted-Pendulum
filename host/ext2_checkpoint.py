"""R40 read-only Extension2 checkpoints, separate from C101 and C831.

Public telemetry is physical. These recursively restorable observer words are
explicit LEGACY INTERNAL Q24; never pass them through public_row or infer a
missing delta. Constants are bound by source_manifest, never packet readback.
"""
import binascii,struct
from coordinates import BUILD
TAG=0xE239
R40_STATE_WORD_SCHEMA="r40-v3-transfer8-unwind1"
B_NAMES=('disturbance_q24','sine_bias_internal_q24','sine_position_ff_q24',
         'sine_reference_internal_q24','feedback_lp_q24','shape_fast_q24','shape_slow_q24')
C_NAMES=('smooth_position_internal_q24','smooth_step_internal_q24',
         'smooth_normalized_q24','smooth_previous_feedback_q24','smooth_delta_q24',
         'smooth_bound_q24','smooth_innovation_internal_q24')

def decode(frame):
 if len(frame)!=64 or frame[:2]!=b'\xa5\x5a' or frame[2] not in (9,10) or frame[3]!=64 or binascii.crc_hqx(frame[2:62],0xffff)!=int.from_bytes(frame[62:64],'little'):
  raise ValueError('invalid R39 Extension2 checkpoint frame')
 u16=lambda p:struct.unpack_from('<H',frame,p)[0]
 i16=lambda p:struct.unpack_from('<h',frame,p)[0]
 i32=lambda p:struct.unpack_from('<i',frame,p)[0]
 u32=lambda p:struct.unpack_from('<I',frame,p)[0]
 is_b=frame[2]==9
 if u32(4)!=BUILD or u16(50)!=TAG or u16(48)!=(0x1901 if is_b else 0x1a01) or u16(60)!=(0x1701 if is_b else 0x1801):
  raise ValueError('R39 Extension2 checkpoint identity/format mismatch')
 out=dict(kind='ext2_checkpoint_b' if is_b else 'ext2_checkpoint_c',build=BUILD,
          ms=u32(8),capture_id=u32(12),u=i16(20 if is_b else 16),
          coordinate_units='legacy_internal_q24',format=u16(48),tag=u16(50))
 if is_b:
  out.update(zip(B_NAMES,(i32(x)for x in (16,28,32,36,44,52,56))))
  out.update(age=u16(22),reference_origin=i32(24)/65536,flags=u32(40))
 else:
  out.update(zip(C_NAMES,(i32(x)for x in (20,24,28,32,36,40,44))))
  out['flags']=u16(18)
  state_word=u32(52)
  if state_word&~0x1ff or (state_word&255)>128:raise ValueError('invalid R40 v3 state word')
  out['profile_transfer_ticks']=state_word&255
  out['position_unwind_active']=bool(state_word&256)
  out['state_word_schema']=R40_STATE_WORD_SCHEMA
 return out

def join(a,b,c):
 """Strict same-cohort collection; missing data never replaced by zero."""
 if any(type(p)is not dict for p in (a,b,c)):raise ValueError('complete type4/9/10 cohort required')
 if a.get('kind')!='observer_checkpoint' or b.get('kind')!='ext2_checkpoint_b' or c.get('kind')!='ext2_checkpoint_c':raise ValueError('checkpoint types differ')
 for k in ('build','ms','capture_id','u'):
  if a.get(k)!=b.get(k) or a.get(k)!=c.get(k):raise ValueError('checkpoint cohort mismatch: '+k)
 if a['build']!=BUILD or b.get('tag')!=TAG or c.get('tag')!=TAG:raise ValueError('old/foreign build is not R39')
 if a.get('age')!=b.get('age') or a.get('reference_origin')!=b.get('reference_origin') or (b.get('flags',0)&63)!=c.get('flags'):raise ValueError('checkpoint state or flags mismatch')
 if type(a.get('z')) is not list or len(a['z'])!=7:raise ValueError('seven base observer words required')
 for n in (*B_NAMES,*C_NAMES):
  v=(b if n in B_NAMES else c).get(n)
  if type(v)is not int or not -(1<<31)<=v<(1<<31):raise ValueError('missing/invalid recursive state: '+n)
 if not 0<=c.get('profile_transfer_ticks',-1)<=128:raise ValueError('missing/invalid R40 transfer counter')
 if type(c.get('position_unwind_active'))is not bool or c.get('state_word_schema')!=R40_STATE_WORD_SCHEMA:raise ValueError('missing/invalid R40 v3 unwind state')
 if c['position_unwind_active'] and b.get('flags',0)&(32|32768):raise ValueError('ordinary unwind authorization cannot survive a fault or cross into Extension2')
 flags=b['flags']
 if flags&~65535:raise ValueError('unknown Extension2 flags')
 if ((flags>>6)&255)>200:raise ValueError('invalid observer samples')
 if abs(c['smooth_delta_q24'])>167772 or c['smooth_bound_q24'] not in (0,167772):raise ValueError('smooth correction bound')
 if flags&8 and (not flags&4 or c['smooth_bound_q24']!=167772):raise ValueError('active smoothing lacks valid state')
 if not flags&8 and (c['smooth_delta_q24'] or c['smooth_bound_q24']):raise ValueError('inactive smoothing must be clear')
 return dict(a,ext2=dict(b,**{k:v for k,v in c.items()if k not in ('kind','age','reference_origin','flags')}),
             restoration_scope='R40 A observer words, transfer counter and unwind authorization; other ordinary retained memories and centered reference require a full seamless snapshot for continuation')


def native_state(a,b,c):
 """Convert a complete R39 cohort to the strict20-field local oracle state.

 Fault/ready/samples are explicitly packetized in B flags; none is guessed
 from today's model or silently set to zero. Constants are separately source-bound.
 """
 join(a,b,c)
 f=b['flags']
 if c['profile_transfer_ticks'] or c['position_unwind_active']:raise ValueError('profile transition or retained ordinary unwind requires a complete R40 seamless snapshot')
 if not f&32:raise ValueError('checkpoint did not execute Extension2 observer')
 return dict(z=a['z']+[b['disturbance_q24']],integral=a['integral'],lp=a['integral_lp'],
  age=a['age'],samples=(f>>6)&255,fault=int(bool(f&32768)),ready=int(bool(f&16384)),
  bias_clipped=bool(f&16),feedback_lp=b['feedback_lp_q24'],shape_fast=b['shape_fast_q24'],
  shape_slow=b['shape_slow_q24'],shape_valid=bool(f&1),sine_bias=b['sine_bias_internal_q24'],
  sine_valid=bool(f&2),sine_position_ff=b['sine_position_ff_q24'],sine_reference=b['sine_reference_internal_q24'],
  smooth_position=c['smooth_position_internal_q24'],smooth_velocity_step=c['smooth_step_internal_q24'],
  smooth_valid=bool(f&4),smooth_correction=c['smooth_delta_q24'])
