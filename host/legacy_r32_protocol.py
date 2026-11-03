"""J280 wire format, little endian, CRC-16/CCITT-FALSE (init FFFF)."""
import binascii,struct
from collections import deque
from legacy_r32_coordinates import BUILD as CAL_BUILD,SCHEMA,public_row,descriptor_valid
Q=65536
STATES={0:'IDLE',1:'SWING',2:'CAPTURE',3:'HOLD',4:'TRACK',5:'FAULT',6:'BRAKE',7:'COAST'}
FAULTS={0:'none',1:'arm_limit',2:'sensor_invalid',3:'control_watchdog',4:'fallen',5:'swing_timeout',6:'arm_speed',7:'arithmetic_or_drive_output',8:'session_timeout',9:'reference_reject'}
STATUS={0:'accepted',1:'unknown_opcode',2:'wrong_state_or_gain_range',3:'out_of_range',4:'calibration_or_arm_precondition'}
OPS={'stop':1,'cal':2,'arm':3,'start':4,'capture':5,'target':38,'trajectory':7,'clear':8,'jog':9,'ktheta':16,'kdtheta':17,'kphi':18,'kdphi':19,'limit':20,'swing':21}
OPS.update(motioncfg=54,d02info=48,d02start=49,d02read=50)
D02_BUILD=0x4a020101  # legacy, offline replay only
D03_BUILD=0x4a030101
CONTROL_BUILD=CAL_BUILD
FRACTIONAL_BUILDS=(0x4a07022f,CONTROL_BUILD)
DIAGNOSTIC_BUILDS=(D02_BUILD,D03_BUILD)
def crc(data):return binascii.crc_hqx(data,0xffff)
def command(seq,opcode,value=0):
 body=struct.pack('<BBi',seq&255,opcode,value)
 return b'\xaa\x55'+body+struct.pack('<H',crc(body))
def decode(frame):
 if len(frame)!=64 or frame[:2]!=b'\xa5\x5a' or frame[2] not in (1,5) or frame[3]!=64 or crc(frame[2:62])!=int.from_bytes(frame[62:64],'little'):raise ValueError('invalid telemetry')
 i32=lambda at:struct.unpack_from('<i',frame,at)[0]
 u16=lambda at:struct.unpack_from('<H',frame,at)[0]
 result={'ms':struct.unpack_from('<I',frame,4)[0]}
 for at,name in [(8,'theta'),(12,'dtheta'),(16,'phi'),(20,'dphi'),(24,'reference'),(28,'ref_velocity')]:result[name]=i32(at)/Q
 result.update(u=i32(32),adc=u16(36),zero=u16(38),state=frame[40],fault=frame[41],flags=u16(42),ack_seq=frame[44],ack_status=frame[45],bad_commands=u16(46),encoder=i32(48),target=i32(52)/Q,limit=u16(56),encoder_illegal=u16(58),dropped=u16(60))
 return public_row(result,live=True) if frame[2]==5 else result
class Decoder:
 def __init__(self):self.buffer=bytearray();self.bad_frames=0;self.diagnostics=deque(maxlen=512);self.coordinates=None;self.contract_errors=0
 def feed(self,data):
  self.buffer.extend(data);out=[]
  while len(self.buffer)>=4:
   start=self.buffer.find(b'\xa5\x5a')
   if start<0:self.buffer[:]=self.buffer[-1:];break
   if start:del self.buffer[:start]
   if len(self.buffer)<64:break
   try:
    frame=self.buffer[:64]
    if frame[2] in (2,3,4,6):
     record=decode_d02(frame,coordinates=self.coordinates)
     if record['kind']=='coordinates':
      record['verified']=descriptor_valid(record)
      self.coordinates=record if record['verified'] else None
      if not record['verified']:self.contract_errors+=1
     if record['kind']=='info' and record['build']!=CONTROL_BUILD:self.coordinates=None
     self.diagnostics.append(record);del self.buffer[:64];continue
    record=decode(frame)
   except ValueError:del self.buffer[0];self.bad_frames+=1
   else:out.append(record);del self.buffer[:64]
  return out
 def take_diagnostics(self):
  result=list(self.diagnostics);self.diagnostics.clear();return result

def decode_d02(frame,*,coordinates=None):
 if len(frame)!=64 or frame[:2]!=b'\xa5\x5a' or frame[3]!=64 or crc(frame[2:62])!=int.from_bytes(frame[62:64],'little'):
  raise ValueError('invalid D02 packet')
 u16=lambda p:struct.unpack_from('<H',frame,p)[0]
 u32=lambda p:struct.unpack_from('<I',frame,p)[0]
 i16=lambda p:struct.unpack_from('<h',frame,p)[0]
 if frame[2]==6:
  if u16(60)!=0xc101:raise ValueError('invalid coordinate descriptor tag')
  return dict(kind='coordinates',build=u32(4),contract_token=u32(8),schema=u16(12),descriptor_flags=u16(14),cpr_num=u32(16),cpr_den=u32(20),legacy_q16_per_count=u32(24),physical_rpc_q24=u32(28),target_k_q24=u32(32),encoder_zero=struct.unpack_from('<i',frame,36)[0],recording_encoder_zero=struct.unpack_from('<i',frame,40)[0],capture_id=u32(44),ms=u32(48))
 if frame[2]==2:
  info = dict(kind='info',build=u32(4),capture_id=u32(8),phase=frame[12],reason=frame[13],count=u16(14),
   trigger_index=u16(16),interval_ms=u16(18),amplitude=i16(20),limit=u16(22),start_ms=u32(24),ms=u32(28),
   encoder_cpr=u16(32),motor_sign=struct.unpack_from('<b',frame,34)[0],channel=frame[35],sample_bytes=u16(36),
   capabilities=u16(38),capacity=u16(40),ready=bool(frame[42]),dumping=bool(frame[43]))
  if info['build'] in FRACTIONAL_BUILDS and u16(48)==0xF401:
   info.update(adc_q4=u16(44),zero_q4=u16(46),fractional_valid=bool(frame[50]),fractional_ready=bool(frame[51]),filter_format='adc_mean_q4_512_v1')
  if info['build'] in FRACTIONAL_BUILDS and u16(56)==0x1401:info['motion_config']=u32(52)
  if info['build']==CONTROL_BUILD:info.update(coordinate_schema=SCHEMA,physical_rpc_q24=u32(58),encoder_cpr_is_rounded=True)
  return info
 if frame[2]==4 and u32(4) in FRACTIONAL_BUILDS and u16(60)==0x1301:
  age=u16(46)
  return dict(kind='observer_checkpoint',build=u32(4),ms=u32(8),capture_id=u32(12),z=list(struct.unpack_from('<7i',frame,16)),u=struct.unpack_from('<h',frame,44)[0],reference_origin=struct.unpack_from('<i',frame,48)[0]/Q,coordinate_units='legacy_internal' if u32(4)==CONTROL_BUILD else 'legacy_original',integral=struct.unpack_from('<i',frame,52)[0],integral_lp=struct.unpack_from('<i',frame,56)[0],age=age,samples=min(age,200),ready=age>200,format='r13_integral_q24_post_commit')
 if frame[2]==4:
  checkpoint=dict(kind='observer_checkpoint',build=u32(4),ms=u32(8),capture_id=u32(12),z=list(struct.unpack_from('<7i',frame,16)),u=struct.unpack_from('<i',frame,44)[0],reference=struct.unpack_from('<i',frame,48)[0]/Q,samples=frame[52],ready=bool(frame[53]),state=frame[54],fault=frame[55],limit=u16(56),flags=u16(58),format='normalized_q24_post_commit')
  if u32(4) in FRACTIONAL_BUILDS and u16(60)==0x1201:
   checkpoint['reference_origin']=checkpoint.pop('reference');checkpoint['format']='r12_fixed_origin_q24_post_commit'
  return checkpoint
 if frame[2]!=3:raise ValueError('unknown D02 frame')
 values=struct.unpack_from('<IHBBHHi iiiihhHH'.replace(' ',''),frame,12)
 names='ms sample_index state fault adc zero encoder theta dtheta phi dphi u applied flags encoder_illegal'.split()
 row=dict(zip(names,values))
 for k in ('theta','dtheta','phi','dphi'):row[k]/=Q
 if u32(4)&0x80000000 and u32(52) in FRACTIONAL_BUILDS:
  # R10 repurposes redundant payload index for transaction cap; header index remains exact.
  rv_high=(row['sample_index']>>15)&1;row['adc_q4']=((row['sample_index']>>10)&31)<<9 | (row['fault']>>4)<<5 | (row['state']>>3);row['state']&=7;row['fault']&=15;row['limit']=row['sample_index']&1023;row['sample_index']=u16(8)
  q=u16(10)|(u16(60)<<16);row['adc_raw']=q>>22;row['adc_fractional_valid']=bool((q>>21)&1);q=q&0x1fffff;q=q-(1<<21) if q&(1<<20) else q;row['filtered_theta']=q/262144
  ref=row.pop('adc')|(row.pop('zero')<<16);ref=ref-(1<<32) if ref&(1<<31) else ref;row['reference']=ref/Q
  rv=row.pop('encoder_illegal')|(rv_high<<16);row['reference_velocity']=(rv-131072 if rv&65536 else rv)/Q
 if u32(52)==CONTROL_BUILD:
  if not coordinates or not descriptor_valid(coordinates) or coordinates['capture_id']!=u32(4):raise ValueError('missing/mismatched capture-bound coordinates')
  row=public_row(row,encoder_zero=coordinates['recording_encoder_zero'])
 return dict(kind='sample',capture_id=u32(4),index=u16(8),row=row,build=u32(52),count=u16(56),reason=frame[58],phase=frame[59])
