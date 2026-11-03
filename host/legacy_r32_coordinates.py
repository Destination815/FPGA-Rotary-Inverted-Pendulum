"""R32 physical/legacy boundary. All public arm angles are physical radians.
The controller's legacy coordinates are intentionally retained and explicitly logged.
"""
from pathlib import Path
import json,math
PROFILE=json.loads((Path(__file__).resolve().parents[1]/'calibration/legacy_r32_encoder_calibration.json').read_text(encoding='utf-8'))
BUILD=int(PROFILE['build'],16);TOKEN=PROFILE['contract_token'];SCHEMA=PROFILE['wire_schema']
RPC_Q24=PROFILE['physical_rpc_q24'];LEGACY_Q16=PROFILE['legacy_q16_per_count'];TARGET_K=PROFILE['target_legacy_per_physical_q24']
SCALE=RPC_Q24/(LEGACY_Q16*256);KAPPA=1/SCALE
EXTRA_FIELDS='phi_internal dphi_internal reference_internal ref_velocity_internal target_internal coordinate_schema phi_valid'.split()
def signed32(x):return (int(x)+(1<<31))%(1<<32)-(1<<31)
def symmetric_shift(x,bits):return (int(x)+(1<<(bits-1)))>>bits if x>=0 else -((-int(x)+(1<<(bits-1)))>>bits)
def physical_q16(count,zero=0):
 v=symmetric_shift(signed32(int(count)-int(zero))*RPC_Q24,8)
 return max(-(1<<31),min((1<<31)-1,v))
def target_internal_q16(value):
 if not isinstance(value,int) or not -(1<<31)<=value<(1<<31):raise ValueError('physical target Q16 outside int32')
 v=symmetric_shift(value*TARGET_K,24)
 if not -(1<<31)<=v<(1<<31):raise ValueError('target conversion overflow')
 return v

def internal_row(row):
 out=dict(row)
 if int(row.get('coordinate_schema',0))==SCHEMA:
  for key in ('phi','dphi','reference','ref_velocity','reference_velocity','target'):
   if key+'_internal' in row:out[key]=row[key+'_internal']
 return out

def public_row(row, *, live=False, encoder_zero=None):
 out=dict(row)
 if int(out.get('coordinate_schema',0))==SCHEMA:raise ValueError('Already physical: double conversion rejected')
 out['coordinate_schema']=SCHEMA;out['phi_valid']=int(not(int(row.get('flags',0))&32768))
 for key in ('dphi','reference','ref_velocity','reference_velocity','target'):
  if key in row:
   out[key+'_internal']=row[key];out[key]=row[key]*SCALE
 if live:
  # Type5 phi is physical Q16, calculated on board from the same tick's raw count.
  if out['phi_valid']:
   dq=round(round(row['phi']*65536)*256/RPC_Q24)
   out['phi_internal']=max(-2000000,min(2000000,dq*LEGACY_Q16))/65536
  else:out['phi_internal']=0.0
 else:
  if encoder_zero is None:raise ValueError('R32 passive sample requires capture-bound encoder zero descriptor')
  out['phi_internal']=row['phi'];out['encoder_zero']=encoder_zero
  out['phi']=physical_q16(row['encoder'],encoder_zero)/65536
 return out

def descriptor_valid(p):
 return all(p.get(k)==v for k,v in dict(build=BUILD,contract_token=TOKEN,schema=SCHEMA,cpr_num=PROFILE['cpr_num'],cpr_den=PROFILE['cpr_den'],legacy_q16_per_count=LEGACY_Q16,physical_rpc_q24=RPC_Q24,target_k_q24=TARGET_K).items())
