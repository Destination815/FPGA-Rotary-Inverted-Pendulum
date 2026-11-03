"""Validate C831 source bytes and expose measured unit conversions without fabricated fields."""
from pathlib import Path
import csv,io,json,hashlib,math,sys,zipfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
import compact_protocol as cp
from protocol import Decoder,CONTROL_BUILD
from coordinates import descriptor_valid,signed32

def payload(path):
 path=Path(path)
 if path.is_dir():items=[(p.name,p.read_bytes())for p in path.rglob('*')if p.is_file()]
 else:
  with zipfile.ZipFile(path)as z:items=[(Path(n).name,z.read(n))for n in z.namelist()if not n.endswith('/')]
 result={}
 for name,value in items:
  if name in result:raise ValueError('Ambiguous source basename '+name)
  result[name]=value
 return result

def validate(files,expected_build=CONTROL_BUILD):
 for name in ('raw_samples_1khz.csv','schema.json','info.json','wire.bin','SHA256.json'):
  if name not in files:raise ValueError('Missing '+name)
 manifest=json.loads(files['SHA256.json'])
 for name,sha in manifest.items():
  if name not in files or hashlib.sha256(files[name]).hexdigest()!=sha:raise ValueError('Hash mismatch '+name)
 if any(n not in manifest for n in ('raw_samples_1khz.csv','schema.json','info.json','wire.bin')):raise ValueError('Missing evidence hashes')
 schema=json.loads(files['schema.json']);info=json.loads(files['info.json'])
 if schema.get('schema')!=cp.SCHEMA or info['build']!=expected_build or not info['capture_id']&0x80000000:raise ValueError('Wrong schema/version/capture')
 decoder=Decoder();records=[]
 for offset in range(0,len(files['wire.bin']),4096):decoder.feed(files['wire.bin'][offset:offset+4096]);records.extend(decoder.take_diagnostics())
 infos=[r for r in records if r['kind']=='compact_info']
 if not infos:raise ValueError('No authoritative C831 INFO in wire')
 wire_info=infos[-1]
 if any(info.get(k)!=v for k,v in wire_info.items()):raise ValueError('INFO differs from wire')
 rows,summary=cp.validate_capture(wire_info,[r for r in records if r['kind']=='compact_data'])
 csvrows=list(csv.DictReader(io.StringIO(files['raw_samples_1khz.csv'].decode('utf-8-sig'))))
 csvrows=[{k:v if k=='segment' else int(v)for k,v in r.items()}for r in csvrows]
 if rows!=csvrows:raise ValueError('CSV differs from wire')
 descriptor=info.get('coordinate_descriptor')
 if not descriptor or not descriptor_valid(descriptor) or descriptor['capture_id']!=info['capture_id']:raise ValueError('Missing matching coordinate descriptor')
 wire_descriptors=[r for r in records if r['kind']=='coordinates' and r.get('capture_id')==info['capture_id']]
 if not any(all(r.get(k)==v for k,v in descriptor.items()if k!='verified')for r in wire_descriptors):raise ValueError('Coordinate descriptor differs from wire')
 zero=descriptor['recording_encoder_zero'];rpc=2*math.pi*descriptor['cpr_den']/descriptor['cpr_num']
 derived=[dict(r,theta=r['theta_q16']/65536,dtheta=r['dtheta_q16']/65536,u=r['command_pwm'],applied=r['applied_pwm'],phi=signed32(r['encoder']-zero)*rpc)for r in rows]
 summary.update(wire_csv_exact=True,build=f'{expected_build:08X}',bad_wire_candidates=decoder.bad_frames,phi_source=dict(kind='derived_from_raw_encoder_and_session_zero',formula='signed32(encoder-recording_encoder_zero) * 2*pi*cpr_den/cpr_num',encoder_zero=zero,cpr_num=descriptor['cpr_num'],cpr_den=descriptor['cpr_den']),missing_signals=schema['missing'])
 return info,derived,summary

def segments(rows):
 out=[]
 for r in rows:
  if not out or r['source_ordinal']!=out[-1][-1]['source_ordinal']+1 or ((r['ms']-out[-1][-1]['ms'])&0xffffffff)!=1:out.append([])
  out[-1].append(r)
 return out

def startup(info,rows):
 """First state2 is the committed original 30ms-confirmed controller state; do not re-time its gate."""
 head=[r for r in rows if r['segment']=='head'];begin=info['begin_ms']
 offset=lambda r:(r['ms']-begin)&0xffffffff
 aligned=bool(head and offset(head[0])<=1)
 continuous=bool(head and len(segments(head))==1)
 first3=[r for r in head if offset(r)<=3000]
 covers_first3=bool(aligned and first3 and offset(first3[-1])==3000 and len(segments(first3))==1)
 captures=[i for i,r in enumerate(head)if r['state']==2 and (i==0 or head[i-1]['state']in(1,6,7))]
 first=captures[0]if captures else None
 hold=head[first:first+1000]if first is not None else []
 complete=bool(len(hold)==1000 and len(segments(hold))==1 and all(r['state']in(2,3,4)and not r['fault']for r in hold))
 rms=lambda a:math.sqrt(sum(v*v for v in a)/len(a))if a else None
 values=[math.degrees(r['theta'])for r in hold];omegas=[r['dtheta']for r in hold]
 pre=head[max(0,first-29):first+1]if first is not None else []
 pre_stats=dict(samples=len(pre),continuous_1ms=bool(pre and len(segments(pre))==1),actual_theta_peak_deg=max(abs(math.degrees(r['theta']))for r in pre)if pre else None,actual_omega_peak_rad_s=max(abs(r['dtheta'])for r in pre)if pre else None,filtered_gate_replay_possible=False)
 delay=offset(head[first])/1000 if first is not None else None
 good=bool(info['automatic_session'] and info['reason']in(1,3) and aligned and covers_first3 and delay is not None and delay<=3 and complete and rms(values)<3 and max(map(abs,values))<=12)
 return dict(automatic_session=info['automatic_session'],begin_ms=begin,first_sample_alignment_valid=aligned,head_continuous_1ms=continuous,covers_first3s=covers_first3,confirmed_capture_ms=head[first]['ms']if first is not None else None,confirmed_capture_delay_s=delay,first_second_samples=len(hold),first_second_continuous_and_balanced=complete,first_second_theta_rms_deg=rms(values),first_second_theta_peak_deg=max(map(abs,values))if values else None,confirmed_actual_theta_deg=math.degrees(head[first]['theta'])if first is not None else None,confirmed_actual_omega_rad_s=head[first]['dtheta']if first is not None else None,first_second_omega_rms_rad_s=rms(omegas),first_second_omega_peak_rad_s=max(map(abs,omegas))if omegas else None,preceding_30_observations=pre_stats,strict_first3_and_first_second=good,thresholds=dict(capture_delay_s_max=3,first_second_theta_rms_deg_lt=3,first_second_theta_peak_deg_max=12),confirmation_source='actual state2 after original controller confirmation; actual theta_q16/dtheta_q16 and stored millis; filtered theta is absent',scope='Only retained head. Omitted middle requires whole-run telemetry; first-second thresholds are the unchanged RTL screen, whole-run stability keeps original stricter run criteria.')
