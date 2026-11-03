"""Read-only J280 millisecond diagnostic analysis; never opens serial or starts hardware."""
from pathlib import Path,PurePosixPath
from project_identity import current_build,current_build_text
CURRENT_BUILD=current_build();CURRENT_BUILD_TEXT=current_build_text()
import argparse,csv,io,json,hashlib,zipfile,math,sys

def inspect_payload(files, expected_build=CURRENT_BUILD):
 if 'raw_samples_1khz.csv' in files:
  from compact_evidence import validate,segments,startup
  info,rows,summary=validate(files,expected_build)
  rms=lambda a:math.sqrt(sum(v*v for v in a)/len(a)) if a else None
  summary.update(samples=len(rows),trigger_index=info['trigger_index'],reason=info['reason'],startup=startup(info,rows),
   contiguous_intervals=[dict(first_ms=b[0]['ms'],last_ms=b[-1]['ms'],samples=len(b),theta_rms_deg=rms([math.degrees(r['theta'])for r in b]),omega_rms=rms([r['dtheta']for r in b]))for b in segments(rows)],
   saturation_pct=100*sum(abs(r['u'])==r['limit']for r in rows)/len(rows),invalid_output_samples=sum(abs(r['u'])>r['limit']for r in rows),
   encoder_transitions=[dict(retained_index=b['retained_index'],source_ordinal=b['source_ordinal'],ms=b['ms'],encoder=b['encoder'],delta=b['encoder']-a['encoder'],logical_u=b['u'],logical_applied=-b['applied'],actual_theta_deg=math.degrees(b['theta']),actual_omega_rad_s=b['dtheta'],derived_phi_deg=math.degrees(b['phi']))for a,b in zip(rows,rows[1:])if b['source_ordinal']==a['source_ordinal']+1 and ((b['ms']-a['ms'])&0xffffffff)==1 and a['encoder']!=b['encoder']],
   interpretation='Actual theta/omega. Phi is explicitly derived from raw encoder/session zero/exact CPR. No filtered_theta, dphi or reference is fabricated. Head/tail are separate intervals; no middle interpolation.')
  return summary
 required=['info.json','samples_1khz.csv','wire.bin','SHA256.json']
 for name in required:
  if name not in files:raise ValueError('Missing '+name)
 m=json.loads(files['SHA256.json'])
 for name,h in m.items():
  if name not in files or hashlib.sha256(files[name]).hexdigest()!=h:raise ValueError('Hash mismatch '+name)
 if any(n not in m for n in required[:-1]):raise ValueError('Missing evidence hashes')
 info=json.loads(files['info.json']);rows=list(csv.DictReader(io.StringIO(files['samples_1khz.csv'].decode('utf-8-sig'))))
 if info['build']!=expected_build or not info['capture_id']&0x80000000 or info['phase']!=4:raise ValueError('Wrong build or not a frozen passive record')
 if not 0<len(rows)<=(4096 if expected_build in (0x4A070209,CURRENT_BUILD) else 1024) or len(rows)!=info['count']:raise ValueError('Count mismatch')
 for r in rows:
  for k in r:r[k]=float(r[k]) if k in ['theta','dtheta','phi','dphi','filtered_theta','reference','reference_velocity'] or k.endswith('_internal') else (r[k] in ('True','1') if k=='adc_fractional_valid' else int(r[k]))
  if not all(math.isfinite(float(v)) for v in r.values()):raise ValueError('Non-finite data')
 for i,r in enumerate(rows):
  if r['sample_index']!=i:raise ValueError('Index mismatch')
  if i and ((r['ms']-rows[i-1]['ms'])&0xffffffff)!=1:raise ValueError('Non-1ms interval')
 # Compare every exported field with fresh decoding of original received bytes.
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
 from protocol import Decoder
 decoder=Decoder();decoded={};records=[]
 for offset in range(0,len(files['wire.bin']),4096):
  decoder.feed(files['wire.bin'][offset:offset+4096]);records.extend(decoder.take_diagnostics())
 for item in records:
  if item['kind']=='sample' and item['capture_id']==info['capture_id']:
   if item['build']!=info['build'] or item['count']!=len(rows):raise ValueError('Wire record identity mismatch')
   i=item['index'];r=item['row']
   if i in decoded and decoded[i]!=r:raise ValueError('Conflicting duplicate sample')
   decoded[i]=r
 if set(decoded)!=set(range(len(rows))):raise ValueError('Wire incomplete')
 for i,r in enumerate(rows):
  if r!=decoded[i]:raise ValueError('CSV differs from wire at '+str(i))
 first_count=next((i for i in range(1,len(rows)) if rows[i]['encoder']!=rows[i-1]['encoder']),None)
 first_nonzero=next((i for i,r in enumerate(rows) if r['applied']),None)
 flips=[];last_sign=0
 for i,r in enumerate(rows):
  logical=-r['applied'];sgn=(logical>0)-(logical<0)
  if sgn and last_sign and sgn!=last_sign:flips.append(i)
  if sgn:last_sign=sgn
 events=[dict(sample=i,elapsed_ms=i,ms=r['ms'],encoder=r['encoder'],delta=r['encoder']-rows[i-1]['encoder'],logical_u=r['u'],logical_applied=-r['applied'],theta_deg=math.degrees(r['filtered_theta']),omega=r['dtheta'],estimated_arm_velocity=r['dphi']) for i,r in enumerate(rows) if i and r['encoder']!=rows[i-1]['encoder']]
 rms=lambda k:math.sqrt(sum(r[k]**2 for r in rows)/len(rows))
 return dict(build=f'{expected_build:08X}',samples=len(rows),interval_ms=1,wire_csv_exact=True,bad_wire_candidates=decoder.bad_frames,trigger_index=info['trigger_index'],reason=info.get('reason'),first_count_index=first_count,first_nonzero_applied_index=first_nonzero,observed_count_latency_ms=None if first_count is None or first_nonzero is None else first_count-first_nonzero,logical_pwm_sign_reversals=flips,encoder_transitions=events,omega_rms=rms('dtheta'),saturation_pct=100*sum(abs(r['u'])==r.get('limit',150) for r in rows)/len(rows),invalid_output_samples=sum(abs(r['u'])>r.get('limit',150) for r in rows),limit_range=[min(r.get('limit',150) for r in rows),max(r.get('limit',150) for r in rows)],recovery_boost_samples=sum(bool(r['flags']&1024) for r in rows),recovery_used_samples=sum(bool(r['flags']&2048) for r in rows),settled_samples=sum(bool(r['flags']&4096) for r in rows),interpretation='Only the retained window. First count is quantized encoder evidence, not exact mechanical startup or dead-zone measurement. applied is physical sign; logical applied=-applied. No 1kHz extrapolation and no gain recommendation from this summary alone.')

def read_files(path):
 if path.is_dir():return {p.name:p.read_bytes() for p in path.iterdir() if p.is_file()}
 out={}
 with zipfile.ZipFile(path) as z:
  for n in z.namelist():
   if n.endswith('/'):continue
   b=PurePosixPath(n).name
   if b in out:raise ValueError('Ambiguous duplicate basename '+b)
   out[b]=z.read(n)
 return out
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('record',type=Path);ap.add_argument('--build',choices=[CURRENT_BUILD_TEXT],required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 r=inspect_payload(read_files(a.record),int(a.build,16));a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print('PASS wire/CSV agreement; '+str(a.out))
