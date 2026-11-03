"""Read-only bundling: preserve all source bytes, reject mismatched board/time/cache.
No serial commands. Works with extracted folders or original ZIPs, including R12.
"""
from pathlib import Path
from project_identity import current_build,current_build_text
CURRENT_BUILD=current_build();CURRENT_BUILD_TEXT=current_build_text()
import argparse,csv,hashlib,io,json,zipfile,sys,traceback
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from studio_data import load_output_contract

def text_bytes(raw,label,encodings):
 # Strict decoding only. Never replace or discard invalid source bytes.
 if raw.startswith((b'\xff\xfe',b'\xfe\xff')):
  encoding='utf-16'
  try:value=raw.decode(encoding)
  except UnicodeError as e:raise ValueError(label+': invalid BOM-marked UTF-16: '+str(e)) from e
 else:
  try:value=raw.decode('utf-8-sig');encoding='utf-8-sig'
  except UnicodeDecodeError:
   try:value=raw.decode('gb18030');encoding='gb18030 (GBK/CP936 compatible)'
   except UnicodeDecodeError as e:raise ValueError(label+': neither UTF-8 nor GBK/GB18030; preserve this source file: '+str(e)) from e
 encodings[label]=encoding
 return value

def json_bytes(raw,label,encodings):
 try:return json.loads(text_bytes(raw,label,encodings))
 except json.JSONDecodeError as e:raise ValueError(label+': invalid JSON at line '+str(e.lineno)+', column '+str(e.colno)) from e

def build_id(value):
 if isinstance(value,int) and not isinstance(value,bool):return f'{value:08X}'
 if isinstance(value,str):
  try:return f'{int(value.upper().removeprefix("0X"),16):08X}'
  except ValueError:pass
 raise ValueError('Missing or invalid build identity')

def files(path):
 p=Path(path)
 if p.is_dir():return {f.relative_to(p).as_posix():f.read_bytes()for f in p.rglob('*')if f.is_file()}
 with zipfile.ZipFile(p)as z:
  out={}
  for n in z.namelist():
   q=Path(n.replace('\\','/'))
   if q.is_absolute()or '..'in q.parts or ':' in str(q):raise ValueError('Unsafe archive path: '+n)
   if n in out:raise ValueError('Duplicate archive entry: '+n)
   if not n.endswith('/'):out[n]=z.read(n)
  return out

def find(data,suffix):
 a=[(k,v)for k,v in data.items()if k.endswith(suffix)]
 if len(a)!=1:raise ValueError('Expected one '+suffix+', found '+str(len(a)))
 return a[0][1]

def bundle(run,passive,review,out):
 encodings={}
 rr,pp=files(run),files(passive);info=json_bytes(find(pp,'info.json'),'passive/info.json',encodings)
 live=[(k,v)for k,v in rr.items()if k.endswith('.csv')and Path(k).name.startswith('run_')]
 if len(live)!=1:raise ValueError('Choose exactly one run')
 rows=list(csv.DictReader(io.StringIO(text_bytes(live[0][1],'run/'+live[0][0],encodings))));events=[json.loads(s)for s in text_bytes(find(rr,'.commands.jsonl'),'run/commands.jsonl',encodings).splitlines()if s.strip()]
 raw_compact=any(Path(n).name=='raw_samples_1khz.csv'for n in pp)
 compact_summary=None
 if raw_compact:
  from compact_evidence import validate as validate_compact
  _,samples,compact_summary=validate_compact({Path(n).name:v for n,v in pp.items()},info['build'])
 else:
  samples=list(csv.DictReader(io.StringIO(text_bytes(find(pp,'samples_1khz.csv'),'passive/samples_1khz.csv',encodings))))
 if not rows or not samples:raise ValueError('Empty recording')
 cp=[e for e in events if e.get('event')=='observer_checkpoint' and e.get('capture_id')==info['capture_id'] and e.get('build')==info['build']]
 sample_ms={int(r['ms'])for r in samples}
 if raw_compact:
  rr_by_ms={int(float(r['ms'])):r for r in rows};common=[(r,rr_by_ms[int(r['ms'])])for r in samples if int(r['ms'])in rr_by_ms]
  keys=('u','state','fault','encoder')
  if len(common)<3 or any(any(int(float(a[k]))!=int(float(b[k]))for k in keys)for a,b in common):raise ValueError('Compact run/cache overlapping telemetry does not match')
  identities=[e for e in events if e.get('event') in ('cache_identity','compact_cache_identity') and all(e.get(k)==info.get(k)for k in ('build','capture_id','count'))]
  if not identities:raise ValueError('Compact run lacks matching frozen cache identity')
 binding='observer_checkpoint'
 if not any(int(e['ms'])in sample_ms for e in cp):
  # Failed swing can have no warmed observer checkpoint. Require explicit cache
  # identity plus independently matching timestamps and measured fields instead.
  identities=[e for e in events if e.get('event')=='cache_identity' and all(e.get(k)==info.get(k) for k in ('build','capture_id','count','phase'))]
  rr_by_ms={int(float(r['ms'])):r for r in rows};common=[(r,rr_by_ms[int(r['ms'])])for r in samples if int(r['ms'])in rr_by_ms]
  keys=('u','state','fault','encoder')
  matches=[a for a,b in common if all(int(float(a[k]))==int(float(b[k]))for k in keys)]
  if not identities or len(common)<3 or len(matches)!=len(common):raise ValueError('Run/cache identity or overlapping telemetry does not match; preserve separate source files')
  binding='cache_identity_and_overlapping_telemetry'

 # Existing manifests must match before joining; no rewriting failed source evidence.
 for group in [rr,pp]:
  m=json_bytes(find(group,'SHA256.json'),'manifest/'+('run' if group is rr else 'passive'),encodings);m=m.get('files',m)
  roots=[str(Path(k).parent)for k in group if k.endswith('SHA256.json')]
  base=''if roots[0]=='.'else roots[0]+'/'
  for n,sha in m.items():
   if isinstance(sha,dict):sha=sha['sha256']
   if hashlib.sha256(group[base+n]).hexdigest()!=sha:raise ValueError('Source manifest mismatch: '+n)
 payload={**{'run/'+n:v for n,v in rr.items()},**{'passive/'+n:v for n,v in pp.items()}}
 review_binding=None
 if review:
  raw=Path(review).read_bytes();rv=json_bytes(raw,'review/'+Path(review).name,encodings);csv_name=Path(live[0][0]).name
  if rv.get('schema')=='j280.r39.ext2-review.v1':
   # New function only: ordinary session/position reviews keep their exact gate.
   if (info.get('build')!=CURRENT_BUILD or rv.get('ext2',{}).get('config')!=75082174
       or rv.get('coordinate_schema')!=0xC101 or rv.get('passive_bound')is not True
       or rv.get('passive_capture_id')!=info.get('capture_id')
       or rv.get('passive_wire_sha256')!=hashlib.sha256(find(pp,'wire.bin')).hexdigest()
       or rv.get('source_csv_sha256')!=hashlib.sha256(live[0][1]).hexdigest()):
    raise ValueError('R39 Extension2 review is not bound to this exact physical run/passive cohort')
  declared=[rv[k] for k in ('build','evaluation_build','recorded_build') if k in rv]
  if not declared or any(build_id(v)!=build_id(info['build']) for v in declared):
   raise ValueError('Review build does not match run/passive: '+str(declared)+' vs '+build_id(info['build']))
  if rv.get('source_csv_sha256'):
   if rv['source_csv_sha256']!=hashlib.sha256(live[0][1]).hexdigest():raise ValueError('Review belongs to another run CSV')
   review_binding='csv_sha256'
  elif rv.get('input_file') and rv.get('input_sha256'):
   input_name=rv['input_file'].replace('\\','/').rsplit('/',1)[-1]
   run_path=Path(run)
   if input_name.lower().endswith('.csv'):
    if input_name!=csv_name or rv['input_sha256']!=hashlib.sha256(live[0][1]).hexdigest():raise ValueError('Review source CSV hash/name mismatch')
    review_binding='csv_sha256'
   elif run_path.is_file():
    if Path(input_name).stem!=Path(csv_name).stem:raise ValueError('Review filename does not match run')
    if hashlib.sha256(run_path.read_bytes()).hexdigest()!=rv['input_sha256']:raise ValueError('Review source ZIP hash mismatch')
    review_binding='zip_sha256'
   else:raise ValueError('Review refers to original ZIP; select that original run ZIP to verify its hash')
  else:raise ValueError('Review lacks verifiable source hash; select the original matching trial review, or cancel review selection and return the two source ZIPs')
  if raw_compact and rv.get('passive_wire_sha256'):
   if rv['passive_wire_sha256']!=hashlib.sha256(find(pp,'wire.bin')).hexdigest()or rv.get('passive_capture_id')!=info['capture_id']:raise ValueError('Review belongs to another passive cache')
  payload['derived/'+Path(review).name]=raw

 if info['build']==CURRENT_BUILD:
  from coordinates import descriptor_valid
  d=info.get('coordinate_descriptor')
  if not d or not descriptor_valid(d) or d['capture_id']!=info['capture_id']:raise ValueError('R36 cache lacks matching coordinate descriptor')
  payload['metadata/coordinate_descriptor.json']=(json.dumps(d,indent=2)+'\n').encode()
 receipt=dict(tool='R36 bundle 2.0.0',text_encodings=encodings,schema='j280.round.v1',build=f"{info['build']:08X}",capture_id=info['capture_id'],full_run_rows=len(rows),passive_1khz_rows=len(samples),matching_checkpoints=len(cp),binding=binding,measurement_sources=2,review_binding=review_binding,derived_review_is_not_a_third_measurement=True,scope='Whole-session 50Hz run and1kHz retained window overlap intentionally; source bytes preserved. Review is derived and its decision is not changed.')
 if compact_summary is not None:receipt['compact_evidence']=compact_summary
 contract=load_output_contract(info['build'])
 if contract is not None:
  receipt['output_contract']=contract
  receipt['output_contract_binding']='passive.info.build; source bytes preserved, not a full bitstream readback'
  payload['metadata/output_contract.json']=(json.dumps(contract,ensure_ascii=False,indent=2)+'\n').encode()
 payload['bundle_receipt.json']=(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n').encode()
 payload['SHA256.json']=(json.dumps({n:hashlib.sha256(v).hexdigest()for n,v in payload.items()},indent=2)+'\n').encode()
 out=Path(out)
 if out.exists():raise FileExistsError('Refuse to overwrite '+str(out))
 with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED)as z:
  for n,v in payload.items():z.writestr(n,v)
 return receipt
if __name__=='__main__':
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run');ap.add_argument('--passive');ap.add_argument('--review');ap.add_argument('--out');a=ap.parse_args()
 if not a.run:
  import tkinter as tk
  from tkinter import filedialog as fd,messagebox
  root=tk.Tk();root.withdraw();a.run=fd.askopenfilename(title='选择本轮 run ZIP',filetypes=[('ZIP','*.zip')]);a.passive=fd.askopenfilename(title='选择同轮 passive ZIP',filetypes=[('ZIP','*.zip')]);a.review=fd.askopenfilename(title='选择评估 JSON；没有可取消',filetypes=[('JSON','*.json')]);a.out=fd.asksaveasfilename(title='返回团队的单个实验包',defaultextension='.zip',initialfile='round_complete.zip')
  if not a.run or not a.passive or not a.out:raise SystemExit('Cancelled, no files changed')
 try:
  print(json.dumps(bundle(a.run,a.passive,a.review,a.out),ensure_ascii=True,indent=2))
  print('PASS - original run/passive/review bytes preserved in: '+str(a.out))
 except Exception as e:
  log=Path(__file__).resolve().parents[1]/'bundle_fix'/'logs'/('bundle_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.log')
  log.parent.mkdir(parents=True,exist_ok=True)
  log.write_text('run='+str(a.run)+'\npassive='+str(a.passive)+'\nreview='+str(a.review)+'\nout='+str(a.out)+'\n'+traceback.format_exc(),encoding='utf-8')
  raise SystemExit('FAILED: '+str(e)+'\nDiagnostic log: '+str(log)+'\nKeep all original files. No measurement or control data changed.')
