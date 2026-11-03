"""Read-only R23 wire audit and manual checkpoint-seeded exact observer replay.
A tail alone does not determine seven hidden controller states. Supply run events
when available; missing history is reported, never substituted with R04 arithmetic.
Automatic sessions use fixed-frame submission: a row's u is the command already
committed from the previous frame, not the newly computed pending result.
"""
from pathlib import Path
from project_identity import current_build,current_build_text
CURRENT_BUILD=current_build();CURRENT_BUILD_TEXT=current_build_text()
import argparse,csv,io,json,math,sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'host'),str(ROOT/'observer')]
from analyze_passive import inspect_payload,read_files
from fixed_model import Observer
from coordinates import internal_row,KAPPA,SCHEMA
from studio_data import load_output_contract
BUILD=CURRENT_BUILD

def replay_observer(rows,checkpoints,reference):
 byms={int(r['ms']):i for i,r in enumerate(rows)};checked={};state_checks=0;bad=[];model=None;last_ms=None;prior=None
 cps={int(c['ms']):c for c in checkpoints if c.get('build')==BUILD and c.get('format')=='r13_integral_q24_post_commit' and c.get('ready') and c.get('samples')==200 and abs(c['reference_origin']-reference)<1/65536}
 for r in rows:
  ms=int(r['ms'])
  if model is not None and ms==last_ms+1 and r['state'] in (2,3,4) and not r['fault']:
   u,ready,fault=model.tick(round(r['theta']*65536),round((r['phi']-reference)*65536),prior,r['limit'],round((r['reference']-reference)*65536),round(r['reference_velocity']*65536));checked[ms]=u
   if u!=r['u'] or fault:bad.append(dict(ms=ms,expected=u,actual=r['u'],oracle_fault=fault))
  else:model=None
  if ms in cps:
   c=cps[ms]
   if c['u']!=r['u']:raise ValueError('Checkpoint/live tail mismatch at '+str(ms))
   if model is not None:
    state_checks+=1
    if model.z!=c['z'] or model.integral!=c['integral'] or model.lp!=c['integral_lp']:bad.append(dict(ms=ms,reason='checkpoint_state_mismatch',expected=model.z,actual=c['z']))
   model=Observer();model.z=list(c['z']);model.samples=c['samples'];model.ready=int(c['ready']);model.integral=c['integral'];model.lp=c['integral_lp'];model.age=c['age']
  last_ms=ms;prior=r['u']
 return dict(control_rows_checked=len(checked),control_mismatches=bad,checkpoint_states_checked=state_checks,checkpoint_count=sum(ms in byms for ms in cps),history_status='checkpoint_seeded'if checked else 'no_usable_checkpoint; arithmetic not reconstructed')

def analyze(path,build,local_capture,reference_rad=None,events=None):
 if local_capture and reference_rad is None:raise ValueError('Provide capture reference; never assume zero.')
 ref=0. if reference_rad is None else float(reference_rad)
 if not math.isfinite(ref):raise ValueError('non-finite reference')
 if build!=BUILD:raise ValueError('Use matching firmware analyzer.')
 files=read_files(Path(path));audit=inspect_payload(files,build)
 rows=list(csv.DictReader(io.StringIO(files['samples_1khz.csv'].decode('utf-8-sig'))));rows=[{k:float(v)if k in ('theta','dtheta','phi','dphi','filtered_theta','reference','reference_velocity') or k.endswith('_internal') else int(v in ('True','1')) if k=='adc_fractional_valid' else int(v) for k,v in r.items()}for r in rows]
 info=json.loads(files['info.json']);cp=[]
 if events:
  cp=[r for r in (json.loads(line)for line in Path(events).read_text(encoding='utf-8-sig').splitlines()if line.strip())if r.get('event')=='observer_checkpoint' and r.get('capture_id')==info['capture_id']]
 contract=load_output_contract(build)
 if local_capture:
  replay=replay_observer([internal_row(r) for r in rows],cp,ref*KAPPA)
 else:
  replay=dict(control_rows_checked=0,control_mismatches=[],checkpoint_states_checked=0,checkpoint_count=len(cp),history_status='not_replayed: automatic fixed-frame submission; same-row observer/u equivalence is not a valid contract')
 # applied is carrier-latched after polarity mapping and a full-period reversal
 # coast. A 1 kHz tail has no carrier phase/enable history: neither -u[row] nor
 # -u[row-1] is a universally valid equality, in manual or automatic sessions.
 result=dict(audit=audit,rows=len(rows),reference_rad=ref,output_pipeline_mismatches=[],output_pipeline_status='not_checked: carrier phase, enable and reversal history are not recorded at 50 MHz; no guessed row-shift equality',session_contract='manual_local_capture' if local_capture else 'automatic_fixed_frame',output_contract=contract,scope=__doc__,**replay)
 terms=[dict(ms=r['ms'],theta_deg=math.degrees(r['theta']),phi_error_deg=math.degrees(r['phi']-ref),actual_u=r['u']/1000,limit=r['limit']/1000)for r in rows]
 return result,terms,rows
if __name__=='__main__':
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('record',type=Path);a.add_argument('--build',default=CURRENT_BUILD_TEXT);a.add_argument('--local-capture',action='store_true');a.add_argument('--reference-rad',type=float);a.add_argument('--run-events',type=Path);a.add_argument('--out',type=Path,required=True);args=a.parse_args()
 result,terms,rows=analyze(args.record,int(args.build,16),args.local_capture,args.reference_rad,args.run_events);args.out.mkdir(parents=True,exist_ok=True);(args.out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 if result['control_mismatches'] or result['output_pipeline_mismatches']:raise SystemExit(2)
