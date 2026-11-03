"""Read-only assessment for R13 run folder/ZIP; never sends serial commands.
Thresholds are engineering screening criteria, not official contest tolerances.
"""
from pathlib import Path
from project_identity import current_build,current_build_text
CURRENT_BUILD=current_build();CURRENT_BUILD_TEXT=current_build_text()
import argparse,json,sys,math,tempfile,zipfile,hashlib
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'host'))
from studio_data import load_run

def final_balance_rows(rows):
 """A new swing/coast interrupts holding; earlier blocks cannot pad its duration."""
 block=[];last=[]
 for row in rows:
  if row['state'] in (2,3,4):block.append(row)
  elif block:last=block;block=[]
 return block or last

def assess(run,phase):
 rows=run['rows'];active=final_balance_rows(rows)
 coast=[r for r in rows if r['state']==7]
 coast_zero=all(r.get('u',0)==0 and not(int(r['flags'])&16) for r in coast)
 if not active:return dict(decision='STOP_AND_RETURN',reason='no_active_capture',phase=phase,coast_observed_frames=len(coast),coast_zero_output_valid=coast_zero)
 start=active[0]['ms'];end=active[-1]['ms'];duration=(end-start)/1000+.02
 tail=[r for r in active if r['ms']>=end-2000]
 rms=lambda values:math.sqrt(sum(v*v for v in values)/len(values))
 stats=dict(phase=phase,duration_s=duration,faults=sorted({r['fault']for r in rows if r['fault']}),theta_rms_deg=rms([math.degrees(r['theta'])for r in active]),theta_peak_deg=max(abs(math.degrees(r['theta']))for r in active),position_tail_rms_deg=rms([math.degrees(r['phi']-r['reference'])for r in tail]),theta_tail_rms_deg=rms([math.degrees(r['theta'])for r in tail]),reference_span_deg=math.degrees(max(r['reference']for r in active)-min(r['reference']for r in active)),target_final_deg=math.degrees(active[-1]['target']),reference_final_deg=math.degrees(active[-1]['reference']),settled_frames=sum(bool(r['flags']&4096)for r in active))
 events=run['commands'];accepted=[e for e in events if e.get('event')=='ack' and e.get('status')==0];names=[e.get('name')for e in accepted]
 start_ack=next((e for e in accepted if e.get('name')=='start'),None)
 swing_start=next((r['ms']for r in rows if r['state'] in (1,6,7)),None)
 if start_ack and start_ack.get('ms') is not None:swing_start=min(swing_start,start_ack['ms']) if swing_start is not None else start_ack['ms']
 first_capture=next(r['ms'] for r in rows if r['state'] in (2,3,4))
 stats['swing_capture_delay_s']=(first_capture-swing_start)/1000 if swing_start is not None else None
 session_rows=[r for r in rows if r['state'] in (1,2,3,4,6,7)]
 stats['swing_session_observed_duration_s']=(session_rows[-1]['ms']-swing_start)/1000+.02 if session_rows and swing_start is not None else None
 stats['coast_observed_frames']=len(coast);stats['coast_zero_output_valid']=coast_zero
 stats['final_active_state']=session_rows[-1]['state'] if session_rows else None
 swing_ok=bool(start_ack and swing_start is not None and 0<=stats['swing_capture_delay_s']<=12)
 motion_present=(phase=='baseline' or (phase=='swing' and swing_ok) or (phase=='position' and 'target'in names and stats['reference_span_deg']>.5) or (phase=='shuttle' and 'trajectory'in names and stats['reference_span_deg']>4))
 stats['motion_command_observed']=motion_present
 stats['matching_build']=run.get('meta',{}).get('expected_build')==CURRENT_BUILD_TEXT
 stats['actual_arm_span_deg']=math.degrees(max(r['phi']for r in active)-min(r['phi']for r in active))
 stats['actual_motion_observed']=phase in ('baseline','swing') or stats['actual_arm_span_deg']>=.5*stats['reference_span_deg']
 stats['screen_met']=stats['actual_motion_observed'] and stats['matching_build'] and not stats['faults'] and coast_zero and stats['final_active_state'] in (2,3,4) and duration>=10 and motion_present and stats['theta_tail_rms_deg']<=1.5 and stats['position_tail_rms_deg']<=(1 if phase=='position' else 3)
 stats['decision']='SCREEN_MET_REQUIRES_OPERATOR_REVIEW' if stats['screen_met'] else 'STOP_AND_RETURN'
 stats['interpretation']='Not automatic authorization. Do not repeat movement to improve the score; return the combined run + passive bundle. A moving reference without matching actual arm motion is not a passed positioning test.'
 return stats

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('record',type=Path);ap.add_argument('--phase',choices=['baseline','position','shuttle','swing'],required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 with tempfile.TemporaryDirectory()as t:
  root=a.record
  if root.is_file() and root.suffix.lower()=='.zip':
   with zipfile.ZipFile(root)as z:
    for name in z.namelist():
     if not (Path(t)/name).resolve().is_relative_to(Path(t).resolve()):raise ValueError('Unsafe ZIP path')
    z.extractall(t)
   root=Path(t)
  files=list(root.rglob('run*.csv')) if root.is_dir() else [root]
  if len(files)!=1:raise ValueError('Choose one unambiguous run ZIP/folder')
  run=load_run(files[0]);r=assess(run,a.phase);r['source_csv_name']=files[0].name;r['source_csv_sha256']=hashlib.sha256(files[0].read_bytes()).hexdigest();r['registered_bitstream']=run['meta'].get('selected_bitstream');a.out.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');print(json.dumps(r,indent=2,ensure_ascii=False))
if __name__=='__main__':main()


def assess_compact_capture(files,expected_build=CURRENT_BUILD):
 from compact_evidence import validate,startup
 info,rows,summary=validate(files,expected_build)
 result=startup(info,rows)
 return dict(result,schema=summary['schema'],phi_source=summary['phi_source'],wire_csv_exact=True,
  passive_capture_id=info['capture_id'],passive_wire_sha256=hashlib.sha256(files['wire.bin']).hexdigest(),
  full_session_continuity_claimed=False)
