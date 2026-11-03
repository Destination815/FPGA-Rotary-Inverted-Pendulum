"""Infer actual trial from accepted commands. Read-only; never requests another trial."""
from pathlib import Path
from project_identity import current_build,current_build_text
CURRENT_BUILD=current_build();CURRENT_BUILD_TEXT=current_build_text()
import argparse,hashlib,json,tempfile,zipfile,sys
P=Path(__file__).resolve().parents[1];sys.path[:0]=[str(P/'host'),str(P/'tools')]
from studio_data import load_run,summarize
from coordinates import SCHEMA,internal_row,target_internal_q16
from review_motion import assess,final_balance_rows

def review(path):
 path=Path(path)
 with tempfile.TemporaryDirectory() as td:
  root=path
  if path.is_file() and path.suffix.lower()=='.zip':
   with zipfile.ZipFile(path) as z:
    for n in z.namelist():
     if not (Path(td)/n).resolve().is_relative_to(Path(td).resolve()):raise ValueError('Unsafe ZIP path')
    z.extractall(td)
   root=Path(td)
  cs=list(root.rglob('run*.csv')) if root.is_dir() else [root]
  if len(cs)!=1:raise ValueError('请选择一轮 run，不能混合多轮')
  c=cs[0];r=load_run(c);
  if any(int(x.get('coordinate_schema',0))==SCHEMA for x in r['rows']):
   rr=[internal_row(x) for x in r['rows']];events=[dict(e,value_i32=target_internal_q16(e['value_i32'])) if e.get('event')=='send' and e.get('name')=='target' else dict(e) for e in r['commands']];r=dict(r,rows=rr,summary=summarize(rr),commands=events)
  acks=[e for e in r['commands']if e.get('event')=='ack'and e.get('status')==0];names={e.get('name')for e in acks}
  if not names.intersection({'start','capture'}):
   unarmed=all(not(int(x['flags'])&1) and x['u']==0 for x in r['rows'])
   return dict(decision='NOT_APPLICABLE_UNARMED_IDENTIFICATION' if unarmed else 'STOP_AND_RETURN',reason='no_accepted_start_or_capture',screen_met=None,assessment_applicable=False,phase='unarmed_identification' if unarmed else 'unclassified',source_csv_name=c.name,source_csv_sha256=hashlib.sha256(c.read_bytes()).hexdigest(),registered_bitstream=r['meta'].get('selected_bitstream'),build=CURRENT_BUILD_TEXT,phase_inferred_from_accepted_commands=False,previous_review_required=False,note='没有已接受的start/capture，不能按A/B保持或起摆评分；未使能自由摆动/手动带臂记录请使用专用未使能辨识工具。此结论不是起摆失败。')
  phase='swing'if 'start'in names else 'shuttle'if 'trajectory'in names else 'position'if 'target'in names else 'baseline'
  a=assess(r,phase);a.update(source_csv_name=c.name,source_csv_sha256=hashlib.sha256(c.read_bytes()).hexdigest(),registered_bitstream=r['meta'].get('selected_bitstream'),build=CURRENT_BUILD_TEXT,phase_inferred_from_accepted_commands=True,previous_review_required=False)
  active=[x for x in final_balance_rows(r['rows'])if not x['fault']]
  if phase=='position'and active:
   import math
   tail=[x for x in active if x['ms']>=active[-1]['ms']-2000]
   arrived=all(abs(math.degrees(x['reference']-x['target']))<.1 and abs(x.get('ref_velocity',1))<.001 for x in tail)
   a['reference_arrived_before_tail']=arrived
   if not arrived:a['screen_met']=False;a['decision']='STOP_AND_RETURN'
  a['missing_fs_registration']=not bool(a['registered_bitstream'])
  a['note']='run为全程50Hz记录；passive为真实1kHz末段缓存；review由前者计算，不是第三份独立实测。没有前一关JSON依赖。实际下载版本登记缺失会单列，不改写测量结果。'
  return a
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('record',nargs='?');ap.add_argument('--out');a=ap.parse_args()
 if not a.record:
  import tkinter as tk
  from tkinter import filedialog
  root=tk.Tk();root.withdraw();a.record=filedialog.askopenfilename(title='选择本轮 run ZIP',filetypes=[('run ZIP','*.zip')]);root.destroy()
  if not a.record:raise SystemExit('Cancelled')
 out=Path(a.out)if a.out else Path(a.record).with_suffix('.R23_review.json')
 if out.exists():raise FileExistsError('不覆盖已有评价：'+str(out))
 result=review(a.record);out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2));print(out)
