"""Sourcebound completeRTL recovery, exact1ms frames and physicalPWM commit times."""
from pathlib import Path
import argparse,json,sys
O=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--case-index',type=int,default=0);p.add_argument('--ms',type=int,default=18000);a=p.parse_args();P=a.project.resolve();out=a.out.resolve();out.mkdir(exist_ok=False,parents=True);sys.path.insert(0,str(P/'tools'));import check_r35_typical_full as full
cases=json.loads((O/'RECOVERY_CASES.json').read_text());case=cases[a.case_index]
plant,binding=full.compile_plant(out)
r=full.one(P,out/'case',case['case'],a.ms,plant,binding,True,case)
(out/'RESULT.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
if r['duration_ms']!=a.ms or r['fault'] or r['post_exit_recapture_s']is None:raise SystemExit(1)
