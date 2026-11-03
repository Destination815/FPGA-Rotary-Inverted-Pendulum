"""CompleteRTL +90/-90/0 profile; declaredM05plant, actual1mscommitintegration."""
from pathlib import Path
import argparse,json,sys
p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--speed',type=float,default=45);p.add_argument('--ms',type=int,default=64000);a=p.parse_args();P=a.project.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);sys.path.insert(0,str(P/'tools'));import check_r35_typical_full as full
case=dict(full.cases()['M05_swing_domain_constrained_1']);case.update(case='R43_position_zero',wire_config=(360&255)|((360>>8)<<29)|(150<<8)|(2<<19)|(20<<21)|(1<<28),numeric_speed_deg_s=a.speed,targets_wait_ready=False,target_schedule=[dict(ms=6000,physical_deg=90.),dict(ms=22000,physical_deg=-90.),dict(ms=42000,physical_deg=0.)])
plant,binding=full.compile_plant(out);r=full.one(P,out/'case',case['case'],a.ms,plant,binding,True,case);(out/'RESULT.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
if r['duration_ms']!=a.ms or r['fault'] or not r['target_schedule_completed']:raise SystemExit(1)
