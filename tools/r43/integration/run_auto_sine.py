"""Complete automatic controller chain into/out of sine; declared M05 stress plant."""
from pathlib import Path
import argparse,importlib.util,json,sys
p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--zero-ms',type=int,default=14010);p.add_argument('--ms',type=int,default=16000);a=p.parse_args();P=a.project.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(P/'tools'))
spec=importlib.util.spec_from_file_location('auto_engine',Path(__file__).with_name('auto_sequence_engine.py'));full=importlib.util.module_from_spec(spec);spec.loader.exec_module(full);full.BASE=P
case=dict(full.cases()['M05_swing_domain_constrained_1']);case.update(case='R43_automatic_sine_chain',wire_config=(360&255)|((360>>8)<<29)|(150<<8)|(2<<19)|(20<<21)|(1<<28),numeric_speed_deg_s=60.,sine_amplitude=90.,sine_period=9.42,sine_schedule=[dict(ms=6000,enabled=True),dict(ms=12000,enabled=False)],targets_wait_ready=False,target_schedule=[dict(ms=a.zero_ms,physical_deg=0.)])
plant,binding=full.compile_plant(out);r=full.one(P,out/'case',case['case'],a.ms,plant,binding,True,case)
r['scope']+=' Automatic down-start, fixed 6000ms sine-on / 12000ms sine-off / configured zero time; no quiet/readiness wait in test harness. This is a mode/commit contract stress test, not a validated nonlinear sine-performance prediction.'
(out/'RESULT.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
if r['duration_ms']!=a.ms or r['fault'] or r['sine_active_rows']==0 or not r['target_schedule_completed'] or not r['sine_active_transitions'] or r['sine_active_transitions'][-1]['active']:raise SystemExit(1)
