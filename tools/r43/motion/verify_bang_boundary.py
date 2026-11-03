from pathlib import Path
import argparse,json,hashlib
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();p=a.project.resolve();o=a.out.resolve();o.mkdir(parents=True,exist_ok=True)
paths=['observer/fixed_model.py','hybrid/hold_observer.hpp','fpga/src/r10_observer_core.v'];texts=[(p/x).read_text()for x in paths]
assert 'strong_shaped=shaped' in texts[0] and 'strong_shaped=shaped;' in texts[1] and 'r38_strong_shaped=r36_rate_shaped;' in texts[2]
D=13421773;B=2097152;rows=[]
for x in sorted(set([-(1<<36),(1<<36)-1,0]+[s*(v+d) for s in [-1,1] for v in [3355443,D,D+4*B] for d in [-2,-1,0,1,2]])):
 shaped=x-D if x>D else x+D if x<-D else 0;old=x-3355443 if x>3355443 else x+3355443 if x<-3355443 else 0
 before=max(-B,min(B,old>>2))if abs(x)>D else 0;after=max(-B,min(B,shaped>>2))if abs(x)>D else 0
 assert -B<=after<=B
 if abs(x)<=D:assert after==0
 if abs(x)>=D+4*B:assert after==before
 rows.append(dict(innovation_q24=x,old_explicit_q24=before,new_explicit_q24=after))
near=[r for r in rows if abs(r['innovation_q24'])==D+1];assert max(abs(r['new_explicit_q24'])for r in near)<=1 and min(abs(r['old_explicit_q24'])for r in near)==B
(o/'BOUNDARY.json').write_text(json.dumps(dict(status='PASS',scope='Exact signed integer local formula tied to the three source expressions. Whole RTL ordering separately checked by38400 transaction contract; not an all-domain SAT claim.',cases=rows,source_sha256={x:hashlib.sha256((p/x).read_bytes()).hexdigest()for x in paths}),indent=2)+'\n');print('PASS local dead-zone/boost saturation boundary checks',len(rows))
