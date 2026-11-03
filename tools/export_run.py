"""Export a real GUI CSV/experiment ZIP to degree-based CSV and its existing summary."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from studio_data import load_run, elapsed_ms

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('record',type=Path,nargs='?',default=ROOT/'examples/demo/run.csv')
    p.add_argument('--out',type=Path)
    a=p.parse_args()
    data=load_run(a.record)
    out=a.out or ROOT/'logs/analysis'/a.record.stem
    out.mkdir(parents=True,exist_ok=True)
    times,_=elapsed_ms(data['rows'])
    columns=['time_s','theta_deg','phi_deg','reference_deg','theta_speed_deg_s','arm_speed_deg_s','output_percent','state','fault']
    with (out/'curves_degrees.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(columns)
        for t,row in zip(times,data['rows']):
            w.writerow([t/1000,math.degrees(row['theta']),math.degrees(row['phi']),
                        math.degrees(row['reference']),math.degrees(row['dtheta']),
                        math.degrees(row['dphi']),row['u']/10,row['state'],row['fault']])
    (out/'summary.json').write_text(json.dumps(data['summary'],ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Exported',len(data['rows']),'frames to',out.resolve())

if __name__=='__main__':
    main()
