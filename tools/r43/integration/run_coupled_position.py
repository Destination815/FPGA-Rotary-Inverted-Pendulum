"""Frozen limited-calibration parameters extrapolated through fullRTL, never physical proof."""
from pathlib import Path
import argparse,sys,json,math,re,hashlib
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();P=a.project.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
T=Path(__file__).resolve().parents[3];sys.path.insert(0,str(T/'tools'));import check_r35_typical_full as full
full.BASE=P
s=(T/'tools/r42/sine_a7/plant.hpp').read_text();get=lambda name:[float(v)for v in re.search(name+r'\[\d+\]=\{([^}]+)\}',s)[1].split(',')];sensor=get('SENSOR');qpar=get('QPAR')
gamma=.3;bias_deg=.3899232695494066
case=dict(full.cases()['M05_swing_domain_constrained_1']);params=list(case['plant']);params[9]*=.5
case.update(case='R43_coupled_position_zero',plant=params,wire_config=(360&255)|((360>>8)<<29)|(150<<8)|(2<<19)|(20<<21)|(1<<28),numeric_speed_deg_s=45.,targets_wait_ready=False,target_schedule=[dict(ms=6000,physical_deg=90.),dict(ms=22000,physical_deg=-90.),dict(ms=42000,physical_deg=0.)])
def sense(x,p):
 sensed=math.atan2(math.sin(x[0]),math.cos(x[0]));sensed=sensed-2*math.pi if sensed>1.7/p[16]else sensed;sensed*=p[16]
 sensed+=gamma*(sensor[1]*x[1]+sensor[3]*x[3]+sensor[4]*qpar[0]/p[6]*x[6])+math.radians(bias_deg)
 code=(746+sensed/(386/65536))%(2*math.pi/(386/65536));blind=code>1023
 if blind:code=0.
 q=min(16368,max(0,round(code*16)));enc=round(x[2]/(396/65536));return q>>4,q,enc,96<=q<=16272,blind
full.sense_values=sense
pl,bind=full.compile_plant(O);r=full.one(P,O/'case',case['case'],64000,pl,bind,True,case)
r['scope']+=' NEW extrapolation: selected groups1/2 limited HOLD feature parameters p9*.5 and A7 derivative/motor measurement coupling*.3, fixed common tare+.389923deg. Full-angle/startup behavior not physically identified. Old M03 failed5degree windows remain as independent counterexamples, regardless of this result.'
r['extra_sensor_model']={'scale':gamma,'SENSOR':sensor,'QPAR_motor_gain':qpar[0],'bias_deg':bias_deg,'p9_scale':.5,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'sensor_fixture_sha256':hashlib.sha256((T/'tools/r42/sine_a7/plant.hpp').read_bytes()).hexdigest()}
(O/'RESULT.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
if r['duration_ms']!=64000 or r['fault'] or not r['target_schedule_completed']:raise SystemExit(1)
