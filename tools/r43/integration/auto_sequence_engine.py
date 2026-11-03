#!/usr/bin/env python3
"""R35 fixed representative complete RTL, optional declared velocity impulse, four-phase metrics."""
from pathlib import Path
import argparse,ctypes as C,hashlib,importlib.util,json,math,re,subprocess,sys,time
import numpy as np
BASE=Path(__file__).resolve().parents[1]
from full_rtl_coordinates import configure_from_wire
CASE_NAMES=[f'M05_{domain}_domain_constrained_{sign}'for domain in ('swing','handoff')for sign in(-1,1)]
COLUMNS=['t_s','true_theta_rad','true_omega_rad_s','true_phi_physical_rad','true_v_physical_rad_s','theta_rad','omega_rad_s','phi_physical_rad','v_physical_rad_s','pwm_fraction','rtl_state','reference_physical_rad','reference_velocity_physical_rad_s','cycles_to_done','fault','active_limit_fraction','filtered_theta_rad','gap_active','gap_ms','measurement_valid','handoff_recovery','raw_encoder','adc','adc_q4','adc_q4_valid','true_load_phi_physical_rad','true_load_v_physical_rad_s','phi_internal_rad','v_internal_rad_s','reference_internal_rad','reference_velocity_internal_rad_s','ext2_active']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def load(path,name):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def binding(T):
 provpath=T/'build/r01/rtl_library_provenance.json';prov=json.loads(provpath.read_text());libpath=T/('build/r01/libr01_control.dylib'if sys.platform=='darwin'else'build/r01/libr01_control.so')
 assert sha(libpath)==prov['library_sha256'],'Library SHA mismatch'
 for f,h in prov['source_sha256'].items():assert sha(T/f)==h,'Source not bound to RTL: '+f
 return dict(rtl_library_sha256=sha(libpath),rtl_source_sha256=prov['source_sha256'],provenance_sha256=sha(provpath),ffi_sha256=sha(T/'sim/rtl_controller.py'),coordinates_sha256=sha(T/'host/coordinates.py'),calibration_sha256=sha(T/'calibration/encoder_calibration.json'))

def cases():
 spec=json.loads((BASE/'evidence/r32_swing_selected/scenarios.json').read_text());result={r['name']:r for r in spec['cases']if r['name']in CASE_NAMES};bank=json.loads((BASE/'models/calibrated/backend/model_bank.json').read_text())['models']
 for name,c in result.items():
  domain=name[4:].rsplit('_',1)[0];assert c['plant']==bank[domain]['p'];assert c['bounce']==c['event']==0 and c['mass']>=2
 return result

def sense_values(x,p):
 sensed=math.atan2(math.sin(x[0]),math.cos(x[0]));sensed=sensed-2*math.pi if sensed>1.7/p[16]else sensed;sensed*=p[16]
 code=(746+sensed/(386/65536))%(2*math.pi/(386/65536));blind=code>1023
 if blind:code=0.
 q=min(16368,max(0,round(code*16)));enc=round(x[2]/(396/65536))
 return (q>>4,q,enc,96<=q<=16272,blind)

def clock_frame(r,fill=True):
 o=r.tick();idle=50000-(o['cycles']+1);assert idle>=0,(o['cycles'],idle)
 if fill:r.idle(idle)
 o['idle_clocks']=idle if fill else 0
 last=-1
 for cycle,u in o['commits']:
  assert 0<=cycle<=o['cycles'] and cycle>=last,(cycle,last,o['cycles']);last=cycle
 return o

def compile_plant(O):
 header=BASE/'evidence/r32_swing_selected/m03_plant.hpp';(O/'m03_plant.hpp').write_bytes(header.read_bytes())
 cc=O/'plant.cpp';cc.write_text('#include "m03_plant.hpp"\nextern "C" void advance(double*x,double u,const double*p,double dt,int sub){m03_step(x,u,p,dt,sub);}\n')
 libpath=O/('plant.dylib'if sys.platform=='darwin'else'plant.so');cmd=['c++','-O3','-shared','-fPIC','-std=c++17',str(cc),'-o',str(libpath)];subprocess.run(cmd,check=True)
 lib=C.CDLL(str(libpath));lib.advance.argtypes=[C.POINTER(C.c_double),C.c_double,C.POINTER(C.c_double),C.c_double,C.c_int]
 return lib,dict(header_sha256=sha(header),source_sha256=sha(cc),library_sha256=sha(libpath),compile_command=cmd)

def advance_commit_frame(pl,x,p,old,o,j,commits):
 ptr=C.POINTER(C.c_double);elapsed=0.;applied=old
 for cycle,u in o['commits']:
  when=cycle/50e6;dt=when-elapsed
  assert 0<=dt<=.001
  if dt:pl.advance(x.ctypes.data_as(ptr),applied,p.ctypes.data_as(ptr),dt,max(1,math.ceil(dt/.00025)))
  elapsed=when;applied=u;commits.append([j,cycle,u])
 dt=.001-elapsed
 if dt:pl.advance(x.ctypes.data_as(ptr),applied,p.ctypes.data_as(ptr),dt,max(1,math.ceil(dt/.00025)))
 assert applied==o['u'],(j,applied,o['u'])
 assert np.isfinite(x).all(),'Plant diverged'
 return applied

def one(T,O,name,horizon,pl,pbinding,fill=True,case_override=None):
 O.mkdir(parents=True,exist_ok=False);before=binding(T);c=cases()[name]if case_override is None else case_override;p=np.array(c['plant'],float);coord=load(T/'host/coordinates.py','coordinates_'+str(time.time_ns()));K=coord.SCALE
 assert coord.LEGACY_Q16==396 and coord.RPC_Q24==97763
 R=load(T/'sim/rtl_controller.py','rtl_'+str(time.time_ns())).RTL;r=R();config=configure_from_wire(r,T,K,c.get('wire_config',310941374))
 if 'numeric_speed_deg_s' in c:r.numeric_speed(physical_deg_s=c['numeric_speed_deg_s'])
 if 'sine_period' in c:r.sine_parameters(c['sine_amplitude'],c['sine_period'])
 x=np.array([c['sign']*(math.pi-.025)+p[15]+c.get('initial_delta',0.),0.,0.,0.,0.,0.,0.]);initial=x.copy();warm=[];rows=[];commits=[];captures=[];falls=[];transitions=[];last=1;old=0.;first_origin=None;last_origin=None;beg=time.monotonic()
 def sense():
  adc,q,enc,fine,blind=sense_values(x,p);r.fraction(adc,q,enc,fractional_valid=fine);return adc,q,enc,fine,blind
 for k in range(40):sense();warm.append(clock_frame(r,fill))
 assert all(o['u']==0 for o in warm),'Unarmed warmup motion'
 o=warm[-1]
 r.begin()
 try:
  eventtick=None;target_events=[];target_index=0;sine_events=[];sine_index=0
  for j in range(horizon):
   sines=c.get('sine_schedule',[])
   if sine_index<len(sines) and j>=sines[sine_index]['ms']:
    ev=sines[sine_index];r.ext2(ev['enabled']);sine_events.append(dict(ev,actual_ms=j));sine_index+=1
   schedule=c.get('target_schedule',[])
   if target_index<len(schedule):
    target_event=schedule[target_index]
    due=j>=target_event['ms']
    admitted=not c.get('targets_wait_ready',False) or (o['ready'] and o['state'] in (3,4) and not o['trajectory'])
    if due and admitted:
     r.command(6,math.radians(target_event['physical_deg'])/K)
     target_events.append(dict(target_event,actual_ms=j,internal_rad=math.radians(target_event['physical_deg'])/K))
     target_index+=1
   if c.get('event',0) and captures and j==captures[0]+c.get('event_delay_ms',6000):
    x[1]+=c['event'];eventtick=j
   adc,q,enc,fine,blind=sense();o=clock_frame(r,fill)
   if o['state']!=last:transitions.append(dict(tick=j,from_state=last,to_state=o['state'],fault=o['fault']))
   if o['state']==2 and last in(1,6,7):
    captures.append(j);last_origin=o['phi']*K
    if first_origin is None:first_origin=last_origin
   if last in(2,3,4)and o['state']in(1,6,7):falls.append(j)
   assert round(o['phi']*65536)==max(-2000000,min(2000000,enc*396)),'Raw encoder/internal estimator phi mismatch'
   public_phi=coord.physical_q16(enc)/65536
   assert abs(public_phi-o['phi']*K)<=.5/65536+1e-12,'C101 physical conversion rounding mismatch'
   rows.append([j*.001,x[0],x[1],x[2]*K,x[3]*K,o['theta'],o['omega'],public_phi,o['vel']*K,o['u'],o['state'],o['ref']*K,o['ref_v']*K,o['cycles'],o['fault'],o['limit'],o['filtered'],int(o['gap']),o['gap_ms'],int(o['measurement_ok']),int(o['handoff_recovery']),enc,adc,q,int(fine),x[4]*K,x[5]*K,o['phi'],o['vel'],o['ref'],o['ref_v'],int(r.ext2_active())]);last=o['state']
   if o['fault']:break
   old=advance_commit_frame(pl,x,p,old,o,j,commits)
   if j%2000==0:print(json.dumps(dict(case=name,ms=j,state=o['state'],fault=o['fault'],captures_ms=captures,wall_s=round(time.monotonic()-beg,2))),flush=True)
  r.stop();sense();stopped=clock_frame(r,fill)
 finally:r.close()
 arr=np.array(rows);first=captures[0]if captures else None;hold=arr[first:first+1000]if first is not None else arr[:0];tail=arr[-1000:];pre=arr[:first+1]if first is not None else arr;deg=lambda z:np.arctan2(np.sin(z),np.cos(z))*180/math.pi
 rms=lambda a:float(np.sqrt(np.mean(np.asarray(a)**2)))if len(a)else None
 hold_theta=deg(hold[:,1]);tail_theta=deg(tail[:,1]);theta_rms=rms(tail_theta);hold_rms=rms(hold_theta);hold_peak=float(np.max(abs(hold_theta)))if len(hold)else None
 tail_ref=rms(np.degrees(tail[:,3]-tail[:,11]));tail_home=rms(np.degrees(tail[:,3]-first_origin))if first_origin is not None else None
 failures=[t for t in transitions if t['from_state']==6 and t['to_state']in(1,7)and(first is None or t['tick']<first)]
 np.savez_compressed(O/'trace.npz',data=arr,commits=np.array(commits).reshape(-1,3));(O/'columns.json').write_text(json.dumps(COLUMNS,indent=2));(O/'warmup.json').write_text(json.dumps(warm,indent=2))
 strict=bool(first is not None and first<=3000 and len(hold)==1000 and hold_rms<3 and hold_peak<=12 and theta_rms<3 and not falls and not o['fault']and o['state']in(2,3,4)and len(arr)==horizon)
 result=dict(case=name,build=json.loads((T/'candidate.json').read_text())['build'],horizon_ms=horizon,duration_ms=len(arr),captures_ms=captures,first_capture_s=None if first is None else first*.001,first_brake_s=next((t['tick']*.001 for t in transitions if t['to_state']==6),None),failed_brakes_before_capture=len(failures),falls_ms=falls,fault=o['fault'],final_state=o['state'],strict3=strict,first_second_theta_rms_deg=hold_rms,first_second_theta_peak_deg=hold_peak,theta_tail_rms_deg=theta_rms,arm_tail_reference_rms_deg=tail_ref,arm_tail_first_origin_rms_deg=tail_home,first_origin_physical_rad=first_origin,last_origin_physical_rad=last_origin,arm_peak_before_capture_deg=float(np.max(abs(np.degrees(pre[:,3])))),arm_travel_before_capture_deg=float(np.sum(abs(np.diff(np.degrees(pre[:,3]))))),arm_peak_all_deg=float(np.max(abs(np.degrees(arr[:,3])))),arm_travel_all_deg=float(np.sum(abs(np.diff(np.degrees(arr[:,3]))))),first_second_arm_peak_from_origin_deg=float(np.max(abs(np.degrees(hold[:,3]-first_origin))))if first_origin is not None else None,maximum_cycles=int(np.max(arr[:,13])),pwm_commit_events=len(commits),commit_cycle_min=min((q[1]for q in commits),default=None),commit_cycle_max=max((q[1]for q in commits),default=None),maximum_pwm=float(np.max(abs(arr[:,9]))),stop_zero=stopped['u']==0,stop_guard_clear=not stopped['handoff_recovery'],elapsed_s=time.monotonic()-beg,transitions=transitions,configuration=config,clock_contract='50MHz; each 1ms frame: tick consumes cycles_to_done+1 clocks then r20_idle fills exactly50000; PWM integrated at recorded zero-based commit_cycle/50MHz; begin command adds three setup clocks before first frame; no PWM carrier/electrical model.',fill_idle=fill,coordinate_contract='M05 plant keeps legacy encoder/load phi coordinates; raw encoder=round(phi/(396/65536)); RTL FFI phi/v/ref/config are INTERNAL; checked internal phi=clamp(encoder*396). Public physical phi computed by host.coordinates.physical_q16 (not an extra instantiated calibrated_position RTL); legacy velocity/reference/plant phi converted exactly once with host.coordinates.SCALE; theta/PWM unchanged.',plant_binding=pbinding,case_parameters=c,initial_legacy_state=initial.tolist(),m05_backend_sha256=sha(BASE/'models/calibrated/backend/model_bank.json'),model_manifest_sha256=sha(BASE/'evidence/r32_swing_selected/scenarios.json'),rtl_binding=before,runner_sha256=sha(__file__),trace_sha256=sha(O/'trace.npz'),scope='Complete controller/estimator/reference/guard RTL, M05 domain-parameter stress extrapolation. These domain fits have short prediction windows and are not validated full-angle physical plants. No integer CPU substitute, no future truth in controller, no hardware/STA guarantee.')
 assert binding(T)==before,'Sources/library changed during run';assert result['stop_zero']and result['stop_guard_clear'];result['bindings_unchanged_after_run']=True
 result['intervention_ms']=eventtick
 result['target_events']=target_events
 result['sine_events']=sine_events
 result['sine_active_rows']=int(arr[:,31].sum())
 result['sine_active_transitions']=[dict(ms=int(k),active=bool(arr[k,31]),state=int(arr[k,10]),fault=int(arr[k,14]))for k in np.flatnonzero(np.diff(arr[:,31]))+1]
 result['target_schedule_completed']=target_index==len(c.get('target_schedule',[]))
 result['numeric_speed_deg_s']=c.get('numeric_speed_deg_s')
 if target_events:
  final_goal=target_events[-1]['physical_deg']
  result['final_position_goal_physical_deg']=final_goal
  result['final_position_actual_mean_physical_deg']=float(np.mean(np.degrees(tail[:,3])))
  result['final_position_reference_mean_physical_deg']=float(np.mean(np.degrees(tail[:,11])))
  result['final_position_error_rms_deg']=rms(np.degrees(tail[:,3])-final_goal)
  result['peak_reference_speed_physical_deg_s']=float(np.max(abs(np.degrees(arr[:,12]))))
  result['motion_theta_peak_deg']=float(np.max(abs(deg(arr[c['target_schedule'][0]['ms']:,1]))))
 result['intervention_omega_rad_s']=c.get('event',0)
 result['strict_first_capture3']=first is not None and first<=3000 and len(hold)==1000 and hold_rms<3 and hold_peak<=12
 result['unforced_falls_before_intervention']=[v for v in falls if eventtick is None or v<eventtick]
 rec=next((v for v in captures if eventtick is not None and v>eventtick),None)
 exit=next((v for v in falls if eventtick is not None and v>=eventtick),None)
 result['post_intervention_recapture_s']=(rec-eventtick)*.001 if rec is not None else None
 result['post_exit_recapture_s']=(rec-exit)*.001 if rec is not None and exit is not None else None
 result['local_recenter_reference_changes']=int(sum((abs(np.diff(arr[:,11]))>1e-6)&(arr[1:,10]==3)&(arr[:-1,10]==3)))
 result['scope']+=' Optional event is a declared instantaneous angular-velocity impulse at first capture+'+str(c.get('event_delay_ms',6000))+'ms; no measured human force was identified. Disturbed falls do not count as pre-event spontaneous failures. First and recovered phases reported separately.'
 (O/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:result[k]for k in ['case','first_capture_s','strict3','fault','falls_ms','first_second_theta_rms_deg','arm_peak_before_capture_deg','arm_travel_before_capture_deg','maximum_cycles','elapsed_s']}),flush=True)
 return result

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--rtl-project',type=Path,default=BASE);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--case',choices=CASE_NAMES+['all'],default='all');ap.add_argument('--ms',type=int,default=18000);ap.add_argument('--legacy-compressed-idle',action='store_true',help='Validation diagnostic only: omit idle clocks as older pipeline did.');a=ap.parse_args();T=a.rtl_project.resolve();binding(T);O=a.out.resolve();O.mkdir(parents=True,exist_ok=False);pl,pb=compile_plant(O)
 selected=CASE_NAMES if a.case=='all'else[a.case];out=[]
 for name in selected:out.append(one(T,O/name,name,a.ms,pl,pb,not a.legacy_compressed_idle));(O/'summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
