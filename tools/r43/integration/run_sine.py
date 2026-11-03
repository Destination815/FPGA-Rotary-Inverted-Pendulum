"""Actual complete controller/estimator RTL, declared local A plant, one continuous session.
Not a full-angle physical model, vendor implementation or hardware result.
"""
from pathlib import Path
import argparse,ctypes as C,hashlib,importlib.util,json,math,sys,time
import numpy as np
from scipy.linalg import expm

def load(p,name):
 s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--ms',type=int,default=36000);ap.add_argument('--amplitude',type=float,default=90);ap.add_argument('--period',type=float,default=12.57);ap.add_argument('--speed',type=float,default=45);a=ap.parse_args();exit_ms=round((12+2*a.period)*1000);a.ms=exit_ms+8000;P=a.project.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
 prov=P/'build/r01/rtl_library_provenance.json';binding=json.loads(prov.read_text());assert all(sha(P/n)==h for n,h in binding['source_sha256'].items());lib=P/('build/r01/libr01_control.dylib'if sys.platform=='darwin'else'build/r01/libr01_control.so');assert sha(lib)==binding['library_sha256']
 RTL=load(P/'sim/rtl_controller.py','rtl_seq').RTL;co=load(P/'host/coordinates.py','coords_seq');K=co.SCALE;r=RTL();r.configure(workspace=360,amplitude=15,speed=2,period=20,full_turn_recovery=True);r.numeric_speed(physical_deg_s=a.speed);r.sine_parameters(a.amplitude,a.period)
 modelpath=P/'models/ext2_a391/plant_prior.json';prior=json.loads(modelpath.read_text())['original']['one'];A=np.array(prior['A']);B=np.array(prior['B']);Q=prior['q'];Ctheta=np.array(prior['C'])[0];aug=np.zeros((9,9));aug[:7,:7]=A;aug[:7,7:9]=B;cache={};x=np.zeros(7);x[0]=math.radians(.1)
 old=0.;rows=[];commits=[];events=[];maxcycles=0;beg=time.monotonic();stage=0;active_seen=False;transitions=[];last=None
 def sense():
  measured=float(Ctheta@x);q=int(np.clip(round((746+measured/(386/65536))*16),0,16368));enc=round(x[2]/(396/65536));r.fraction(q>>4,q,enc,fractional_valid=96<=q<=16272);return q,enc
 def frame():
  o=r.tick();assert o['cycles']+1<=50000;r.idle(50000-o['cycles']-1);return o
 def advance(o):
  nonlocal x,old
  previous=0;u=old;friction=Q[11]*math.tanh(x[3]/.1)
  for cyc,value in [*o['commits'],(50000,o['u'])]:
   clocks=cyc-previous;assert clocks>=0
   if clocks:
    if clocks not in cache:cache[clocks]=expm(aug*(clocks/50e6))
    q=cache[clocks];eff=max(0.,u-Q[9]) if u>=0 else min(0.,u+Q[9])*Q[8];x=q[:7,:7]@x+q[:7,7]*eff+q[:7,8]*friction
   previous=cyc;u=value
  old=o['u']
 for j in range(40):sense();o=frame();assert o['u']==0
 r.begin(manual=True)
 try:
  for j in range(a.ms):
   ready=o['state']in(3,4)and o['ready']and not o['trajectory'];active=r.ext2_active();active_seen|=active
   if stage==0 and j>=3000:r.command(6,math.radians(20)/K);events.append(dict(ms=j,command='target',physical_deg=20));stage=1
   elif stage==1 and j>=8000:r.ext2(True);events.append(dict(ms=j,command='ext2',value=1));stage=2
   elif stage==2 and j>=exit_ms:r.ext2(False);events.append(dict(ms=j,command='ext2',value=0));stage=3
   elif stage==3 and j>=exit_ms+4000:r.command(6,0);events.append(dict(ms=j,command='target',physical_deg=0));stage=4
   q,enc=sense();o=frame();commits.extend([j,cy,u]for cy,u in o['commits']);maxcycles=max(maxcycles,o['cycles']);active=r.ext2_active()
   if (o['state'],active)!=last:transitions.append(dict(ms=j,state=o['state'],ext2=active,fault=o['fault']));last=(o['state'],active)
   rows.append([j*.001,x[0],x[1],x[2]*K,x[3]*K,o['theta'],o['omega'],o['phi']*K,o['vel']*K,o['u'],o['state'],o['ref']*K,o['ref_v']*K,o['cycles'],o['fault'],o['limit'],active,q,enc])
   if o['fault'] or abs(x[0])>.5:break
   advance(o)
   if j%4000==0:print(json.dumps(dict(ms=j,state=o['state'],ext2=active,wall_s=round(time.monotonic()-beg,1))),flush=True)
  r.stop();sense();stopped=frame();assert stopped['u']==0 and not r.ext2_active()
 finally:r.close()
 arr=np.array(rows);np.savez_compressed(O/'trace.npz',data=arr,commits=np.array(commits).reshape(-1,3));cols=['t_s','true_theta_rad','true_omega_rad_s','true_phi_physical_rad','true_v_physical_rad_s','theta_rad','omega_rad_s','phi_physical_rad','v_physical_rad_s','pwm_fraction','rtl_state','reference_physical_rad','reference_velocity_physical_rad_s','cycles_to_done','fault','active_limit_fraction','ext2_active','adc_q4','raw_encoder'];(O/'columns.json').write_text(json.dumps(cols,indent=2)+'\n')
 windows={};rms=lambda x:float(np.sqrt(np.mean(x*x)))
 for name,lo,hi in [('ordinary_before',2,3),('position_before',6,8),('sine',12,exit_ms/1000),('exit',exit_ms/1000,exit_ms/1000+4),('ordinary_after',exit_ms/1000+5,a.ms/1000)]:
  ix=(arr[:,0]>=lo)&(arr[:,0]<hi)
  if ix.any():windows[name]=dict(rows=int(sum(ix)),theta_rms_deg=rms(np.rad2deg(arr[ix,1])),tracking_rms_deg=rms(np.rad2deg(arr[ix,3]-arr[ix,11])),tracking_mean_deg=float(np.rad2deg(arr[ix,3]-arr[ix,11]).mean()),pwm_rms=rms(arr[ix,9])*1000)
 res=dict(settings=dict(amplitude_physical_deg=a.amplitude,period_s=a.period,speed_physical_deg_s=a.speed),status='COMPLETED'if len(arr)==a.ms and stage==4 and not o['fault']else'INCOMPLETE',model='A original declared 7-state local plant with original deadzone/asymmetric gain/friction, C sensor map, Q4 ADC/quantized encoder',request_schedule='Fixed wall times, no readiness/quiet gate in harness; ext2_active and reference must prove admission separately',scope='Actual controller, estimator, observer, reference, guard and CPU RTL; manual capture then target20, continuous parameterized sine entry/two full periods/exit, target0, STOP. UART/top admission tested separately. Local plant is not an independently identified full-angle hardware model.',duration_ms=len(arr),maximum_cycles=maxcycles,pwm_commit_events=len(commits),commit_cycle_min=min((x[1]for x in commits),default=None),commit_cycle_max=max((x[1]for x in commits),default=None),events=events,transitions=transitions,windows=windows,fault=o['fault'],stop_zero=stopped['u']==0,ext2_seen=active_seen,final_stage=stage,elapsed_s=time.monotonic()-beg,clock_contract='50000 clocks per 1ms; actual command changes integrated at commit_cycle/50MHz; matrix exponential integration, no PWM carrier model',binding=binding,plant_sha256=sha(modelpath),runner_sha256=sha(Path(__file__)),trace_sha256=sha(O/'trace.npz'))
 assert all(sha(P/n)==h for n,h in binding['source_sha256'].items());(O/'RESULT.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in res.items()if k not in('binding','transitions')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
