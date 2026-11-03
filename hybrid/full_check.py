from pathlib import Path
import sys,math,json,time,numpy as np,ctypes as C,argparse,os
D=Path(__file__).resolve().parent;P=D.parent;sys.path.insert(0,str(P/'sim'));import rtl_controller
RTL=rtl_controller.RTL
RELEASE_DELAY=int(os.environ.get("SENSOR_EXIT_DELAY_MS","0"));assert 0<=RELEASE_DELAY<=32
PREFIX=("motion" if os.environ.get("MOTION_CHECK") else "full")+(f"_sensor{RELEASE_DELAY}"if RELEASE_DELAY else "")
bank=json.loads((D/'model_bank.json').read_text());models=bank['models'];KEYS=bank['keys'];O=P/'build/reproduce/full';O.mkdir(parents=True,exist_ok=True)
lib=C.CDLL(str(P/'build/reproduce/hybrid'/('hybrid_fixed.dylib' if sys.platform=='darwin' else 'hybrid_fixed.so')));ptr=C.POINTER(C.c_double)
for n in ['mass_step','effective_step']:getattr(lib,n).argtypes=[ptr,C.c_double,ptr,C.c_double,C.c_int]
def one(mi,sign,horizon,event):
 import hashlib
 snapshot={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest()for f in sorted((P/'fpga/src').glob('*'))if f.is_file()}
 r=RTL();motion=bool(os.environ.get("MOTION_CHECK"));r.configure(workspace=190,amplitude=30,speed=2,period=12);shuttle_started=False;motion_commanded=False;return_commanded=False;p=np.array([models[mi]['p'][k]for k in KEYS]);x=np.array([sign*(math.pi-.025),0.,0.,0.]);advance=lib.mass_step if mi>=2 else lib.effective_step
 prior_blind=False;contact_age=999;contact_rail=0
 def sense():
  nonlocal prior_blind,contact_age,contact_rail
  code=(746+math.atan2(math.sin(x[0]),math.cos(x[0]))/(386/65536))%(2*math.pi/(386/65536));blind=code>1023
  if prior_blind and not blind:contact_age=0;contact_rail=1022 if code>512 else 0
  prior_blind=blind
  if blind:code=0
  elif contact_age<RELEASE_DELAY:code=contact_rail
  contact_age+=1;fine=min(16368,max(0,round(code*16)));r.fraction(fine>>4,fine,round(x[2]/(396/65536)),fractional_valid=96<=fine<=16272)
 for k in range(40):sense();r.tick()
 r.begin();old=0;rows=[];capture=-1;st=time.monotonic();laststate=1;captures=[];maxcycles=0;commit_events=[]
 for j in range(horizon):
  if event and capture>=0 and j==capture+6000:x[1]+=event*4.
  if motion and capture>=0 and j==capture+3000:r.command(6,math.pi/2);motion_commanded=True
  if motion and motion_commanded and not return_commanded and j>capture+9000 and o['ready'] and abs(x[2]-math.pi/2)<.08:r.command(6,0);return_commanded=True
  if motion and return_commanded and not shuttle_started and j>capture+16000 and o['ready'] and abs(o['ref'])<.001 and abs(x[2])<.08:r.command(7,1);shuttle_started=True
  sense();o=r.tick();maxcycles=max(maxcycles,o['cycles'])
  if o['state']==2 and laststate in [1,6]:captures.append(j);capture=j if capture<0 else capture
  rows.append([j*.001,*x,o['u'],o['state'],o['fault'],o['ref'],o['limit'],o['cycles'],o['theta'],o['omega'],o['vel']]);laststate=o['state']
  if o['fault']:break
  # Integrate using measured hardware command changes, not computation-done.
  elapsed=0.;applied=old
  for cycle,value in o['commits']:
   when=cycle/50e6;dt=when-elapsed
   if dt>0:advance(x.ctypes.data_as(ptr),applied,p.ctypes.data_as(ptr),dt,max(1,math.ceil(dt/.00025)))
   elapsed=when;applied=value;commit_events.append([j,cycle,value])
  dt=.001-elapsed
  advance(x.ctypes.data_as(ptr),applied,p.ctypes.data_as(ptr),dt,max(1,math.ceil(dt/.00025)));old=applied
  assert old==o['u']
  if os.environ.get('EXTRA_IDLE'):r.idle(int(os.environ['EXTRA_IDLE']))
  if j%2000==0:print(mi,j,o['state'],o['fault'],flush=True)
 r.stop();sense();stopped=r.tick();r.close();arr=np.array(rows);np.savez_compressed(O/f'{PREFIX}_{mi}_{sign}_{event}.npz',data=arr)
 tail=arr[-1000:];res=dict(return_commanded=return_commanded,motion_commanded=motion_commanded,shuttle_started=shuttle_started,model=mi,sign=sign,event=event,captures_ms=captures,fault=o['fault'],state=o['state'],duration_ms=j+1,theta_rms_deg=float(np.sqrt(np.mean(np.arctan2(np.sin(tail[:,1]),np.cos(tail[:,1]))**2))*180/math.pi),phi_rms_deg=float(np.sqrt(np.mean((tail[:,3]-tail[:,8])**2))*180/math.pi),maximum_cycles=maxcycles,stop_zero=stopped['u']==0,seconds=time.monotonic()-st)
 import hashlib
 res['sensor_exit_delay_ms']=RELEASE_DELAY
 res['measured_command_change_cycles']=sorted(set(q[1]for q in commit_events));res['command_changes']=len(commit_events);res['extra_idle_clocks']=int(os.environ.get('EXTRA_IDLE','0'));res['hardware_commit_measured']=True
 assert snapshot=={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest()for f in sorted((P/'fpga/src').glob('*'))if f.is_file()},'RTL changed during run'
 res['rtl_sha256']=snapshot
 (O/f'{PREFIX}_{mi}_{sign}_{event}.json').write_text(json.dumps(res,indent=2));print(res,flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--model',type=int,default=0);a.add_argument('--sign',type=int,default=1);a.add_argument('--event',type=int,default=0);a.add_argument('--ms',type=int,default=15000);o=a.parse_args();one(o.model,o.sign,o.ms,o.event)
