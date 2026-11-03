from pathlib import Path
import ctypes as C,csv,json,subprocess,sys,numpy as np
P=Path(__file__).resolve().parent.parent;D=P/'evidence/r16r3_inputs';O=P/'build/reproduce';E=O/'estimator';V=Path(subprocess.check_output(['verilator','--getenv','VERILATOR_ROOT'],text=True).strip());ext='.dylib'if sys.platform=='darwin'else'.so';L=O/('measured_replay'+ext)
subprocess.run(['c++','-shared','-fPIC','-O2','-std=c++17','-I'+str(E),'-I'+str(V/'include'),'-I'+str(V/'include/vltstd'),str(P/'hybrid/estimator_replay.cpp'),str(E/'Vstate_estimator__ALL.a'),str(V/'include/verilated.cpp'),str(V/'include/verilated_threads.cpp'),'-o',str(L)],check=True)
lib=C.CDLL(str(L));lib.make.argtypes=[C.c_int,C.c_int];lib.make.restype=C.c_void_p;lib.step.argtypes=[C.c_void_p,*([C.c_int]*4),C.POINTER(C.c_int)];lib.release.argtypes=[C.c_void_p];results=[]
for label in ['B1','B2']:
 rows=list(csv.DictReader((D/(label+'.csv')).open()));info=json.loads((D/(label+'_info.json')).read_text());h=lib.make(info['zero_q4'],int(rows[0]['encoder']));out=(C.c_int*10)()
 for k in range(40):lib.step(h,int(rows[0]['adc_q4'])>>4,int(rows[0]['adc_q4']),int(rows[0]['encoder']),0,out)
 bad=[];delta=[]
 for i,r in enumerate(rows):
  lib.step(h,int(r['adc_q4'])>>4,int(r['adc_q4']),int(r['encoder']),1,out)
  if i>100 and not out[4]:bad.append(i)
  if i>100:delta.append(abs(out[0]/65536-float(r['theta'])))
 assert not bad,(label,bad)
 # Same retained endpoint state, then a synthetic stuck rail. This verifies refusal,
 # NOT a claim about what the real pendulum did after the recorded fault.
 first_bad=None
 for j in range(200):
  lib.step(h,0,0,int(rows[-1]['encoder']),1,out)
  if not out[4] and first_bad is None:first_bad=j+1
 assert first_bad is not None and first_bad<=181
 results.append(dict(label=label,samples=len(rows),new_invalid_indices=bad,median_theta_difference_rad=float(np.median(delta)),synthetic_stuck_rail_refused_after_ms=first_bad,scope='Coarse ADC reconstructed as Q4>>4; original 16-conversion raw ADC and exact fractional-valid bit absent. Median equality is not proof of every input bit. Recorded horizon only, not successful swing.'))
 lib.release(h)
(O/'measured_replay.json').write_text(json.dumps(results,indent=2));print('PASS measured boundary regression + stuck-rail refusal',results)
