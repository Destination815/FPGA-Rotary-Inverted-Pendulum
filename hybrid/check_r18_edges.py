from pathlib import Path
import ctypes as C,csv,json,numpy as np,sys
P=Path(__file__).resolve().parents[1];A=P/'build/reproduce';R=P/'evidence/r18_inputs';lib=C.CDLL(str(P/'build/reproduce'/('measured_replay.dylib' if sys.platform=='darwin' else 'measured_replay.so')));lib.make.argtypes=[C.c_int,C.c_int];lib.make.restype=C.c_void_p;lib.step.argtypes=[C.c_void_p,*([C.c_int]*4),C.POINTER(C.c_int)];lib.release.argtypes=[C.c_void_p]
result=[]
for label in ['B1','B2']:
 f=R/(label+'.csv');rows=list(csv.DictReader(f.open()));info=json.loads((R/(label+'_info.json')).read_text())
 for case in ['rail0','rail1022','disallow','return0','return2','return4']:
  h=lib.make(info['zero_q4'],int(rows[0]['encoder']));out=(C.c_int*10)()
  for _ in range(40):lib.step(h,int(rows[0]['adc_q4'])>>4,int(rows[0]['adc_q4']),int(rows[0]['encoder']),0,out)
  for i,r in enumerate(rows):
   lib.step(h,int(r['adc_q4'])>>4,int(r['adc_q4']),int(r['encoder']),1,out)
   if i>100:assert out[4],(label,case,i)
  first=None;exit_ms=None
  for j in range(200):
   if case.startswith('rail'):adc=int(case[4:]);gap=1
   elif case=='disallow':adc=0;gap=0
   else:
    delay=int(case[6:]);gap=1
    adc=(1022 if label=='B1'else 0)if j<delay else (1018-(j-delay)*2 if label=='B1'else 6+(j-delay)*2)
   lib.step(h,adc,adc*16,int(rows[-1]['encoder']),gap,out)
   if not out[4] and first is None:first=j+1
   if out[4]and not out[5]and exit_ms is None:exit_ms=j+1
  lib.release(h)
  if case.startswith('rail')or case=='disallow':assert first is not None and first<=181,(label,case,first)
  else:assert exit_ms is not None and first is None,(label,case,first,exit_ms)
  result.append(dict(record=label,case=case,first_invalid_ms=first,reacquired_ms=exit_ms))
(A/'edge_checks.json').write_text(json.dumps(result,indent=2));print('PASS',len(result),'record-prefix / synthetic continuations; NOT physical post-fault behavior',result)
