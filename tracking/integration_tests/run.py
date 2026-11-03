from pathlib import Path
import subprocess,json,hashlib
HERE=Path(__file__).resolve().parent;R01=HERE.parents[1];SRC=R01/'fpga/src';BUILD=HERE/'build';BUILD.mkdir(exist_ok=True)
source_before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.glob('*.v')}
results={}
for case in ('supervisor','drive','top'):
 cmd=['verilator','--binary','--timing','--assert','-j','2','-Wno-fatal','--top-module','tb_'+case,'-I'+str(SRC),'--Mdir',str(BUILD/case),str(HERE/f'tb_{case}.sv'),*[str(p) for p in SRC.glob('*.v')],str(R01/'sim/dpb_sim_model.v')]
 for name,argv in [(case+'_build',cmd),(case,[str(BUILD/case/('Vtb_'+case))])]:
  r=subprocess.run(argv,cwd=BUILD,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT);(BUILD/(name+'.log')).write_text(r.stdout)
  if r.returncode:raise RuntimeError(name+'\n'+r.stdout[-3000:])
 results[case]=r.stdout.strip();print(r.stdout,flush=True)
# Decode actual top UART bytes: frame CRC and board identity, active output bounds.
import binascii,struct
wire=(BUILD/'top_telemetry.bin').read_bytes();telemetry=capabilities=0;boost_frames=spent_frames=0
import sys
sys.path.insert(0,str(R01/"host"))
from protocol import decode_d02
passive=[];checkpoints=[]
for offset in range(0,len(wire)-63,64):
 frame=wire[offset:offset+64]
 assert frame[:2]==b'\xa5\x5a' and frame[3]==64
 assert binascii.crc_hqx(frame[2:62],0xffff)==int.from_bytes(frame[62:64],'little')
 if frame[2]==2:
  capabilities+=1;assert struct.unpack_from('<I',frame,4)[0]==0x4a070228
 elif frame[2]==4:
  c=decode_d02(frame);assert c['build']==0x4a070228 and abs(c['integral'])<=5872026 and c['age']<=1000 and c['samples']<=200;checkpoints.append(c)
 elif frame[2]==3:
  item=decode_d02(frame);assert item['capture_id']&0x80000000;assert 'filtered_theta' in item['row'];passive.append(item)
 elif frame[2]==1:
  telemetry+=1
  command=struct.unpack_from('<i',frame,32)[0];limit=struct.unpack_from('<H',frame,56)[0]
  assert 150<=limit<=350
  # Fault injection intentionally puts bad command on telemetry while gate disables drive.
  flags=struct.unpack_from('<H',frame,42)[0]
  if flags&16:assert abs(command)<=limit
  if flags&1024:boost_frames+=1;assert flags&2048 and 150<=limit<=350
  if flags&2048 and not flags&1024 and limit==150:spent_frames+=1
assert telemetry>=10 and capabilities>=1
assert boost_frames>=2 and spent_frames>=1,(boost_frames,spent_frames)
assert len(checkpoints)>=5 and any(c['ready']for c in checkpoints)
results['observer_UART']=dict(count=len(checkpoints),CRC='PASS',post_commit_state=True)
results['recovery_UART']=dict(boost_frames=boost_frames,spent_base_frames=spent_frames)
assert len(passive)>=5
assert [r['index'] for r in passive]==list(range(len(passive)))
assert all(b['row']['ms']-a['row']['ms']==1 for a,b in zip(passive,passive[1:]))
results['passive_actual_UART']=dict(samples=len(passive),CRC='PASS',sequence='PASS',interval_ms=1,filtered_q18_extension='PASS')
results['wire_protocol']=dict(telemetry_frames=telemetry,capability_frames=capabilities,CRC='PASS',build_id='4A070228')
source_after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in SRC.glob('*.v')}
assert source_after==source_before,'Sources changed during test; rerun frozen version'
report=dict(status='PASS',results=results,source_sha256=source_after)
(HERE/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
