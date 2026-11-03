"""Actual top command admission at sine fade exit; sensor/state injection only."""
from pathlib import Path
import argparse,hashlib,json,subprocess
p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();P=a.project.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
base=P/'sim/test_r41t2_top.sv';s=base.read_text().replace('integer capture_pulses=0,start_pulses=0;','integer capture_pulses=0,start_pulses=0,target_pulses=0,saved_target_pulses;').replace('if(dut.start)start_pulses++;','if(dut.start)start_pulses++;\n  if(dut.target_request)target_pulses++;')
needle="cmd('h38,0,0);need(dut.ext2_prepared&&!dut.shuttle,\"exit removed profile before fade done\");same_config();"
insert=needle+'''
  // Direct controller injection at this point would reject mode!=0 with fault9.
  // Actual opcode26 must return busy without any request pulse into controller.
  saved_target_pulses=target_pulses;
  cmd('h26,0,2);
  need(target_pulses==saved_target_pulses,"exit-pending target leaked into controller");
  need(dut.fault==0 && dut.ext2_prepared,"exit-pending target caused fault/profile change");
  force dut.trajectory_running=0;force dut.reference_ready=0;clocks(3);
  cmd('h26,0,2);
  need(target_pulses==saved_target_pulses,"not-ready fade boundary leaked target");
  need(dut.fault==0 && dut.ext2_prepared,"not-ready fade boundary caused fault/profile change");
'''
assert s.count(needle)==1;s=s.replace(needle,insert)
needle2="cmd('h26,0,0);same_config();"
assert s.count(needle2)==1;s=s.replace(needle2,needle2+'\n  need(target_pulses==saved_target_pulses+1,"ready fade boundary lost/duplicated legal target");\n  need(dut.fault==0,"legal post-fade target caused fault");')
bench=O/'test_r41t2_top.sv';bench.write_text(s)
sources=sorted((P/'fpga/src').glob('*.v'))+[P/'sim/dpb_sim_model.v'];bound=sources+sorted((P/'fpga/src').glob('*.vh'))+[base];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();hashes={str(f.relative_to(P)):sha(f)for f in bound}
cmd=['verilator','--binary','--timing','--assert','-Wno-fatal','-j','2','--top-module','test_r41t2_top','-I'+str(P/'fpga/src'),'--Mdir',str(O/'obj'),*map(str,sources),str(bench),'-o','check']
with(O/'build.log').open('w')as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
r=subprocess.run([str(O/'obj/check')],capture_output=True,text=True,cwd=O);(O/'run.log').write_text(r.stdout+r.stderr)
changed=[n for n,h in hashes.items()if sha(P/n)!=h];result=dict(status='PASS'if not r.returncode and not changed else'FAIL',source_sha256=hashes,generated_bench_sha256=sha(bench),changed=changed,output=r.stdout+r.stderr,scope='Actual j280_top opcode26 admission, ACK busy2 and no target_request/fault during sine fade; exact legal target accepted once after fade-ready. Existing 88 top checks retained. Test forces physical/state inputs; not a plant run.')
(O/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(result['status'],result['output']);assert result['status']=='PASS'
