from pathlib import Path
import argparse,json,subprocess,hashlib,sys
D=Path(__file__).resolve().parent;W=D.parents[2]
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);q=ap.parse_args();P=q.project.resolve();O=q.out.resolve();O.mkdir(parents=True,exist_ok=True);H=P/'hybrid';S=P/'fpga/src';sys.path.insert(0,str(H));import native_machine as nm
nm.D=O
for name,file in [('uncompact',H/'hybrid_program_uncompacted.json'),('baseline',D/'fixtures/frozen255_program.json')]:nm.generate(file,name)
a=json.loads((H/'hybrid_program.json').read_text());b=json.loads((H/'hybrid_program_uncompacted.json').read_text());c=json.loads((D/'fixtures/frozen255_program.json').read_text());names=[n for n in a['names']if not n.startswith(('tmp','constant'))and n in b['names']and n in c['names']];(O/'maps.hpp').write_text('static const int MAPS[][3]={'+','.join('{'+','.join(str(d['names'][n])for d in[a,b,c])+'}'for n in names)+'};\n')
s=(D/'fixtures/cpu_verify.cpp').read_text()
(O/'verify.cpp').write_text(s)
cmd=['verilator','--cc','--exe','--build','-j','2','-Wno-fatal','-CFLAGS','-O2 -I'+str(H),'--top-module','s23_compute','--Mdir',str(O/'obj'),*[str(S/n)for n in ['s23_compute.v','s23_memory.v','s23_alu.v','s23_soft_mul24.v']],str(P/'sim/dpb_sim_model.v'),str(O/'verify.cpp')]
with(O/'build.log').open('w')as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
r=json.loads(subprocess.check_output([str(O/'obj/Vs23_compute'),str(D/'fixtures/group3.bin'),str(D/'fixtures/group4.bin')],text=True));r['scope']='Actual microcode CPU RTL vs integer oracle, all writable memory checked, compact/uncompacted named state exact; both directions, valid/gap, first/recovery, coupling boundary and confirmed-HOLD. Not full 1ms controller schedule or physical recovery proof.';r['source_sha256']={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest()for f in[H/'build.py',H/'hybrid_program.json',H/'layout.json',H/'hybrid_fixed.hpp',S/'s23_compute.v',S/'s23_memory.v',S/'s23_alu.v']};(O/'RESULT.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
