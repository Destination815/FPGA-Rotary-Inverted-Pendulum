from pathlib import Path
import sys,json,subprocess,hashlib,argparse,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[2];P=W/'experiments/r41t2_sine_ui_20261029';ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args();H=args.project.resolve()/'hybrid' if args.project else D/'jointmid/hybrid';O=args.out.resolve() if args.out else D/'contract';O.mkdir(parents=True,exist_ok=True);sys.path.insert(0,str(H));import native_machine as nm
nm.D=O
for name,file in [('selected',H/'hybrid_program.json'),('uncompacted',H/'hybrid_program_uncompacted.json'),('baseline',D/'fixtures/frozen255_program.json')]:nm.generate(file,name)
a=json.loads((H/'hybrid_program.json').read_text());b=json.loads((H/'hybrid_program_uncompacted.json').read_text());c=json.loads((D/'fixtures/frozen255_program.json').read_text());names=[n for n in a['names']if not n.startswith(('tmp','constant'))]
pairs=','.join('{'+str(a['names'][n])+','+str(b['names'][n])+'}'for n in names);basepairs=','.join('{'+str(a['names'][n])+','+str(c['names'][n])+'}'for n in names if n in c['names'])
s=(D/'fixtures/compact_verify.cpp').read_text()
# Index pairs are generated from the current source, not the archived candidate.
import re
s=re.sub(r'int pairs\[\]\[2\]=\{[^;]+;', 'int pairs[][2]={'+pairs+'};',s)
s=re.sub(r'int basepairs\[\]\[2\]=\{[^;]+;', 'int basepairs[][2]={'+basepairs+'};',s)
(O/'contract.cpp').write_text(s);files=[]
for g in range(1,5):files.append(D/f'fixtures/group{g}.bin')
subprocess.run(['c++','-O2','-std=c++17',str(O/'contract.cpp'),'-o',str(O/'contract')],check=True)
r=json.loads(subprocess.check_output([str(O/'contract'),*map(str,files)],text=True));r['source_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest()for f in[H/'build.py',H/'hybrid_program.json',H/'hybrid_program_uncompacted.json',Path(__file__)]};r['scope']='Exact integer named-state comparison of compact vs uncompacted CPU, five inherited model priors on real 1kHz inputs. Separate first-acquisition and confirmed-HOLD branch equality to255. Not actual FPGA scheduling or physical improvement.';(O/'RESULT.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
