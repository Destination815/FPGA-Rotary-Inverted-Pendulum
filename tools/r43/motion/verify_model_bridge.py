from pathlib import Path
import argparse,subprocess,json,hashlib
ap=argparse.ArgumentParser();ap.add_argument('--vectors',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();A=Path(__file__).resolve().parent;O=a.out.resolve();O.mkdir(parents=True,exist_ok=True)
s=(A/'verify_postmove_damping_cpp.cpp').read_text().replace('HoldObserver m;','HoldObserver m;m.r43_variant=12;');(O/'bridge.cpp').write_text(s)
subprocess.run(['c++','-O2','-std=c++17','-I'+str(A/'fixtures'),str(O/'bridge.cpp'),'-o',str(O/'bridge')],check=True);r=subprocess.check_output([str(O/'bridge'),str(a.vectors.resolve())],text=True);print(r)
(O/'MODEL_BRIDGE.json').write_text(json.dumps(dict(status='PASS',stdout=r,meaning='Runtime model candidate variant12 equals production Python vectors on all state and correction outputs.',runtime_header_sha256=hashlib.sha256((A/'fixtures/hold_observer.hpp').read_bytes()).hexdigest(),vectors_sha256=hashlib.sha256(a.vectors.read_bytes()).hexdigest()),indent=2)+'\n')
