from pathlib import Path
import subprocess,json
P=Path(__file__).resolve().parent;B=P/'build'/'abort' ;B.mkdir(exist_ok=True,parents=True)
with (B/'build.log').open('w')as f:subprocess.run(['verilator','--cc','--exe','--build','-j','2','--Mdir',str(B),'--top-module','r01_swing_core',str(P/'r01_swing_core.v'),str(P/'abort_driver.cpp')],check=True,stdout=f,stderr=subprocess.STDOUT)
s=subprocess.check_output([str(B/'Vr01_swing_core')],text=True);print(s);(P/'abort_verification.json').write_text(json.dumps(dict(status='PASS',output=s),indent=2))
