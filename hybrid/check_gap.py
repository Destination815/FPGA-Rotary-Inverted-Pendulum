from pathlib import Path
import sys,math,subprocess,json
P=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(P/'swing/gap_observer'));from gap_fixed import GapFixed
B=P/'build/gap_equivalence';B.mkdir(exist_ok=True);g=GapFixed();rows=[];wrap=lambda x:(x+math.pi)%(2*math.pi)-math.pi;previous=0
for j in range(2400):
 reset=int(j%400==0)
 if reset:g=GapFixed()
 angle=wrap(1.8+.013*j);vel=.013;arm=round(math.sin(j*.05)*65536);adcphase=(angle+746*386/65536)%(2*math.pi);adc=min(1023,max(0,round(adcphase/(386/65536))))if adcphase<=1023*386/65536 else 0
 if j%400>330:adc=0
 pos=round(angle*2**24);v=round(vel*2**24);raw=round(wrap((adc-746)*386/65536)*65536);allow=int(j%181!=180);e=g.update(pos,v,arm,previous,adc,11936,raw,True,allow);previous=arm
 rows.append([reset,pos,v,arm,previous if False else round(math.sin((j-1)*.05)*65536)if j else 0,adc,11936,raw,1,allow,int(e['needgap']),int(e['age']!=0),int(e['ok']),e['pout'],e['vout'],e['age']])
# Inputs passed to both models are identical; reset affects bridge state only.
v=B/'vectors.txt';v.write_text('\n'.join(' '.join(map(str,r))for r in rows)+'\n')
with (B/'build.log').open('w')as f:subprocess.run(['verilator','--cc','--exe','--build','-j','2','-Wno-fatal','--top-module','r01_gap_bridge','--Mdir',str(B/'obj'),str(P/'fpga/src/r01_gap_bridge.v'),str(P/'hybrid/gap_test.cpp')],stdout=f,stderr=subprocess.STDOUT,check=True)
r=subprocess.run([str(B/'obj/Vr01_gap_bridge')],input=v.read_text(),text=True,capture_output=True,check=True);(B/'result.txt').write_text(r.stdout);print(r.stdout)
