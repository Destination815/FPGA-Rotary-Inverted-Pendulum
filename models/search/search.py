from pathlib import Path
import numpy as np,json,sys,ctypes as ct,concurrent.futures,itertools,time
P=Path(__file__).resolve().parent;W=P.parents[1];FW=W;sys.path.insert(0,str(FW/'models'));import measured_grey as G
G.DT=.001
q0=np.array(json.loads((FW/'models/measured_grey_parameters.json').read_text())['q']);qj=np.array(json.loads((W/'evidence/joint_fit.json').read_text())['q'])
params=json.loads((FW/'observer/parameters.json').read_text());K=np.array(params['K']);L=np.array(params['L']);ptr=ct.POINTER(ct.c_double);pt=lambda x:np.ascontiguousarray(x,dtype=float).ctypes.data_as(ptr)
def set_backend(name):
 global lib
 import subprocess
 source=P/('search_motion'+{'initial':'_initial','sensor':'_sensor','damping':''}[name]+'.cpp');binary=P/(name+('_fast.dylib' if sys.platform=='darwin' else '_fast.so'))
 if not binary.exists() or binary.stat().st_mtime<source.stat().st_mtime:subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC',str(source),'-o',str(binary)],check=True)
 lib=ct.CDLL(str(binary));lib.setup.argtypes=[ptr]*4;lib.run.argtypes=[ct.c_int]+[ptr]*10+[ct.c_double]*2+[ptr]*2;lib.setup(*[pt(v)for v in G.matrices(q0)])
set_backend('initial')
def reference(N,kind):
 t=np.arange(N)*.001;r=np.zeros(N)
 if kind.startswith('point'):
  amp=float(kind[5:]);x=np.clip((t-3)/8,0,1);r=np.radians(amp)*(10*x**3-15*x**4+6*x**5)
 elif kind=='sine15':
  age=np.maximum(0,t-3);r=np.radians(15)*np.sin(age*np.pi/10)*np.clip(age/2,0,1)
 rv=np.gradient(r,.001);ra=np.gradient(rv,.001);return np.ascontiguousarray(np.c_[r,rv,ra])
N=30000;rng=np.random.default_rng(2026100721);noise=np.radians(rng.normal(0,.04,N));scenes=[]
for model,q in [('S16',q0),('S20',qj)]:
 for bias in [-1.2,0,1.2]:
  for profile in ['hold','point30','point-30','sine15']:
   scenes.append(dict(model=model,q=q.tolist(),bias_deg=bias,profile=profile))
prepared=[(s,G.matrices(np.array(s['q'])),reference(N,s['profile']))for s in scenes]
def runparam(row):
 idx,p=row;out=[]
 for s,mats,ref in prepared:
  met=np.zeros(8);n=lib.run(N,*[pt(v)for v in [*mats,np.array(s['q']),K,L,ref,noise,np.array(p)]],np.radians(s['bias_deg']),s.get('kick',0),pt(met),None)
  out.append(dict(**s,completed=n==N,metrics=met.tolist(),strict=bool(n==N and met[4]<np.radians(1.5) and met[5]<np.radians(1))))
 r=dict(index=idx,p=p,complete=sum(x['completed']for x in out),strict=sum(x['strict']for x in out),worst_position_deg=float(max(np.degrees(x['metrics'][5])if x['completed']else 100 for x in out)),rows=out)
 print(idx,p,r['complete'],r['strict'],round(r['worst_position_deg'],2),flush=True);return r
if __name__=='__main__':
 choices=[[0,.35,.1,1,0,0]]
 choices +=[[ki,.35,tau,kp,0,0]for ki in [.05,.1,.2,.4,.8]for tau in [.05,.2]for kp in [1,1.5,2]]
 choices +=[[.2,.35,.1,1,ff,aa]for ff in [-.5,0,.5]for aa in [0,.02,.05]if ff or aa]
 rows=list(enumerate(choices));(P/'search_plan.json').write_text(json.dumps(dict(configurations=rows,scenes=scenes,N=N,seed=2026100721,scope='fast floating observer screening only; actual estimator/RTL validation follows'),indent=2))
 with concurrent.futures.ThreadPoolExecutor(max_workers=3)as ex:res=list(ex.map(runparam,rows))
 (P/'search_results.json').write_text(json.dumps(res,indent=2));best=sorted(res,key=lambda x:(-x['complete'],-x['strict'],x['worst_position_deg']))[:8];(P/'search_best.json').write_text(json.dumps(best,indent=2));print('BEST',[(x['index'],x['p'],x['complete'],x['strict'])for x in best])
