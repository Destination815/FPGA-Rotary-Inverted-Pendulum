from pathlib import Path
import csv,json,time,sys
import numpy as np
from scipy.optimize import least_squares
import model_api as A
P=Path(__file__).resolve().parent;W=A.W;ROOT=W.parent
TRAIN={'R33_B1_1','R33_B1_2','R33_B2_1','R33_B2_2','R33P1_B1_1','R33P1_B1_2'}
def save(name,obj):(P/name).write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n')
def load():
 out=[]
 for group in ['R33测试结果','R33P1测试结果']:
  for d in sorted((ROOT/group).iterdir()):
   if not d.is_dir():continue
   files=list(d.glob('passive/*/*samples_1khz.csv'))
   if len(files)!=1:continue
   f=files[0];rows=list(csv.DictReader(f.open()));compact='command_pwm'in rows[0]
   vals=lambda k:np.array([float(r[k])for r in rows]);info=json.loads((f.parent/'info.json').read_text())
   ms=vals('ms').astype(np.int64);encoder=vals('encoder');zero=info.get('coordinate_descriptor',{}).get('recording_encoder_zero',encoder[0]);phi=(encoder-zero)*97763/16777216
   theta=vals('theta_q16')/65536 if compact else vals('theta');theta=np.unwrap(theta)
   u=vals('command_pwm'if compact else'u')/1000;applied=vals('applied_pwm'if compact else'applied')/1000;flags=vals('flags').astype(int)
   segs=np.split(np.arange(len(ms)),np.flatnonzero(np.diff(ms)!=1)+1)
   out.append(dict(name=d.name,source=str(f),sha256=A.digest(f),ms=ms,theta=theta,phi=phi,u=u,applied=applied,flags=flags,state=vals('state').astype(int),segments=segs,partition='fit'if d.name in TRAIN else'whole_round_holdout'))
 return out
def scores(y,truth,valid):
 e=y-truth;e[:,0]=np.arctan2(np.sin(e[:,0]),np.cos(e[:,0]));e=e[valid]
 return {'theta_rms_deg':float(np.sqrt(np.mean(e[:,0]**2))*180/np.pi),'phi_rms_deg':float(np.sqrt(np.mean(e[:,1]**2))*180/np.pi)}
def head_replay(d,p,input_lag=0,sign=1):
 seg=d['segments'][0];n=min(3000,len(seg)-1);t=d['theta'][:n+1];phi=d['phi'][:n+1];u=d['u'][:n];
 if input_lag:u=np.r_[np.zeros(input_lag),u][:-input_lag]
 x=np.array([t[0]/p[16],0,phi[0]*A.K,0,phi[0]*A.K,0,0]);states=A.rollout(x,u*sign,p);y=states[:,[0,2]];y[:,0]*=p[16];y[:,1]/=A.K
 valid=(d['flags'][:n+1]&4!=0)&(d['flags'][:n+1]&512==0)
 return y,scores(y,np.c_[t,phi],valid)
def windows(data,stage,horizon=200):
 H=[];U0=[];U1=[];Y=[];M=[];Valid=[]
 for d in data:
  for seg in d['segments']:
   if len(seg)<150+horizon+1:continue
   for i in range(int(seg[0])+150,int(seg[-1])-horizon+1,100):
    state=d['state'][i];ang=np.arctan2(np.sin(d['theta'][i]),np.cos(d['theta'][i]));sel=(stage=='startup' and state==1 and i<3000)or(stage=='brake'and(state==6 or (state==1 and abs(ang)<.8)))or(stage=='hold'and state in [2,3]and abs(ang)<.3)
    if not sel:continue
    flags=d['flags'][i-150:i+horizon+1];valid=(flags&4!=0)&(flags&512==0)
    if np.mean(valid)<.95:continue
    h=np.c_[d['theta'][i-150:i+1:5],d['phi'][i-150:i+1:5]]
    y=np.c_[d['theta'][i:i+horizon+1:5],d['phi'][i:i+horizon+1:5]]
    # Rebase rod chart at each window for numerical conditioning, not future state resetting.
    shift=2*np.pi*np.round(h[-1,0]/(2*np.pi));h[:,0]-=shift;y[:,0]-=shift
    H.append(h);U0.append(d['u'][i-150:i]);U1.append(d['u'][i:i+horizon]);Y.append(y);Valid.append(valid[150::5]);M.append(dict(round=d['name'],partition=d['partition'],sample=i,ms=int(d['ms'][i]),state=int(state)))
 return dict(H=np.array(H),U0=np.array(U0),U1=np.array(U1),Y=np.array(Y),valid=np.array(Valid),meta=M)
def summary_windows(pred,d):
 out={}
 for name in sorted(set(m['round']for m in d['meta'])):
  idx=np.array([m['round']==name for m in d['meta']]);v=d['valid'][idx];y=pred[idx];truth=d['Y'][idx];r={}
  for ms in [20,50,100,200]:
   n=ms//5+1;r[str(ms)]=scores(y[:,:n].reshape(-1,2),truth[:,:n].reshape(-1,2),v[:,:n].ravel())
  out[name]=dict(windows=int(idx.sum()),partition=next(m['partition']for m in d['meta']if m['round']==name),metrics=r)
 return out
def main():
 data=load();save('data_sources.json',[{k:d[k]for k in ['name','source','sha256','partition']}for d in data]);base=A.baseline();report={};arrays={}
 for d in data:
  r={};
  for name in ['swing','handoff']:
   y,s=head_replay(d,A.baseline(name));r[name]=s;arrays[d['name']+'_'+name]=y
  r['input_contract']=dict(same_row_command_equals_negative_applied_fraction=float(np.mean(d['u']==-d['applied'])),previous_command_equals_negative_applied_fraction=float(np.mean(d['u'][:-1]==-d['applied'][1:])),segments=[len(s)for s in d['segments']],head_3s_valid_fraction=float(np.mean((d['flags'][:3001]&4!=0)&(d['flags'][:3001]&512==0))))
  report[d['name']]=r
 save('initial_baseline.json',report);np.savez_compressed(P/'head_baseline.npz',**arrays)
 print(json.dumps(report,ensure_ascii=False,indent=2))
 if '--baseline-only'in sys.argv:return
 for stage in ['startup','brake','hold']:
  d=windows(data,stage);np.savez_compressed(P/(stage+'_windows.npz'),**{k:v for k,v in d.items()if k!='meta'});save(stage+'_window_meta.json',d['meta'])
  if not len(d['H']):continue
  p=A.baseline('handoff'if stage!='startup'else'swing');pred=A.forecast(d['H'],d['U0'],d['U1'],p);save(stage+'_baseline.json',summary_windows(pred,d));print(stage,len(d['H']),flush=True)
if __name__=='__main__':main()
