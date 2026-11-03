"""Empirical causal replay of the *unchanged* P1 first2s input prefix.
NOT a counterfactual dynamics model. Refuses changed input rather than fabricating an answer.
"""
from pathlib import Path
import json,numpy as np
from analyze_models import load,save,scores
P=Path(__file__).resolve().parent
class NominalPrefix:
 def __init__(self,initial_angles):
  self.q=json.loads((P/'nominal_prefix.json').read_text());self.i=0;self.origin=np.asarray(initial_angles,dtype=float)
 def observe(self):return self.origin+np.array(self.q['relative_angles'][self.i])
 def step(self,logical_pwm_fraction):
  if self.i>=len(self.q['logical_pwm']):raise ValueError('Only first2000ms nominalprefix is supported')
  if abs(float(logical_pwm_fraction)-self.q['logical_pwm'][self.i])>1e-12:raise ValueError('Input differs from validatedprefix; counterfactual dynamics is not identified')
  self.i+=1;return self.observe()
def main():
 data=[d for d in load()if d['name'].startswith('R33P1')];train=[d for d in data if d['partition']=='fit'];U=np.array([d['u'][:2000]for d in data]);assert np.all(U==U[0]);relative=np.array([np.c_[d['theta'][:2001]-d['theta'][0],d['phi'][:2001]-d['phi'][0]]for d in train]);reference=np.mean(relative,axis=0)
 save('nominal_prefix.json',dict(schema='j280.empirical_same_input_prefix.v1',dt_ms=1,horizon_ms=2000,fit_rounds=[d['name']for d in train],relative_angles=reference.tolist(),logical_pwm=U[0].tolist(),public_coordinates='unwrapped measuredtheta andphysical encoderphi radians',status='nominal_replay_only_refuse_changed_input',not_dynamics=True))
 result={}
 for d in data:
  p=NominalPrefix([d['theta'][0],d['phi'][0]]);y=[p.observe()]
  for u in d['u'][:2000]:y.append(p.step(u))
  y=np.array(y);valid=(d['flags'][:2001]&4!=0)&(d['flags'][:2001]&512==0);truth=np.c_[d['theta'][:2001],d['phi'][:2001]];result[d['name']]=dict(partition=d['partition'],input_prefix_exactly_equal=True,metrics=scores(y,truth,valid),endpoint_theta_deg=float(np.degrees(y[-1,0])),actual_endpoint_theta_deg=float(np.degrees(truth[-1,0])),endpoint_phi_deg=float(np.degrees(y[-1,1])),actual_endpoint_phi_deg=float(np.degrees(truth[-1,1])))
 save('nominal_prefix_results.json',result)
 p=NominalPrefix([0,0]);rejected=False
 try:p.step(U[0,0]+.001)
 except ValueError:rejected=True
 assert rejected;print(result)
if __name__=='__main__':main()
