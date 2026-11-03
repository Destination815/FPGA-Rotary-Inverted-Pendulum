from machine import Program
from plant_program import emit_library
from pathlib import Path
import math,json,sys
D=Path(__file__).resolve().parent;bank_data=json.loads((D/'model_bank.json').read_text());models=bank_data['models'];KEYS=bank_data['keys']
p=Program();x=p.array('measurement',4);previous=p.reg('applied');sign=p.reg('kick_sign',1);t,w,f,v=x
vreq=p.reg('vreq');iv=p.reg('iv');ih=p.reg('ih');origin=p.reg('origin');mode=p.reg('mode');age=p.reg('age');tick=p.reg('tick');confirm=p.reg('confirm');attempt=p.reg('attempt');ever=p.reg('ever');fault=p.reg('fault');entries=p.reg('entries');restarts=p.reg('restarts');output=p.reg('output');cap=p.reg('cap',.35)
bank=p.array('bank',36,[m['p'][k]for m in models for k in KEYS]);weights=p.array('weights',4,[.25]*4);pred=p.array('pred',16);coeffs=[]
for m in models:
 q=m['p'];a=q['alpha'];lam=a*.647/(.06*9.81*.075);bg=lam/((q['gain_pos']+q['gain_neg'])*.5)
 coeffs += [lam,bg,q['tau_on']*bg,q['kappa']**2,bg*q['gain_pos'],bg*q['gain_neg'],1/q['tau_on'],1/q['tau_off'],math.exp(-.001/q['tau_on']),math.exp(-.001/q['tau_off']),math.exp(-.0005/q['tau_on']),math.exp(-.0005/q['tau_off'])]
cache=p.array('coefficients',48,coeffs);mi=p.reg('mi');z=p.array('z',4);params=p.array('parameters',9);model=p.reg('model');duty=p.reg('physics_duty');dt=p.reg('dt',.001);total=p.reg('total');err=p.reg('error');counter=p.reg('counter');score=p.reg('score');inside=p.reg('inside');enter=p.reg('enter');pump_anchor=p.reg('pump_anchor');want=p.reg('want');inverse=p.reg('inverse');sumu=p.reg('sumu');desired=p.reg('desired');mixweight=p.reg('weight');raw=p.reg('raw');energy=p.reg('energy');acc=p.reg('acc');low=p.reg('low');high=p.reg('high');recovered=p.reg('recovered');
target_enable=p.reg('target_enable');target_position=p.reg('target_position');target_velocity=p.reg('target_velocity');
resume_elapsed=p.reg('resume_elapsed');recovery_energy=p.reg('recovery_energy',1);swing_direction=p.reg('swing_direction');recovery_zone=p.reg('recovery_zone');recovery_lo=p.reg('recovery_lo',-1.09756);recovery_hi=p.reg('recovery_hi',1.09756)
progress_measurement_valid=p.reg('progress_measurement_valid',1)
envelope_latch=p.reg('envelope_latch');envelope_phase=p.reg('envelope_phase');envelope_vv=p.reg('envelope_vv');envelope_original_vreq=p.reg('envelope_original_vreq');envelope_original_desired=p.reg('envelope_original_desired');envelope_clipped=p.reg('envelope_clipped');envelope_trigger=p.reg('envelope_trigger');envelope_release=p.reg('envelope_release');envelope_phase_now=p.reg('envelope_phase_now')
progress_direction=p.reg('progress_direction');progress_turn_peak=p.reg('progress_turn_peak',-1);progress_previous_peak=p.reg('progress_previous_peak',-1);progress_half_valid=p.reg('progress_half_valid');progress_previous_valid=p.reg('progress_previous_valid');progress_eligible=p.reg('progress_eligible');progress_target=p.reg('progress_target',1.1);progress_gain=p.reg('progress_gain');progress_event=p.reg('progress_event')
cos_theta=p.reg('cos_theta')
p.mark('main')
progress_event.set(0)
# Gap discards the unfinished direction crossing, not prior measured turn anchors.
# half_valid remembers the first real turn that closed the initial partial half-swing.
with p.if_(progress_measurement_valid.eq(0)):
 progress_direction.set(0);envelope_latch.set(0);envelope_phase.set(0)
envelope_trigger.set(0);envelope_release.set(0)
# Real-observation re-entry after a bounded zero-output coast. Keep attempt/ever.
with p.if_(resume_elapsed>0):
 progress_direction.set(0);progress_half_valid.set(0);progress_previous_valid.set(0);progress_eligible.set(0)
 envelope_latch.set(0);envelope_phase.set(0)
 mode.set(0);age.set(0);confirm.set(0);iv.set(0);ih.set(0);vreq.set(v)
 counter.set(0);tick.set(0);attempt.set(attempt+resume_elapsed);restarts.set(restarts+1)
 for j in range(4):weights[j].set(.25)
 resume_elapsed.set(0)
tick.set(tick+1);age.set(age+1);attempt.set(attempt+1)
# Hard domain validity is also enforced by caller. No truth or disturbance access.
with p.if_((abs(t)>math.pi+.02)|(abs(w)>40)|(abs(v)>20)):fault.set(2)
with p.if_(counter.eq(0)):
 total.set(0)
 with p.loop(mi,0,4):
  with p.if_(tick>1):
   e0=p.load(pred[0],mi*4)-t
   # Wrap prediction error across the ADC angle seam.
   er=p.reg('angle_error');er.set(e0)
   with p.if_(er>math.pi):er.set(er-2*math.pi)
   with p.if_(er< -math.pi):er.set(er+2*math.pi)
   e1=(p.load(pred[0],mi*4+1)-w)/.3;e2=(p.load(pred[0],mi*4+2)-f)/.015;e3=(p.load(pred[0],mi*4+3)-v)/.15
   err.set((er/.025)*(er/.025)+e1*e1+e2*e2+e3*e3)
   mixweight.set((p.load(weights[0],mi)*(-err.minimum(20)*.2).exp()).maximum(.0001));p.store(weights[0],mi,mixweight);total.set(total+mixweight)
 with p.loop(mi,0,4):
  with p.if_(total>0):p.store(weights[0],mi,p.load(weights[0],mi)/total)
  for k in range(4):p.store(pred[0],mi*4+k,x[k])
with p.loop(mi,0,4):
 for k in range(4):z[k].set(p.load(pred[0],mi*4+k))
 for k in range(9):params[k].set(p.load(bank[0],mi*9+k))
 model.set(mi>=2);duty.set(previous);p.call('load_coefficients');p.call('prediction_step')
 for k in range(4):p.store(pred[0],mi*4+k,z[k])
counter.set(counter+1)
with p.if_(counter>=20):counter.set(0)
cos_theta.set(t.cos())
energy_target=p.reg('energy_target');entry_limit=p.reg('entry_limit');energy_target.set(1.1);entry_limit.set(60)
# Causal hysteretic turning-point loss correction, only post-capture recovery.
with p.if_(mode.eq(0)&(ever>0)):
 with p.if_(((w>.2)&(swing_direction<0))|((w<-.2)&(swing_direction>0))):
  with p.if_(abs(t)<2.5):recovery_energy.set((recovery_energy+.5*(1-cos_theta)).clip(1,1.4))
with p.if_(w>.2):swing_direction.set(1)
with p.if_(w<-.2):swing_direction.set(-1)
with p.if_(ever>0):energy_target.set(recovery_energy)
with p.if_(attempt>6000):energy_target.set(1.4);entry_limit.set(150)
with p.if_((attempt>6000)&(ever>0)):energy_target.set(recovery_energy.maximum(1.15))

a=t*(7.11140191/.174532925)+w*(5.19581027/1.3)+v*(5.59886115/1.2);b=w*(.85420562/1.3)+v*(.90446855/1.2);c=v*(.14617569/1.2)
score.set(a*a+b*b+c*c);inside.set((abs(t)<.244346)&(abs(w)<2)&(abs(v)<1)&(score<11.218469));enter.set(inside|((abs(t)<.52)&(abs(w)<6)&(abs(v)<3)&(score<entry_limit)&(t*w<0)))
with p.if_(mode.eq(0)&enter):mode.set(1);age.set(0);origin.set(f);ih.set(0);confirm.set(0);entries.set(entries+1)
with p.if_(mode.ne(0)&((abs(t)>.8)|(abs(w)>6))):
 recovered.set(mode.eq(2));mode.set(0);age.set(0);iv.set(0);vreq.set(v);restarts.set(restarts+1)
 # Each recovery gets a timeout; failed provisional catches cannot reset it.
 with p.if_(recovered):attempt.set(0);recovery_energy.set(1);recovery_zone.set(0);recovery_lo.set(-1.09756);recovery_hi.set(1.09756)
 with p.if_((ever>0)&(recovery_zone.eq(0))&(abs(f)>1.09756)):
  recovery_lo.set((origin-1.09756).minimum(f));recovery_hi.set((origin+1.09756).maximum(f));recovery_zone.set(1)
want.set(0);cap.set(.35)
with p.if_(mode.ne(0)):
 # R21: freeze a local origin only after the consecutive capture confirmation.
 with p.if_(inside):confirm.set(confirm+1)
 with p.if_(inside.eq(0)):confirm.set(0)
 with p.if_(mode.eq(1)&(confirm>=30)):mode.set(2);origin.set(f);ih.set(0);age.set(0);attempt.set(0);ever.set(1)
 with p.if_(mode.eq(2)&target_enable):origin.set(target_position)
 er=f-origin;relative_v=p.reg('relative_v');relative_v.set(v)
 with p.if_(mode.eq(2)&target_enable):relative_v.set(v-target_velocity)
 raw.set(6.996196749162687*t+.6847930331950819*w+.3162277660168408*er+.48012438370034877*relative_v+ih)
 with p.if_(mode.eq(2)&target_enable):raw.set(raw+.125*target_velocity)
 cap.set(.15)
 with p.if_(mode.eq(1)|(age<500)|((abs(t)>.02618)&(t*w>0))|((abs(t+.1*w)>.04)&(t*w>0))):cap.set(.35)
 with p.if_((age>1000)&(abs(t)<.08)&(abs(w)<1)&(((abs(raw)>cap)&(raw*er>0)).eq(0))):ih.set((ih+.0001*er).clip(-.1,.1))
 want.set(raw.clip(-cap,cap))
# Extended envelope is exclusive to first SWING, never BRAKE/HOLD/recovery.
with p.if_(mode.ne(0)):envelope_latch.set(0);envelope_phase.set(0);progress_direction.set(0);progress_half_valid.set(0);progress_previous_valid.set(0);progress_eligible.set(0)
with p.if_(mode.eq(0)):
 # Only completed, observed first-SWING half-cycles authorize yielding/target changes.
 with p.if_(ever.eq(0)&progress_measurement_valid):
  progress_event.set(((w>.2)&(progress_direction<0))|((w<-.2)&(progress_direction>0)))
  with p.if_(progress_event):
   progress_turn_peak.set(cos_theta)
   with p.if_(progress_half_valid):
    with p.if_(progress_previous_valid):
     progress_gain.set(progress_turn_peak-progress_previous_peak)
     with p.if_(progress_gain<-.05):progress_eligible.set(1)
     with p.if_(progress_gain>.05):progress_eligible.set(0)
     with p.if_((progress_turn_peak>.5)&(progress_turn_peak<math.cos(.244346))&(progress_gain<.02)):
      progress_target.set((progress_target+.1).minimum(1.4))
    progress_previous_peak.set(progress_turn_peak);progress_previous_valid.set(1)
   progress_half_valid.set(1)
  with p.if_(w>.2):progress_direction.set(1)
  with p.if_(w<-.2):progress_direction.set(-1)
 with p.if_(ever.eq(0)):
  energy_target.set(progress_target)
  with p.if_(attempt>6000):energy_target.set(1.4)
 energy.set(.5*w*w/94.50762226949593+cos_theta-energy_target);pump=p.reg('pump');pump.set(0)
 with p.if_(w*cos_theta>0):pump.set(120*energy)
 with p.if_(w*cos_theta<0):pump.set(-120*energy)
 acc.set(pump-3*f-1.78864*v)
 with p.if_(age<150):acc.set(acc+2*sign)
 # Preserve the same unbounded velocity update before the original envelope.
 acc.set(acc.clip(-52.037,52.037));envelope_vv.set(vreq+.001*acc)
 vreq.set(envelope_vv.clip(-4.55807,4.55807));low.set((8*(recovery_lo-f)).clip(-4.55807,4.55807));high.set((8*(recovery_hi-f)).clip(-4.55807,4.55807));vreq.set(vreq.clip(low,high))
 envelope_original_vreq.set(vreq);envelope_clipped.set(vreq.ne(envelope_vv))
 vreq.set(vreq.clip(v-1.6,v+1.6));desired.set((50*(vreq-v)).clip(-40,40));envelope_original_desired.set(desired)
 # Release at coupling-phase zero/crossing, never on a timer or plant truth.
 envelope_phase_now.set(w*cos_theta)
 with p.if_(envelope_latch&(envelope_phase_now*envelope_phase<=0)):
  envelope_latch.set(0);envelope_phase.set(0);envelope_release.set(1)
 # Do not re-trigger during the same transaction as a phase release.
 with p.if_(ever.eq(0)&progress_measurement_valid&progress_eligible&envelope_latch.eq(0)&envelope_release.eq(0)&envelope_clipped&(pump*desired<0)):
  envelope_latch.set(1);envelope_trigger.set(1);envelope_phase.set(1)
  with p.if_(envelope_phase_now<0):envelope_phase.set(-1)
 with p.if_(envelope_latch):
  # Initial +/-1.09756 becomes +/-2.2; migrated recovery origins stay anchored.
  vreq.set(envelope_vv.clip(-6.5,6.5));low.set((8*(recovery_lo-1.10244-f)).clip(-6.5,6.5));high.set((8*(recovery_hi+1.10244-f)).clip(-6.5,6.5));vreq.set(vreq.clip(low,high));vreq.set(vreq.clip(v-1.6,v+1.6))
  desired.set((50*(vreq-v)).clip(-40,40))
 sumu.set(0)
 sn=t.sin();cs=cos_theta;sc=sn*cs
 with p.loop(mi,0,4):
  for k in range(9):params[k].set(p.load(bank[0],mi*9+k))
  p.call('load_coefficients');p.call('inverse_control');sumu.set(sumu+p.load(weights[0],mi)*inverse.clip(-.7,.7))
 want.set(sumu.clip(-.35,.35))
with p.if_(mode.ne(2)&(attempt>=12000)):fault.set(5)
output.set(want.clip(previous-.02,previous+.02))
with p.if_(abs(output)>cap):cap.set(.35)
with p.if_(fault>0):output.set(0)
p.emit('ENDFAST')
co=emit_library(p,z,params,model,duty,dt)
p.mark('load_coefficients')
for k,c in enumerate(co):c.set(p.load(cache[0],mi*12+k))
p.emit('RET')
p.mark('inverse_control')
with p.if_(mi<2):inverse.set((v+params[6]*desired)/((params[4]+params[5])*.5))
with p.if_(mi>=2):
 lam,bg,d,k2=co[:4];restoring=params[0]*sn+sc*v*v-params[2]*w
 inverse.set(((d+(1+k2)*sn*sn)*desired+params[1]*cs*restoring+bg*v+2*sc*v*w-params[1]*sn*w*w)/lam)
p.emit('RET')
a=p.finish(D/'hybrid_program.json');print('instructions',len(a['code']),'words',len(a['initial']))
