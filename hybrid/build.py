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
startup_kick=p.reg('startup_kick',0);cos_theta=p.reg('cos_theta')
launch_active=p.reg('launch_active',1);launch_reverse=p.reg('launch_reverse');first_apex_boost=p.reg('first_apex_boost')
phase_data=json.loads((D/'phase_reference.json').read_text());phase_commands=p.array('phase_commands',20,[round(u*1000)/1000 for u in phase_data['commands']]);phase_active=p.reg('phase_active',1);phase_clock=p.reg('phase_clock');phase_knot=p.reg('phase_knot');phase_enabled=p.reg('phase_enabled',1);phase_rate=p.reg('phase_rate',1)
observer_ready_input=p.reg('observer_ready_input',1)
r34_real_direction=p.reg('r34_real_direction');r34_turn_event=p.reg('r34_turn_event');r34_reanchor=p.reg('r34_reanchor');r34_anchor=p.reg('r34_anchor')
r34_terminal_override=p.reg('r34_terminal_override')
r34_previous_energy=p.reg('r34_previous_energy');r34_energy_valid=p.reg('r34_energy_valid');r34_brake_authorized=p.reg('r34_brake_authorized')
p.mark('main')
r34_terminal_override.set(0)
r34_turn_event.set(0);r34_reanchor.set(0)
returning=resume_elapsed>0
with p.if_(returning|mode.ne(0)|(ever>0)):launch_active.set(0);first_apex_boost.set(0);phase_active.set(0)
progress_event.set(0)
# Gap discards the unfinished direction crossing, not prior measured turn anchors.
# half_valid remembers the first real turn that closed the initial partial half-swing.
with p.if_(progress_measurement_valid.eq(0)):
 launch_active.set(0);progress_direction.set(0);envelope_latch.set(0);envelope_phase.set(0)
envelope_trigger.set(0);envelope_release.set(0)
# Real-observation re-entry after a bounded zero-output coast. Keep attempt/ever.
with p.if_(returning):
 r34_energy_valid.set(0);r34_brake_authorized.set(0)
 progress_direction.set(0);progress_half_valid.set(0);progress_previous_valid.set(0);progress_eligible.set(0)
 envelope_latch.set(0);envelope_phase.set(0)
 mode.set(0);age.set(0);confirm.set(0);iv.set(0);ih.set(0);vreq.set(v)
 counter.set(0);tick.set(0);attempt.set(attempt+resume_elapsed);restarts.set(restarts+1)
 # A real COAST can interrupt the first plan outside the original pump corridor.
 # Migrate the same existing local corridor to the measured recovery location.
 with p.if_(abs(f)>1.09756):
  recovery_lo.set(f-1.09756);recovery_hi.set(f+1.09756);recovery_zone.set(1)
 # Preserve model prior across sensor coast; predictions reset to current measured state.
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
# Use only a real measured hysteretic velocity reversal. Gap/COAST or leaving
# SWING discards the direction history; predicted observations cannot trigger.
with p.if_(returning|mode.ne(0)|progress_measurement_valid.eq(0)):r34_real_direction.set(0)
with p.if_(mode.eq(0)&progress_measurement_valid&returning.eq(0)):
 r34_turn_event.set(((w>.2)&(r34_real_direction<0))|((w<-.2)&(r34_real_direction>0)))
 with p.if_(w>.2):r34_real_direction.set(1)
 with p.if_(w<-.2):r34_real_direction.set(-1)
energy_target=p.reg('energy_target');entry_limit=p.reg('entry_limit');energy_target.set(1.1);entry_limit.set(60)
# Causal hysteretic turning-point loss correction, only post-capture recovery.
with p.if_(mode.eq(0)&(ever>0)):
 with p.if_(r34_turn_event):
  with p.if_(abs(t)<2.5):recovery_energy.set((recovery_energy+.5*(1-cos_theta)).clip(1,1.4))
with p.if_(w>.2):swing_direction.set(1)
with p.if_(w<-.2):swing_direction.set(-1)
with p.if_(ever>0):energy_target.set(recovery_energy)
with p.if_(attempt>6000):energy_target.set(1.4);entry_limit.set(150)
with p.if_((attempt>6000)&(ever>0)):energy_target.set(recovery_energy.maximum(1.15))

entry_scratch_begin=p.tmp
a=t*(7.11140191/.174532925)+w*(5.19581027/1.3)+v*(5.59886115/1.2);b=w*(.85420562/1.3)+v*(.90446855/1.2);c=v*(.14617569/1.2)
score.set(a*a+b*b+c*c);inside.set((abs(t)<.244346)&(abs(w)<2)&(abs(v)<1)&(score<11.218469));enter.set(inside|((abs(t)<.52)&(abs(w)<6)&((abs(v)<3)|(ever.eq(0)&(abs(v)<4)))&(score<entry_limit)&(t*w<0)))
entry_scratch_end=p.tmp
# R37 offline first-catch prewarm only. Final inside/30ms remains unchanged.
# Use real current samples, measured approach, bounded rod/arm rates.
r41_entry_work_begin=p.tmp
with p.if_((ever.eq(0)|((abs(v)<1.5)&(abs(w)<2.5)))&progress_measurement_valid&(abs(t)<.28)&(abs(w)<3)&(abs(v)<3)&(t*w<0)):
 enter.set(1)
r41_entry_work_end=p.tmp
with p.if_(mode.eq(0)&enter):mode.set(1);age.set(0);origin.set(f);ih.set(0);confirm.set(0);entries.set(entries+1)
with p.if_(mode.ne(0)&((abs(t)>.8)|(abs(w)>6))):
 recovered.set(mode.eq(2));mode.set(0);age.set(0);iv.set(0);vreq.set(v);restarts.set(restarts+1)
 # Each recovery gets a timeout; failed provisional catches cannot reset it.
 with p.if_(recovered):attempt.set(0);recovery_energy.set(1);recovery_zone.set(0);recovery_lo.set(-1.09756);recovery_hi.set(1.09756)
 with p.if_(abs(f)>1.09756):
  recovery_lo.set(f-1.09756);recovery_hi.set(f+1.09756);recovery_zone.set(1)
want.set(0);cap.set(.35)
with p.if_(mode.ne(0)):
 # R21: freeze a local origin only after the consecutive capture confirmation.
 with p.if_(inside):confirm.set(confirm+1)
 with p.if_(inside.eq(0)):confirm.set(0)
 with p.if_(mode.eq(1)&(confirm>=30)&observer_ready_input):mode.set(2);origin.set(f);ih.set(0);age.set(0);attempt.set(0);ever.set(1)
 with p.if_(mode.eq(2)&target_enable):origin.set(target_position)
 er=f-origin;relative_v=p.reg('relative_v');relative_v.set(v)
 with p.if_(mode.eq(2)&target_enable):relative_v.set(v-target_velocity)
 brake_scratch_begin=p.tmp
 raw.set(6.996196749162687*t+.6847930331950819*w+.3162277660168408*er+.48012438370034877*relative_v+ih)
 brake_scratch_end=p.tmp
 brake_tmp_begin=p.tmp
 # R38 recovery only: inbound arm momentum beyond the normal SWING request
 # envelope gets bounded rod-rate damping; first acquisition is bypassed.
 with p.if_(mode.eq(1)&(ever>0)&progress_measurement_valid&(abs(v)>5)&(abs(t)<.28)&(t*w<0)&(v*w<0)):raw.set(raw+.35*w)
 brake_tmp_end=p.tmp
 with p.if_(mode.eq(2)&target_enable):raw.set(raw+.125*target_velocity)
 cap.set(.15)
 with p.if_(mode.eq(1)|(age<500)|((abs(t)>.02618)&(t*w>0))|((abs(t+.1*w)>.04)&(t*w>0))):cap.set(.35)
 with p.if_((age>1000)&(abs(t)<.08)&(abs(w)<1)&(((abs(raw)>cap)&(raw*er>0)).eq(0))):ih.set((ih+.0001*er).clip(-.1,.1))
 want.set(raw.clip(-cap,cap))
# Extended envelope is exclusive to first SWING, never BRAKE/HOLD/recovery.
with p.if_(mode.ne(0)):envelope_latch.set(0);envelope_phase.set(0);progress_direction.set(0);progress_half_valid.set(0);progress_previous_valid.set(0);progress_eligible.set(0)
first_swing=ever.eq(0)
with p.if_(mode.eq(0)):
 # Only completed, observed first-SWING half-cycles authorize yielding/target changes.
 with p.if_(progress_measurement_valid):
  progress_event.set(((w>.2)&(progress_direction<0))|((w<-.2)&(progress_direction>0)))
  with p.if_(progress_event):
   progress_turn_peak.set(cos_theta)
   with p.if_(first_swing&restarts.eq(0)&(progress_turn_peak>.7)&(progress_turn_peak<.9)&(abs(v)<1.5)):first_apex_boost.set(1)
   with p.if_(progress_half_valid):
    with p.if_(progress_previous_valid):
     progress_gain.set(progress_turn_peak-progress_previous_peak)
     with p.if_(progress_gain<.02):progress_eligible.set(1)
     with p.if_(progress_gain>.05):progress_eligible.set(0)
     with p.if_(first_swing&(progress_turn_peak<math.cos(.244346))&(progress_gain<.02)):
      progress_target.set((progress_target+.1).minimum(1.4))
    progress_previous_peak.set(progress_turn_peak);progress_previous_valid.set(1)
   progress_half_valid.set(1)
  with p.if_(w>.2):progress_direction.set(1)
  with p.if_(w<-.2):progress_direction.set(-1)
 with p.if_(first_swing):
  energy_target.set(progress_target)
  with p.if_(first_apex_boost):energy_target.set(energy_target.maximum(1.2))
  with p.if_(attempt>6000):energy_target.set(1.4)
 r43_taper_begin=p.tmp
 with p.if_((ever>0)&progress_measurement_valid&(cos_theta>.5)&(t*w<0)):energy_target.set(energy_target.minimum(1.15+.5*(1-cos_theta)))
 r43_taper_end=p.tmp
 energy.set(.5*w*w/94.50762226949593+cos_theta-energy_target);pump=p.reg('pump')
 with p.if_((ever>0)&(cos_theta>0)):energy.set(energy+.01*v*v)
 # Causal angle-phase lead, 0..20ms, weighted by the two slow actuator hypotheses.
 # Energy and measured velocity remain current; no future observations.
 pump.set(0)
 lead_phase=p.reg('lead_phase');lead_phase.set(w*(t+.02*(weights[1]+weights[3])*w).cos())
 with p.if_(lead_phase>0):pump.set(120*energy)
 with p.if_(lead_phase<0):pump.set(-120*energy)
 # R32: migrated pump corridor and centering feedback share the same origin.
 acc.set(pump-3*(f-.5*(recovery_lo+recovery_hi))-1.78864*v)
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
 with p.if_((first_swing|((ever>0)&(progress_gain<-.05)))&progress_measurement_valid&progress_eligible&envelope_latch.eq(0)&envelope_release.eq(0)&envelope_clipped&(pump*desired<0)):
  envelope_latch.set(1);envelope_trigger.set(1);envelope_phase.set(1)
  with p.if_(envelope_phase_now<0):envelope_phase.set(-1)
 with p.if_(envelope_latch):
  # Initial +/-1.09756 becomes +/-2.2; migrated recovery origins stay anchored.
  vreq.set(envelope_vv.clip(-6.5,6.5));low.set((8*(recovery_lo-1.10244-f)).clip(-6.5,6.5));high.set((8*(recovery_hi+1.10244-f)).clip(-6.5,6.5));vreq.set(vreq.clip(low,high));vreq.set(vreq.clip(v-1.6,v+1.6))
  desired.set((50*(vreq-v)).clip(-40,40))
 # When the old accumulated request still accelerates against newly reversed
 # energy pump, reflect the existing velocity error once around actual arm speed
 # Preserve the exact current normal/extended soft corridor and net acc.
 r34_anchor.set(2*v-vreq)
 with p.if_(r34_turn_event&((progress_event&progress_previous_valid&progress_eligible&(progress_gain<.02))|(ever>0))&(cos_theta>0)&((phase_active&phase_enabled).eq(0))&(energy<0)&(pump*desired<0)&(pump*acc>0)&(r34_anchor>=low)&(r34_anchor<=high)&(r34_anchor>=v-1.6)&(r34_anchor<=v+1.6)):
  vreq.set(r34_anchor);desired.set((50*(vreq-v)).clip(-40,40));r34_reanchor.set(1)
 # R43: release stale velocity error at a measured coupling reversal;
 # retain the existing feasible corridor and bounded request/actuator slew.
 r43_phase_begin=p.tmp
 with p.if_((ever>0)&progress_measurement_valid&(abs(t)>1.2)&(abs(t)<2)&(abs(w)>4)&(energy<0)&(pump*desired<0)&(pump*acc>0)&(v>=low)&(v<=high)):
  vreq.set(.5*(vreq+v));desired.set((50*(vreq-v)).clip(-40,40))
 r43_phase_end=p.tmp
 sumu.set(0)
 sn=t.sin();cs=cos_theta;sc=sn*cs
 with p.loop(mi,0,4):
  # Inverse uses only alpha/kappa/damping/gains/tau_on. The prediction loop
  # still loads all nine parameters and all twelve cached coefficients.
  for k in (0,1,2,4,5,6):params[k].set(p.load(bank[0],mi*9+k))
  p.call('load_inverse_coefficients');p.call('inverse_control');sumu.set(sumu+p.load(weights[0],mi)*inverse.clip(-.7,.7))
 want.set(sumu.clip(-.35,.35))
# Common state-phase reference policy replaces the old one-shot launch.
# Scratch expressions here execute after all legacy computations and before
# ENDFAST, and make no subroutine calls. Overlay only these new temporaries.
phase_tmp_begin=p.tmp
# Only the first post-clear measured transaction may admit the common plan.
# Rejecting the declared natural-down/low-speed neighbourhood is permanent
# until a real reset; ordinary bridge prediction later does not re-admit it.
with p.if_(tick.eq(1)&((abs(t)<math.pi-.12)|(abs(w)>.3)|(abs(f)>.12)|(abs(v)>.2)|progress_measurement_valid.eq(0))):
 phase_enabled.set(0);phase_active.set(0)
with p.if_(mode.ne(0)|(ever>0)|(abs(f)>2.2)|(abs(v)>7)|(abs(w)>30)|(phase_clock>=2000)|(attempt>2200)):phase_active.set(0)
with p.if_(phase_active&phase_enabled):
 with p.if_(phase_clock>=100*(phase_knot+1)):phase_knot.set(phase_knot+1)
 want.set(p.load(phase_commands[0],phase_knot));vreq.set(v)
 # At the existing20ms update boundary, use the completed measured-energy
 # interval only. Valid upper approach without energy loss authorizes the
 # stronger brake once; preserve permission when that brake starts working.
 with p.if_(progress_measurement_valid.eq(0)):
  r34_energy_valid.set(0);r34_brake_authorized.set(0)
 with p.if_(progress_measurement_valid&counter.eq(0)):
  with p.if_(r34_energy_valid&(cos_theta>0)&(energy>0)&(abs(w)>=6)&(t*w<0)&(energy>=r34_previous_energy)):
   r34_brake_authorized.set(1)
  r34_previous_energy.set(energy);r34_energy_valid.set(1)
 with p.if_(r34_brake_authorized&progress_measurement_valid&(cos_theta>0)&(energy>0)&(abs(w)>=6)&(t*w<0)):
  want.set(.173)
  with p.if_(w<0):want.set(-want)
  r34_terminal_override.set(1)
 # R37 keep_energy: after the preserved first1.5s, a real upper approach
 # below its current energy target must not receive a table braking request.
 # Zero is a request only: original20permille/ms slew and35% cap still apply.
 with p.if_((phase_clock>=1500)&progress_measurement_valid&(cos_theta>0)&(t*w<0)&(energy<0)&(want*w>0)):
  want.set(0)
 phase_clock.set(phase_clock+1)
# R36 recovery terminal energy permission, complete measured intervals only.
with p.if_(ever>0):
 with p.if_(mode.ne(0)|progress_measurement_valid.eq(0)):
  r34_energy_valid.set(0);r34_brake_authorized.set(0)
 with p.if_(mode.eq(0)&progress_measurement_valid&counter.eq(0)):
  with p.if_(r34_energy_valid&(cos_theta>.5)&(abs(w)>=4)&(t*w<0)&((.5*w*w/94.50762226949593+cos_theta)>1.1)&(energy>=r34_previous_energy)):
   r34_brake_authorized.set(1)
  r34_previous_energy.set(energy);r34_energy_valid.set(1)
 with p.if_(mode.eq(0)&r34_brake_authorized&progress_measurement_valid&(cos_theta>.5)&(abs(w)>=4)&(t*w<0)&((.5*w*w/94.50762226949593+cos_theta)>1.1)):
  want.set(0.35)
  with p.if_(w<0):want.set(-want)
phase_tmp_end=p.tmp
with p.if_(mode.ne(2)&(attempt>=12000)):fault.set(5)
output.set(want.clip(previous-.02,previous+.02))
with p.if_(abs(output)>cap):cap.set(.35)
with p.if_(fault>0):output.set(0)
p.emit('ENDFAST')
co=emit_library(p,z,params,model,duty,dt)
p.mark('load_coefficients')
for k in range(4,12):co[k].set(p.load(cache[0],mi*12+k))
p.mark('load_inverse_coefficients')
for k in range(4):co[k].set(p.load(cache[0],mi*12+k))
p.emit('RET')
p.mark('inverse_control')
with p.if_(mi<2):inverse.set((v+params[6]*desired)/((params[4]+params[5])*.5))
with p.if_(mi>=2):
 lam,bg,d,k2=co[:4];restoring=params[0]*sn+sc*v*v-params[2]*w
 inverse.set(((d+(1+k2)*sn*sn)*desired+params[1]*cs*restoring+bg*v+2*sc*v*w-params[1]*sn*w*w)/lam)
p.emit('RET')
a=p.finish(D/'hybrid_program_uncompacted.json')
# Only new final-block expression values alias existing earlier expression
# temporaries. Every such temporary is dead at entry and recomputed next frame.
phase_new=[n for n in a['names']if n.startswith('tmp')and phase_tmp_begin<=int(n[3:])<phase_tmp_end]
brake_new=[n for n in a['names']if n.startswith('tmp')and brake_tmp_begin<=int(n[3:])<brake_tmp_end]
brake_scratch=[n for n in a['names']if n.startswith('tmp')and brake_scratch_begin<=int(n[3:])<brake_scratch_end]
brake_scratch += [n for n in a['names']if n.startswith('tmp')and entry_scratch_begin<=int(n[3:])<entry_scratch_end]
# All expression temporaries in the raw polynomial die at raw.set. The
# separate er temporary precedes this range and remains live for the I update.
# Entry score expressions likewise die after score/inside/enter are stored;
# none of their temporary values are used by the later BRAKE feedback block.
brake_shared=brake_new[:len(brake_scratch)]
r41_entry_work=[n for n in a['names']if n.startswith('tmp')and r41_entry_work_begin<=int(n[3:])<r41_entry_work_end]
r41_entry_dead=[n for n in a['names']if n.startswith('tmp')and entry_scratch_begin<=int(n[3:])<entry_scratch_end]
assert len(r41_entry_work)<=len(r41_entry_dead)
r43_taper_work=[n for n in a['names']if n.startswith('tmp')and r43_taper_begin<=int(n[3:])<r43_taper_end]
assert len(r43_taper_work)<=len(r41_entry_dead)
r43_phase_work=[n for n in a['names']if n.startswith('tmp')and r43_phase_begin<=int(n[3:])<r43_phase_end]
assert len(r43_phase_work)<=len(r41_entry_dead)
newtemps=r43_phase_work+phase_new+brake_shared+r41_entry_work+r43_taper_work
oldtemps=[n for n in a['names']if n.startswith('tmp')and int(n[3:])<phase_tmp_begin and n not in newtemps]
assert len(oldtemps)>=len(phase_new)
removed={a['names'][n]for n in newtemps}
alias={a['names'][n]:a['names'][oldtemps[i]]for i,n in enumerate(phase_new)}
alias.update({a['names'][n]:a['names'][brake_scratch[i]]for i,n in enumerate(brake_shared)})
alias.update({a['names'][n]:a['names'][r41_entry_dead[i]]for i,n in enumerate(r41_entry_work)})
alias.update({a['names'][n]:a['names'][r41_entry_dead[i]]for i,n in enumerate(r43_taper_work)})
alias.update({a['names'][n]:a['names'][r41_entry_dead[i]]for i,n in enumerate(r43_phase_work)})
kept=[i for i in range(len(a['initial']))if i not in removed];renumber={old:i for i,old in enumerate(kept)};mapping={i:renumber[alias.get(i,i)]for i in range(len(a['initial']))}
a['initial']=[a['initial'][i]for i in kept]
for code in a['code']:
 if code[0]not in [22,23,24]:code[1]=mapping[code[1]]
 code[2]=mapping[code[2]];code[3]=mapping[code[3]]
a['names']={n:mapping[i]for n,i in a['names'].items()if n not in newtemps}
(D/'hybrid_program.json').write_text(json.dumps(a,separators=(',',':')))
(D/'scratch_overlay.json').write_text(json.dumps(dict(scope='Final no-call block scratch plus recovery BRAKE correction over dead raw polynomial scratch. er remains live and is excluded. Verify uncompacted candidate with identical inputs.',new_temporaries=phase_new,earlier_scratch=oldtemps[:len(phase_new)],brake_new=brake_shared,brake_polynomial_scratch=brake_scratch[:len(brake_shared)],words_saved=len(newtemps)),indent=2))
print('instructions',len(a['code']),'words',len(a['initial']))

