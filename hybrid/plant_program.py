"""Fixed Q25 prediction library compiled into shared scheduled operations.
Normalized mass matrix avoids poorly scaled physical-mass determinants.
"""
import math

def emit_library(p, z, params, model, duty, dt):
 # State and operands are shared procedure arguments, never plant truth.
 dz=p.array('physics_derivative',4);scratch=p.array('physics_midpoint',4);saved=p.array('physics_saved',4)
 gain=p.reg('physics_gain');tau=p.reg('physics_tau');eff=p.reg('physics_effective');a0=p.reg('physics_a0');target=p.reg('physics_target');eh=p.reg('physics_eh');em=p.reg('physics_em');vm=p.reg('physics_vm');am=p.reg('physics_am')
 alpha,kappa,damping,coulomb,gpos,gneg,ton,toff,dead=params
 lam=p.reg('physics_lambda');bg=p.reg('physics_beta');d=p.reg('physics_d');k2=p.reg('physics_k2');kp=p.reg('physics_kp');kn=p.reg('physics_kn');ion=p.reg('physics_inverse_on');ioff=p.reg('physics_inverse_off');inv_tau=p.reg('physics_inverse_tau')
 eon=p.reg('physics_exp_on');eoff=p.reg('physics_exp_off');emon=p.reg('physics_exp_half_on');emoff=p.reg('physics_exp_half_off')
 p.mark('prepare_physics')
 lam.set(alpha*(.647/(.06*9.81*.075)));bg.set(lam/((gpos+gneg)*.5));d.set(ton*bg);k2.set(kappa*kappa)
 kp.set(bg*gpos);kn.set(bg*gneg);ion.set(1/ton);ioff.set(1/toff)
 eon.set((-dt*ion).exp());eoff.set((-dt*ioff).exp());emon.set((-.5*dt*ion).exp());emoff.set((-.5*dt*ioff).exp());p.emit('RET')
 mass_A=p.reg('physics_mass_A');mass_inv=p.reg('physics_mass_inverse');mass_cs=p.reg('physics_mass_cos');mass_drag=p.reg('physics_mass_drag')
 p.mark('derivative')
 sn=z[0].sin();cs=z[0].cos();sc=sn*cs
 force=(abs(duty)-dead).maximum(0)
 eff.set(force)
 with p.if_(duty<0):eff.set(-force)
 gain.set(gpos)
 with p.if_(duty<0):gain.set(gneg)
 target.set(gain*eff);tau.set(ton);inv_tau.set(ion)
 with p.if_(eff.eq(0)):tau.set(toff);inv_tau.set(ioff)
 restoring=alpha*sn+sc*z[3]*z[3]-damping*z[1]
 dz[0].set(z[1]);dz[2].set(z[3])
 with p.if_(model.eq(0)):
  dz[3].set((target-z[3])*inv_tau);dz[1].set(restoring-kappa*cs*dz[3])
 with p.if_(model.ne(0)):
  s2=sn*sn
  A=d+k2+s2;den=d+(1+k2)*s2;drag=p.reg('physics_drag');drag.set(bg)
  with p.if_(eff.eq(0)):drag.set((d+k2*s2)*ioff)
  drive=p.reg('physics_drive');drive.set(kp)
  with p.if_(duty<0):drive.set(kn)
  ra=drive*eff-drag*z[3]-2*sc*z[3]*z[1]+kappa*sn*z[1]*z[1]
  # One reciprocal is shared by both acceleration components.
  inverse=p.val(1)/den
  mass_A.set(A);mass_inv.set(inverse);mass_cs.set(cs);mass_drag.set(drag)
  dz[1].set((A*restoring-kappa*cs*ra)*inverse);dz[3].set((ra-kappa*cs*restoring)*inverse)
 p.emit('RET')
 p.mark('prediction_step')
 for i in range(4):saved[i].set(z[i])
 with p.if_(model.eq(0)):
  force=(abs(duty)-dead).maximum(0);eff.set(force)
  with p.if_(duty<0):eff.set(-force)
  gain.set(gpos)
  with p.if_(duty<0):gain.set(gneg)
  target.set(gain*eff);tau.set(ton);inv_tau.set(ion)
  with p.if_(eff.eq(0)):tau.set(toff);inv_tau.set(ioff)
  eh.set(eon);em.set(emon)
  with p.if_(eff.eq(0)):eh.set(eoff);em.set(emoff)
  a0.set((target-z[3])*inv_tau);vm.set(target+(z[3]-target)*em);am.set((target-vm)*inv_tau)
  sn=z[0].sin();cs=z[0].cos()
  wdot=alpha*sn-kappa*cs*a0+sn*cs*z[3]*z[3]-damping*z[1]
  tm=z[0]+dt*.5*z[1];wm=z[1]+dt*.5*wdot
  sn1=tm.sin();cs1=tm.cos()
  z[0].set(saved[0]+dt*wm)
  z[1].set(saved[1]+dt*(alpha*sn1-kappa*cs1*am+sn1*cs1*vm*vm-damping*wm))
  z[2].set(saved[2]+target*dt+(saved[3]-target)*tau*(1-eh));z[3].set(target+(saved[3]-target)*eh)
 with p.if_(model.ne(0)):
  rk=p.array('rk4',16)
  for stage in range(4):
   if stage:
    for j in range(4):z[j].set(saved[j]+dt*(.5 if stage<3 else 1)*rk[(stage-1)*4+j])
   p.call('derivative')
   for j in range(4):rk[stage*4+j].set(dz[j])
  for j in range(4):z[j].set(saved[j]+dt*(rk[j]+2*rk[4+j]+2*rk[8+j]+rk[12+j])/6)
 p.emit('RET')
 return [lam,bg,d,k2,kp,kn,ion,ioff,eon,eoff,emon,emoff]
