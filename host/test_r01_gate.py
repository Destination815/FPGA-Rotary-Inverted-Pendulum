import unittest,tempfile,json,hashlib,sys,time
from unittest.mock import patch
from pathlib import Path
from r01_gate import qualification,check,BUILD,board_ok
from studio_session import CommandPump,Request,permitted
from coordinates import SCALE
class Gates(unittest.TestCase):
 def row(self):return dict(ms=1,flags=15,state=0,fault=0,theta=0.,dtheta=0.,dphi=0.,limit=150,u=0,ack_seq=0,ack_status=0)
 def test_default_locked(self):self.assertFalse(qualification()[0])
 def test_stop_always(self):self.assertTrue(check('stop',0,{}, {},False,False)[0])
 def test_info_always(self):self.assertTrue(check('d02info',0,{}, {},False,False)[0])
 def test_all_mutations_locked(self):
  for n in ('cal','clear','arm','start','capture','target','trajectory','jog','limit','ktheta','d02start'):self.assertFalse(check(n,0,{},dict(build=BUILD,channel=1,motor_sign=-1,coordinate_verified=True),True,False)[0])
 def test_wrong_firmware(self):self.assertFalse(check('arm',0,{},dict(build=BUILD-1,channel=1,motor_sign=-1,coordinate_verified=True),True,True)[0])
 def test_stale_firmware(self):self.assertFalse(check('arm',0,{},dict(build=BUILD,channel=1,motor_sign=-1,coordinate_verified=True),False,True)[0])
 def test_bounded_motion_scope(self):
  i=dict(build=BUILD,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=41981630);row={'reference':0}
  self.assertTrue(check('start',0,row,i,True,True)[0])
  for v in (-205887,0,205887):self.assertTrue(check('target',round(v*SCALE),row,i,True,True)[0])
  for v in (-205923,205923):self.assertFalse(check('target',round(v*SCALE),row,i,True,True)[0])
  for v in (0,1):self.assertTrue(check('trajectory',v,row,i,True,True)[0])
  self.assertFalse(check('trajectory',1,{'reference':1168/65536},i,True,True)[0])
  self.assertTrue(check('trajectory',0,{'reference':1168/65536},i,True,True)[0])
 def test_frozen_params(self):
  for n in ('ktheta','kdtheta','kphi','kdphi','limit','swing','jog','d02start','d02read'):self.assertFalse(check(n,0,{},dict(build=BUILD,channel=1,motor_sign=-1,coordinate_verified=True),True,True)[0])
 def test_capture_bound(self):
  r=self.row();self.assertTrue(permitted('capture',r));r['dtheta']=.21;self.assertFalse(permitted('capture',r))
 def test_start_hanging(self):
  r=self.row();self.assertFalse(permitted('start',r));r['theta']=3.1;self.assertTrue(permitted('start',r))
 def test_no_oldparams_state_permission(self):self.assertFalse(permitted('limit',self.row()))
 def test_pump_gate_at_dispatch(self):
  sent=[];events=[];p=CommandPump(lambda x:sent.append(x) or len(x),events.append,lambda:1.)
  p.feed(self.row());p.submit([Request('arm')],'arm');p.tick();self.assertFalse(sent);self.assertEqual(events[-1]['event'],'batch_failed')
 def test_stop_preempts(self):
  sent=[];p=CommandPump(lambda x:sent.append(x) or len(x),lambda e:None,lambda:1.);p.feed(self.row());p.submit([Request('arm')],'arm');p.stop();p.tick();self.assertEqual(sent[0][3],1)
 def test_frozen_root_qualification(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);host=root/'host';host.mkdir();(root/'x.v').write_text('same source')
   d=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
   m=root/'source_manifest.json';m.write_text(json.dumps({'files':{'x.v':d(root/'x.v')}}))
   s=host/'sta_review.json';s.write_text(json.dumps(dict(accepted=True,build='4A070256',source_manifest_sha256=d(m))))
   q=host/'qualification.json';q.write_text(json.dumps(dict(schema='r01.motion-release.v1',build='4A070256',motion_release=True,source_manifest_sha256=d(m),sta_review_sha256=d(s))))
   with patch.object(sys,'frozen',True,create=True),patch.object(sys,'executable',str(root/'J280_Studio.exe')):
    self.assertTrue(qualification()[0]);(root/'x.v').write_text('changed');self.assertFalse(qualification()[0])
 def test_receipt_binding(self):
  with tempfile.TemporaryDirectory() as t:
   r=Path(t);h=r/'host';h.mkdir();(r/'x.v').write_text('source')
   d=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
   m=r/'source_manifest.json';m.write_text(json.dumps({'files':{'x.v':d(r/'x.v')}}));s=h/'sta_review.json';s.write_text(json.dumps(dict(accepted=True,build='4A070256',source_manifest_sha256=d(m))))
   q=h/'qualification.json';q.write_text(json.dumps(dict(schema='r01.motion-release.v1',build='4A070256',motion_release=True,source_manifest_sha256=d(m),sta_review_sha256=d(s))))
   self.assertTrue(qualification(h)[0]);(r/'x.v').write_text('changed');self.assertFalse(qualification(h)[0]);(r/'x.v').write_text('source');s.write_text('{}');self.assertFalse(qualification(h)[0])
if __name__=='__main__':unittest.main(verbosity=2)
