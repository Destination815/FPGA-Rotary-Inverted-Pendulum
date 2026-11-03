"""R43 board-owned session-center admission; no serial or vendor process opened."""
from pathlib import Path
import argparse,hashlib,json,math,os,sys,unittest
P=Path(__file__).resolve().parents[2]
os.environ['R40_PROJECT']=str(P)
sys.path.insert(0,str(P/'tools'))
import test_r40_host as base
from ext2_sequence import Ext2Sequence
from sine_config import settings
from motion_config import pack_internal

class CenterAdmission(unittest.TestCase):
 def setUp(self):
  self.clock=base.Clock();self.plan=Ext2Sequence(self.clock);self.serial=1;self.sent=[]
  self.cfg=pack_internal(180,15,2,20,True)
 def info(self,**kw):return base.info(motion_config=self.cfg,**kw)
 def tick(self,row,info=None):
  self.plan.tick(row,info or self.info(),[],True,True,True,True)
  actions,events=self.plan.drain();self.sent.extend(actions);return actions,events
 def ack(self,name,status=0):
  seq=self.serial;self.serial+=1
  self.plan.event(dict(event='send',name=name,seq=seq))
  self.plan.event(dict(event='ack',name=name,seq=seq,status=status,ms=1020))
 def configured_entry(self,row,amp=90,period=9.42,already_requested=False):
  if not already_requested:self.assertEqual(self.plan.begin(row,self.info(),True,True,True,settings=dict(amplitude_deg=amp,period_s=period)),'')
  self.assertEqual(self.plan.state,'WAIT_HOLD')
  self.assertEqual(self.tick(row)[0],[]);self.assertEqual(self.plan.state,'CONFIG_AMPLITUDE')
  chosen=settings(amp,period)
  self.assertEqual(self.tick(row)[0],[('sine_amplitude',chosen['amplitude_physical_q16'])]);self.ack('sine_amplitude')
  self.assertEqual(self.tick(row)[0],[('sine_period',chosen['phase_step'])]);self.ack('sine_period')
  self.assertEqual(self.plan.state,'CONFIG_READBACK')
  readback=self.info(sine_settings_fresh=True,sine_amplitude_internal_q16=chosen['amplitude_internal_q16'],sine_phase_step=chosen['phase_step'])
  self.assertEqual(self.tick(row,readback)[0],[]);self.assertEqual(self.plan.state,'WAIT_HOLD')
  actions,events=self.tick(row,readback);self.assertEqual(actions,[('ext2',1)]);self.assertEqual(self.plan.state,'WAIT_ACK')
  event=next(e for e in events if e['event']=='ext2_entry_sent');self.assertEqual(event['entry_reference_physical_rad'],row['reference']);self.assertNotIn('center_physical_rad',event)
 def running(self,row):
  self.ack('ext2');self.assertEqual(self.plan.state,'WAIT_FLAG')
  self.assertEqual(self.tick(dict(row,state=4,flags=23|256),self.info(ext2_prepared=True))[0],[]);self.assertEqual(self.plan.state,'RUNNING')
 def test_positive_recovery_reference_beyond_old_gate_submits_actual_center_to_board(self):
  r=base.row(reference=math.radians(90));self.configured_entry(r);self.running(r)
 def test_negative_recovery_reference_beyond_old_gate_submits_actual_center_to_board(self):
  r=base.row(reference=math.radians(-90));self.configured_entry(r);self.running(r)
 def test_large_legal_amplitude_does_not_use_instantaneous_reference_as_center(self):
  r=base.row(reference=math.radians(120));self.configured_entry(r,amp=120,period=12.57);self.running(r)
 def test_amplitude_too_wide_for_any_center_is_rejected_before_send(self):
  r=base.row(reference=0);self.assertEqual(self.plan.begin(r,self.info(),True,True,True,settings=dict(amplitude_deg=180,period_s=12)),'')
  self.assertEqual(self.tick(r)[0],[]);self.assertEqual(self.plan.state,'READY');self.assertFalse(self.plan.active);self.assertIn('半幅超出',self.plan.message);self.assertEqual(self.sent,[])
 def test_board_center_rejection_does_not_report_running_or_stop_existing_balance(self):
  r=base.row(reference=0);self.configured_entry(r);self.ack('ext2',2)
  self.assertEqual(self.plan.state,'READY');self.assertFalse(self.plan.active);self.assertIsNone(self.plan.pending_seq)
  self.assertEqual(self.tick(r)[0],[]);self.assertFalse(any(name=='stop'for name,value in self.sent));self.assertNotIn('运行中',self.plan.message)
 def test_parameter_edit_exits_then_reenters_without_old_reference_precheck(self):
  r=base.row(reference=math.radians(90));self.configured_entry(r);self.running(r);self.sent=[]
  active=dict(r,state=4,flags=23|256,reference=math.radians(150))
  self.assertEqual(self.plan.begin(active,self.info(ext2_prepared=True),True,True,True,settings=dict(amplitude_deg=120,period_s=12.57)),'')
  self.assertEqual(self.plan.state,'EXIT_READY');self.assertEqual(self.tick(active,self.info(ext2_prepared=True))[0],[('ext2',0)]);self.ack('ext2');self.assertEqual(self.plan.state,'EXITING')
  self.assertEqual(self.tick(active,self.info(ext2_prepared=True))[0],[]);self.assertEqual(self.plan.state,'EXITING')
  returned=base.row(reference=math.radians(150));self.assertEqual(self.tick(returned)[0],[]);self.assertEqual(self.plan.state,'WAIT_HOLD')
  self.configured_entry(returned,amp=120,period=12.57,already_requested=True);self.running(returned)
  self.assertEqual([name for name,value in self.sent],['ext2','sine_amplitude','sine_period','ext2'])
 def test_shaky_reference_has_no_added_settled_or_damping_gate(self):
  r=base.row(reference=math.radians(90),theta=.14,dtheta=3,dphi=2,flags=23|128|2048);self.configured_entry(r);self.running(r)
 def test_accepted_ack_alone_does_not_report_running_before_board_flag(self):
  r=base.row(reference=math.radians(90));self.configured_entry(r);self.ack('ext2');self.assertEqual(self.plan.state,'WAIT_FLAG')
  self.assertEqual(self.tick(r)[0],[]);self.assertEqual(self.plan.state,'WAIT_FLAG')

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
 files=['host/ext2_sequence.py','tools/test_r40_host.py','tools/r43/test_center_admission_host.py'];hashes={n:hashlib.sha256((P/n).read_bytes()).hexdigest()for n in files}
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CenterAdmission))
 unchanged=all(hashlib.sha256((P/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
 receipt=dict(status='PASS'if result.wasSuccessful()and unchanged else'FAIL',tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),source_sha256=hashes,unchanged=unchanged,hardware_access=False,Windows_executed=False,scope='Actual production Ext2Sequence through configuration ACK/readback, entry ACK/board flag, rejection and reconfiguration with deterministic fake clock. No private center inferred from telemetry reference; actual center geometry remains a board RTL contract.')
 (a.out/'RESULT.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n');print(json.dumps(receipt,ensure_ascii=False,indent=2));return 0 if receipt['status']=='PASS'else 1
if __name__=='__main__':raise SystemExit(main())
