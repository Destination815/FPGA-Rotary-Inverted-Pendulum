"""Targeted COAST record/wire/permission checks, synthetic data, no hardware."""
import unittest,struct,tempfile,time
from pathlib import Path
from protocol import STATES,CONTROL_BUILD,crc,decode_d02
from studio_data import FIELDS,ACTIVE,BALANCE,summarize,output_issue,RunWriter,load_run
from studio_session import permitted,SerialSession

def row(ms,state=0,flags=2,**kw):
 r={k:0 for k in FIELDS};r.update(ms=ms,state=state,flags=flags,limit=150,zero=747,adc=747);r.update(kw);return r
class CoastTests(unittest.TestCase):
 def test_motion_segment_preserved(self):
  rows=[row(0),row(20,1,19),row(40,7,3),row(60,7,3),row(80,1,19),row(100,3,23),row(120,3,23),row(140)]
  s=summarize(rows);self.assertEqual(len(s['motions']),1);self.assertEqual(s['motions'][0]['active_frames'],6);self.assertEqual(s['motions'][0]['coast_observed_frames'],2);self.assertFalse(s['motions'][0]['control_evaluation_eligible']);self.assertEqual(s['captures'][0]['active_frames'],2);self.assertIn(7,ACTIVE);self.assertNotIn(7,BALANCE)
 def test_zero_output_contract(self):
  self.assertIsNone(output_issue(row(0,7,3)))
  for r in [row(0,7,3,u=1),row(0,7,19)]:self.assertIn('重获',output_issue(r))
 def test_wire_preserves_state_and_fractional_adc(self):
  f=bytearray(64);f[:4]=b'\xa5\x5a\x03\x40';struct.pack_into('<I',f,4,0x80000001);struct.pack_into('<H',f,8,0)
  fine=12001;packed_index=150|((fine>>9)<<10);packed_state=7|((fine&31)<<3);packed_fault=((fine>>5)&15)<<4
  struct.pack_into('<IHBBHHiiiiihhHH',f,12,1000,packed_index,packed_state,packed_fault,0,0,123,0,0,0,0,0,0,3,0)
  struct.pack_into('<I',f,52,CONTROL_BUILD);struct.pack_into('<H',f,56,1);f[59]=4
  q=(744<<22)|(1<<21);struct.pack_into('<H',f,10,q&65535);struct.pack_into('<H',f,60,q>>16);struct.pack_into('<H',f,62,crc(f[2:62]))
  from coordinates import PROFILE,BUILD,SCHEMA,TOKEN,RPC_Q24,TARGET_K
  coord=dict(build=BUILD,contract_token=TOKEN,schema=SCHEMA,cpr_num=16174,cpr_den=15,legacy_q16_per_count=396,physical_rpc_q24=RPC_Q24,target_k_q24=TARGET_K,capture_id=0x80000001,recording_encoder_zero=0)
  p=decode_d02(f,coordinates=coord);self.assertEqual(p['row']['state'],7);self.assertEqual(p['row']['adc_q4'],fine);self.assertEqual(p['row']['fault'],0);self.assertEqual(STATES[7],'COAST')
 def test_coast_does_not_accept_motion(self):
  r=row(0,7,3)
  self.assertTrue(permitted('stop',r));self.assertTrue(permitted('d02info',r))
  for cmd in ['arm','start','capture','target','trajectory','motioncfg','cal','clear']:self.assertFalse(permitted(cmd,r),cmd)
 def test_run_writer_retains_coast(self):
  with tempfile.TemporaryDirectory()as td:
   w=RunWriter(td,{});w.row(row(0,1,19));w.row(row(20,7,3));w.row(row(40,1,19));w.close();r=load_run(w.path);self.assertEqual([x['state']for x in r['rows']],[1,7,1]);self.assertEqual(len(r['summary']['motions']),1)
 def test_worker_keeps_recording_armed_coast(self):
  class Port:
   def __init__(self):self.ms=0;self.seq=0;self.stopped=False;self.writes=[]
   @property
   def in_waiting(self):return 1
   def write(self,b):self.seq=b[2];self.writes.append(b[3]);self.stopped|=b[3]==1;return len(b)
   def read(self,n):
    time.sleep(.005);self.ms+=20;f=bytearray(64);f[:4]=b'\xa5\x5a\x01\x40';struct.pack_into('<I',f,4,self.ms);f[40]=0 if self.stopped else 7;struct.pack_into('<H',f,42,2 if self.stopped else 3);f[44]=self.seq;struct.pack_into('<H',f,56,150);struct.pack_into('<H',f,62,crc(f[2:62]));return bytes(f)
   def close(self):pass
  with tempfile.TemporaryDirectory()as td:
   p=Port();s=SerialSession('SYNTHETIC_COAST',td,serial_factory=lambda *a,**kw:p);s.start();time.sleep(.08);self.assertTrue(s.is_alive());self.assertIsNotNone(s.recorder);self.assertEqual(s.latest['state'],7);self.assertFalse(s.qualification_ok);s.request('close');s.join(3);self.assertFalse(s.is_alive());r=load_run(next(Path(td).glob('*.csv')));self.assertGreater(r['summary']['coast_observed_frames'],3);self.assertTrue(all(op in (1,48)for op in p.writes))
if __name__=='__main__':unittest.main(verbosity=2)
