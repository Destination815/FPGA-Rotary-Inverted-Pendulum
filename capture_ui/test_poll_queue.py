import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
import unittest
from studio_session import Request
from poll_queue import PollFriendlyPump
class QueueTests(unittest.TestCase):
 def setup_poll(self):
  self.clock=10.;self.sent=[];self.events=[]
  p=PollFriendlyPump(lambda b:self.sent.append(b) or len(b),self.events.append,lambda:self.clock)
  p.gate=lambda req:(True,'')
  self.row=dict(ms=1,flags=14,state=0,fault=0,ack_seq=0,ack_status=0,u=0,theta=0,dtheta=0,dphi=0)
  p.feed(self.row);p.submit([Request('d02info')],'version_poll');p.tick()
  return p
 def ack_info(self,p):
  self.clock+=.02;p.feed(dict(self.row,ms=21,ack_seq=p.pending['seq']))
 def test_one_click_survives_query(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm');self.assertEqual(len(self.sent),1)
  self.ack_info(p);p.tick();self.assertEqual([b[3] for b in self.sent],[48,3]);self.assertIsNone(p.manual_wait_until)
 def test_no_wait_for_second_manual(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm')
  with self.assertRaises(ValueError):p.submit([Request('capture')],'capture')
 def test_300ms_expiry_never_sends_motion(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm');self.clock+=.301;p.tick()
  self.assertEqual([b[3] for b in self.sent],[48]);self.assertFalse(p.busy)
 def test_stop_cancels_waiting_click(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm');p.stop();p.tick()
  self.assertEqual([b[3] for b in self.sent],[48,1]);self.assertIsNone(p.manual_wait_until)
 def test_send_time_gate_rechecked(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm');self.ack_info(p);p.gate=lambda req:(False,'identity expired');p.tick()
  self.assertEqual([b[3] for b in self.sent],[48]);self.assertFalse(p.busy)
 def test_send_time_state_rechecked(self):
  p=self.setup_poll();p.submit([Request('arm',0x4a433131)],'arm');self.ack_info(p);p.latest['fault']=1;p.tick()
  self.assertEqual([b[3] for b in self.sent],[48])
 def test_normal_manual_busy_remains_rejected(self):
  p=self.setup_poll();self.ack_info(p);p.tick();p.submit([Request('arm',0x4a433131)],'arm');p.tick()
  with self.assertRaises(ValueError):p.submit([Request('capture')],'capture')
if __name__=='__main__':unittest.main(verbosity=2)
