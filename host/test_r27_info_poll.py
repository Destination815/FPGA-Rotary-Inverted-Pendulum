import unittest
from types import SimpleNamespace
from studio_session import SerialSession
class Poll(unittest.TestCase):
 def make(self):
  return SimpleNamespace(clock=lambda:10.,pump=SimpleNamespace(busy=False,fresh=True),info_requested_at=0.,d02_info={'build':0x4A070256},d02_info_at=9.9,finalize=False,closing=False)
 def test_unsolicited_info_does_not_suppress_explicit_heartbeat(self):self.assertTrue(SerialSession.info_poll_due(self.make()))
 def test_stale_recovers(self):
  s=self.make();s.d02_info_at=9.;self.assertTrue(SerialSession.info_poll_due(s))
 def test_missing_info_recovers(self):
  s=self.make();s.d02_info=None;self.assertTrue(SerialSession.info_poll_due(s))
 def test_busy_stale_or_finalize_never_query(self):
  for field in ['busy','fresh','finalize','closing','spacing']:
   s=self.make();s.d02_info=None
   if field=='busy':s.pump.busy=True
   elif field=='fresh':s.pump.fresh=False
   elif field=='spacing':s.info_requested_at=9.9
   else:setattr(s,field,True)
   self.assertFalse(SerialSession.info_poll_due(s),field)
if __name__=='__main__':unittest.main(verbosity=2)
