import unittest,threading
from types import SimpleNamespace
from studio_session import SerialSession,Request
from session_preparation import swing_reason
from ext2_sequence import Ext2Sequence
from unittest.mock import patch
class StartConfig(unittest.TestCase):
 def test_matching_disabled_rejected(self):self.assertTrue(swing_reason({'motion_config':0x028896be},{'motion_config':0x028896be}))
 def test_only_selected_rejected(self):self.assertTrue(swing_reason({'motion_config':0x128896be},{'motion_config':0x028896be}))
 def test_matching_enabled(self):self.assertEqual(swing_reason({'motion_config':0x128896be},{'motion_config':0x128896be}),'')
 def test_worker_checks_start_only(self):
  s=SimpleNamespace(lock=threading.Lock(),preparation={'motion_config':0x028896be},expected_bitstreams=[],d02_info={'motion_config':0x028896be},clock=lambda:10,d02_info_at=10,pump=SimpleNamespace(latest={}),qualification_ok=True,ext2_prepare_stage=None,ext2_sequence=Ext2Sequence(),view_rows=[])
  s._board_info=lambda:s.d02_info
  with patch('studio_session.preparation_reason',return_value=''),patch('studio_session.check',return_value=(True,'')):
   self.assertFalse(SerialSession.command_gate(s,Request('start',0,0x028896be))[0])
   for name in ['capture','target','stop']:self.assertTrue(SerialSession.command_gate(s,Request(name,0,0x028896be))[0])
   s.preparation['motion_config']=s.d02_info['motion_config']=0x128896be
   self.assertTrue(SerialSession.command_gate(s,Request('start',0,0x128896be))[0])
if __name__=='__main__':unittest.main(verbosity=2)
