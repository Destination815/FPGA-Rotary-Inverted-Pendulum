"""Compatibility, units, and preservation of fractional calibration metadata."""
import struct, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from protocol import crc, decode_d02, Decoder, CONTROL_BUILD
from studio_session import SerialSession

def packet(build=CONTROL_BUILD,tag=0xF401):
 b=bytearray(64);b[:4]=b'\xa5\x5a\x02\x40'
 struct.pack_into('<I',b,4,build);struct.pack_into('<I',b,28,12340)
 struct.pack_into('<HHH',b,44,11843,11849,tag);b[50]=1;b[51]=1
 b[62:64]=crc(b[2:62]).to_bytes(2,'little');return bytes(b)
class FilterInfo(unittest.TestCase):
 def test_exact_fraction_units(self):
  p=decode_d02(packet());self.assertEqual(p['adc_q4']/16,740.1875);self.assertEqual(p['zero_q4']/16,740.5625);self.assertEqual(p['ms'],12340)
 def test_legacy_or_untagged_does_not_invent_filter_info(self):
  for b in [packet(0x4A070202),packet(tag=0)]:self.assertNotIn('adc_q4',decode_d02(b))
 def test_fragmented_crc_checked(self):
  d=Decoder();b=packet()
  for c in b:d.feed(bytes([c]))
  self.assertEqual(len(d.take_diagnostics()),1)
  b=bytearray(b);b[44]^=1;d.feed(b);self.assertFalse(d.take_diagnostics())
 def test_control_build_metadata_before_legacy_return(self):
  with tempfile.TemporaryDirectory() as t:
   s=SerialSession('NO_DEVICE',Path(t));s.recorder=SimpleNamespace(meta={},event=lambda e:None)
   s.diagnostic_packet(decode_d02(packet()));m=s.recorder.meta
   self.assertEqual(m['adc_filter_info_snapshot']['zero_q4'],11849)
   self.assertFalse(m['measurement_semantics']['host_smoothing']);self.assertEqual(s.snapshot()['d02_info']['adc_q4'],11843)
if __name__=='__main__':unittest.main(verbosity=2)
