"""Real worker event loop against an in-memory serial device, no hardware access."""
import tempfile,time,struct,unittest,json
from pathlib import Path
from protocol import crc
from studio_session import SerialSession
class Port:
 def __init__(self,*a,**k):self.ms=0;self.seq=0;self.writes=[];self.pending=b'';self.closed=False
 @property
 def in_waiting(self):return 1
 def write(self,b):
  self.seq=b[2];self.writes.append((b[3],int.from_bytes(b[4:8],'little',signed=True)))
  if b[3]==48:
   d=bytearray(64);d[:4]=b'\xa5\x5a\x02\x40';struct.pack_into('<I',d,4,0x4A070256);struct.pack_into('<I',d,28,self.ms);struct.pack_into('<H',d,22,150);d[34]=255;d[35]=1;d[62:]=crc(d[2:62]).to_bytes(2,'little');self.pending+=d
  return len(b)
 def read(self,n):
  time.sleep(.01);self.ms+=20;d=bytearray(64);d[:4]=b'\xa5\x5a\x01\x40';struct.pack_into('<I',d,4,self.ms);struct.pack_into('<H',d,42,14);d[44]=self.seq;struct.pack_into('<H',d,56,150);d[62:]=crc(d[2:62]).to_bytes(2,'little');out=bytes(d)+self.pending;self.pending=b'';return out
 def close(self):self.closed=True
class Worker(unittest.TestCase):
 def test_locked_worker_and_finalized_log(self):
  port=Port()
  with tempfile.TemporaryDirectory() as t:
   s=SerialSession('FAKE',Path(t),serial_factory=lambda *a,**k:port);s.start();time.sleep(.15)
   for name,value in [('arm',0x4a433131),('start',0),('capture',0),('target',0),('trajectory',1),('limit',350),('ktheta',0)]:s.request('command',dict(name=name,value=value))
   s.request('profile',dict(name='malicious old config'));time.sleep(.3);s.request('close');s.join(3)
   self.assertFalse(s.is_alive());self.assertTrue(port.closed)
   self.assertTrue(port.writes);self.assertTrue(all(op in (1,48) for op,v in port.writes),port.writes)
   self.assertEqual(s.d02_info['build'],0x4A070256)
   metas=list(Path(t).glob('*.meta.json'));self.assertEqual(len(metas),1)
   meta=json.loads(metas[0].read_text());self.assertFalse(meta['qualification_ok']);self.assertEqual(meta['expected_build'],'4A070256');self.assertIn('source_manifest',meta['qualification_snapshot']);self.assertEqual(meta['qualification_snapshot']['qualification_receipt']['contents']['build'],'4A070256')
if __name__=='__main__':unittest.main(verbosity=2)
