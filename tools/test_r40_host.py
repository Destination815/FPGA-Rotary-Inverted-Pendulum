"""Independent R40 contract fixtures. No physical port, plant, or Gowin."""
from pathlib import Path
import argparse, json, math, os, sys, tempfile, time, unittest, struct, hashlib
from unittest.mock import patch
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
HERE=Path(__file__).resolve().parent
P=Path(os.environ.get('R40_PROJECT', HERE.parent)).resolve()
sys.path[:0]=[str(P/'target_ui_fix'),str(P/'capture_ui'),str(P/'host')]
from ext2_sequence import Ext2Sequence, capture_row, hold_row
from coordinates import BUILD, SCALE, PROFILE, SCHEMA
from motion_config import pack_internal, speed_q16, speed_internal_q16
from studio_session import SerialSession, Request, permitted
from protocol import crc, OPS
from r01_gate import check

CONFIG=pack_internal(360,15,2,20,True)
SPEED=speed_internal_q16(30)

class Clock:
    def __init__(self):self.t=100.
    def __call__(self):return self.t

def row(ms=1000,**updates):
    d=dict(ms=ms,state=3,fault=0,flags=7|16|128,theta=math.radians(3),dtheta=1.,phi=math.radians(90),dphi=.6,reference=math.radians(90),u=50,ack_seq=0,ack_status=0,adc=742,zero=742)
    d.update(updates);return d

def info(**updates):
    d=dict(build=BUILD,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=CONFIG,motion_speed_internal_q16=SPEED,capabilities=255,ext2_prepared=False,ext2_consumed=False)
    d.update(updates);return d

class Sequence(unittest.TestCase):
    def setUp(self):self.clock=Clock();self.plan=Ext2Sequence(self.clock)
    def enter(self,r=None):
        r=r or row()
        self.assertEqual(self.plan.begin(r,info(),True,True,True),'')
        self.plan.tick(r,info(),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[('ext2',1)])
        self.plan.event(dict(event='send',name='ext2',value_i32=1,seq=17))
        self.plan.event(dict(event='ack',name='ext2',status=0,seq=17,ms=r['ms']+20))
        self.plan.tick(row(r['ms']+40,state=4,flags=23|256),info(ext2_prepared=True),[],True,True,True,True)
        self.assertEqual(self.plan.state,'RUNNING')
    def exit(self):
        self.assertEqual(self.plan.exit(),'')
        self.plan.tick(row(3000,state=4,flags=23|256),info(ext2_prepared=True),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[('ext2',0)])
        self.plan.event(dict(event='send',name='ext2',value_i32=0,seq=18))
        self.plan.event(dict(event='ack',name='ext2',status=0,seq=18,ms=3020))
        self.plan.tick(row(3040),info(),[],True,True,True,True)
        self.assertEqual(self.plan.state,'READY')
    def test_automatic_capture_no_recapture_configuration_or_stillness(self):
        self.plan.event(dict(event='ack',name='start',status=0,ms=10))
        self.enter();self.assertEqual(self.plan.capture_ms,10)
        self.assertEqual(self.plan.drain()[0],[])
    def test_manual_capture_nonzero_center_identical_entry(self):
        self.plan.event(dict(event='ack',name='capture',status=0,ms=10))
        self.enter(row(phi=math.radians(-90),reference=math.radians(-90)))
        self.assertEqual(self.plan.capture_ms,10)
    def test_capture_pending_waits_only_for_hold_not_stillness(self):
        r=row(state=2,flags=23)
        self.assertEqual(self.plan.begin(r,info(),True,True,True),'')
        for ms in (1000,2000,500000):
            self.plan.tick(row(ms,state=2,flags=23),info(),[],True,True,True,True)
            self.assertEqual(self.plan.drain()[0],[])
        self.plan.tick(row(500020),info(),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[('ext2',1)])
    def test_position_move_waits_reference_complete(self):
        r=row(state=4,flags=23|256)
        self.assertEqual(self.plan.begin(r,info(),True,True,True),'')
        self.plan.tick(r,info(),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[])
        self.plan.tick(row(1020),info(),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[('ext2',1)])
    def test_one_hour_run_has_no_timer_stop(self):
        self.enter();self.clock.t+=3600
        self.plan.tick(row(3601040,state=4,flags=23|256),info(ext2_prepared=True),[],True,True,True,True)
        self.assertEqual(self.plan.state,'RUNNING');self.assertIsNone(self.plan.deadline)
        self.assertEqual(self.plan.drain()[0],[])
    def test_exit_then_ordinary_target_and_reenter_no_stop(self):
        self.enter();self.exit()
        self.assertEqual(self.plan.command_reason(Request('target',0),row(),info(),[]),'')
        self.enter();self.assertEqual(self.plan.drain()[0],[])
    def test_cancel_pending_request_does_not_stop_balance(self):
        self.assertEqual(self.plan.begin(row(state=2,flags=23),info(),True,True,True),'')
        self.assertEqual(self.plan.exit(),'');self.assertEqual(self.plan.state,'READY')
        self.assertEqual(self.plan.drain()[0],[])
    def test_telemetry_reference_does_not_stand_in_for_session_center(self):
        # R43: telemetry reference may be a recovery catch, not the saved center.
        # The board validates its actual Q18 session center, including boundary bits.
        bound=(360*1144-60058)*SCALE/65536
        for sign in (-1,1):
            for increment,expected in ((0,[('ext2',1)]),(SCALE/65536,[('ext2',1)])):
                self.setUp();r=row(reference=sign*(bound+increment))
                self.assertEqual(self.plan.begin(r,info(),True,True,True),'')
                self.plan.tick(r,info(),[],True,True,True,True)
                self.assertEqual(self.plan.drain()[0],expected)
    def test_stop_cancels_pending_and_running(self):
        self.enter();self.plan.event(dict(event='send',name='stop',seq=25))
        self.assertFalse(self.plan.active);self.assertEqual(self.plan.drain()[0],[])
    def test_recovery_cancels_ext2_without_forcing_stop(self):
        self.enter();self.plan.tick(row(2000,state=1,flags=7),info(),[],True,True,True,True)
        self.assertFalse(self.plan.active);self.assertEqual(self.plan.drain()[0],[])
    def test_entry_identity_and_state(self):
        for kw in ({'build':0x4a070250},{'channel':2},{'motor_sign':1},{'coordinate_verified':False}):
            self.assertTrue(self.plan.begin(row(),info(**kw),True,True,True))
        for kw in ({'state':0},{'fault':4},{'theta':float('nan')},{'flags':0}):
            self.assertTrue(self.plan.begin(row(**kw),info(),True,True,True))
        for conditions in ((False,True,True),(True,False,True),(True,True,False)):
            self.assertTrue(self.plan.begin(row(),info(),*conditions))
    def test_lost_ack_timeout_has_stop(self):
        self.plan.begin(row(),info(),True,True,True);self.plan.tick(row(),info(),[],True,True,True,True);self.plan.drain()
        self.clock.t+=2.1;self.plan.tick(row(3100),info(),[],True,True,True,True)
        self.assertEqual(self.plan.drain()[0],[('stop',0)])
    def test_sequence_exclusivity_keeps_stop_and_query(self):
        self.enter()
        for name in ('target','trajectory','capture','start','motioncfg','motion_speed'):
            self.assertTrue(self.plan.command_reason(Request(name,1),row(),info(),[]))
        for name in ('stop','d02info'):
            self.assertEqual(self.plan.command_reason(Request(name),{}, {},[]),'')

class Gates(unittest.TestCase):
    def test_ext2_begin_in_hold_preserves_numeric_config(self):
        self.assertTrue(permitted('ext2',row()))
        self.assertTrue(check('ext2',1,row(),info(),True,True)[0])
    def test_ext2_exit_in_motion_and_invalid_value(self):
        r=row(state=4,flags=23|256)
        self.assertTrue(check('ext2',0,r,info(ext2_prepared=True),True,True)[0])
        self.assertFalse(check('ext2',2,r,info(),True,True)[0])
    def test_stop_survives_all_missing_inputs(self):
        self.assertTrue(check('stop',0,{}, {},False,False)[0]);self.assertTrue(permitted('stop',{},False))

class WirePort:
    """Valid CRC wire transport; responses are simulated, not a mechanical model."""
    def __init__(self):
        self.ms=0;self.seq=0;self.cfg=CONFIG;self.speed=SPEED;self.active=False
        self.flags=14;self.state=0;self.writes=[];self.closed=False
        self.theta=math.pi;self.ref=math.radians(90)/SCALE;self.position=math.radians(90)
        self.moving_until=0
    @property
    def in_waiting(self):return 192
    def write(self,b):
        if len(b)!=10 or crc(b[2:8])!=int.from_bytes(b[8:10],'little'):raise ValueError('bad host CRC')
        self.seq=b[2];op=b[3];value=int.from_bytes(b[4:8],'little',signed=True)
        self.writes.append((op,value))
        if op==54:self.cfg=value;self.speed=0
        elif op==55:
            from coordinates import target_internal_q16
            self.speed=target_internal_q16(value)
        elif op==56:
            if value:self.active=True;self.state=4;self.flags=23|256
            else:self.active=False;self.state=3;self.flags=23|128
        elif op==3:self.flags=15
        elif op in (4,5):self.flags=23|128;self.state=3;self.theta=math.radians(3)
        elif op==38:
            from coordinates import target_internal_q16
            self.ref=target_internal_q16(value)/65536
            self.state=4;self.flags=23|256;self.moving_until=self.ms+80
        elif op==1:self.flags=14;self.state=0;self.active=False
        return len(b)
    def packet(self,kind):
        d=bytearray(64);d[:4]=bytes((165,90,kind,64))
        if kind==5:
            struct.pack_into('<I',d,4,self.ms)
            for at,value in ((8,self.theta),(12,0 if self.state==0 else 1.),(16,self.position),(20,0 if self.state==0 else .6/SCALE),(24,self.ref)):
                struct.pack_into('<i',d,at,round(value*65536))
            struct.pack_into('<i',d,32,0 if self.state==0 else 50)
            d[40]=self.state;struct.pack_into('<H',d,42,self.flags);d[44]=self.seq
            struct.pack_into('<H',d,56,350)
        elif kind==2:
            struct.pack_into('<I',d,4,BUILD);struct.pack_into('<I',d,28,self.ms)
            struct.pack_into('<H',d,22,350);d[34]=255;d[35]=1
            struct.pack_into('<H',d,38,255);struct.pack_into('<H',d,48,0xf401)
            d[51]=1|(2 if self.active else 0);struct.pack_into('<I',d,52,self.cfg);struct.pack_into('<H',d,56,0x1401)
        else:
            for at,key in ((4,'build'),(8,'contract_token'),(16,'cpr_num'),(20,'cpr_den'),(24,'legacy_q16_per_count'),(28,'physical_rpc_q24'),(32,'target_legacy_per_physical_q24')):
                struct.pack_into('<I',d,at,int(PROFILE[key],16) if key=='build' else PROFILE[key])
            struct.pack_into('<H',d,12,SCHEMA);struct.pack_into('<I',d,48,self.ms)
            struct.pack_into('<i',d,52,self.speed);struct.pack_into('<I',d,56,0x3601);struct.pack_into('<H',d,60,0xc101)
        d[62:]=crc(d[2:62]).to_bytes(2,'little');return bytes(d)
    def read(self,n):
        time.sleep(.002);self.ms+=20
        if self.moving_until and self.ms>=self.moving_until:
            self.moving_until=0;self.state=3;self.flags=23|128
        return self.packet(5)+self.packet(2)+self.packet(6)
    def close(self):self.closed=True

class Worker(unittest.TestCase):
    def wait_for(self,condition,seconds=4):
        end=time.monotonic()+seconds
        while not condition() and time.monotonic()<end:time.sleep(.005)
        self.assertTrue(condition(),'CRC worker fixture deadline')
    def test_actual_crc_worker_start_enter_exit_target_reenter_stop(self):
        wire=WirePort()
        with tempfile.TemporaryDirectory() as td:
            q=SerialSession('SOFTWARE_ONLY_NO_PORT',Path(td),serial_factory=lambda *a,**k:wire)
            # Software-only qualification fixture, never applied to production files.
            q.qualification_ok=True;q.expected_bitstreams=[]
            q.set_preparation(dict(motion_config=CONFIG,motion_speed_internal_q16=SPEED,range_confirmed=True))
            q.start()
            try:
                self.wait_for(lambda:q.connected and q.coordinate_info.get('verified') and q.pump and not q.pump.busy)
                q.request('command',dict(name='arm',value=0x41524d31,expected_motion_config=CONFIG))
                self.wait_for(lambda:wire.flags==15 and not q.pump.busy)
                q.request('command',dict(name='start',seconds=0,expected_motion_config=CONFIG))
                self.wait_for(lambda:wire.state==3 and not q.pump.busy)
                q.request('ext2_begin');self.wait_for(lambda:q.ext2_sequence.state=='RUNNING')
                self.assertEqual((wire.cfg,wire.speed),(CONFIG,SPEED))
                self.assertEqual([p for p in wire.writes if p[0] in (1,4,5,54,55,56)],[(4,0),(56,1)])
                wire.ms+=3600000;self.wait_for(lambda:q.pump.latest.get('ms',0)>3600000)
                self.assertEqual(q.ext2_sequence.state,'RUNNING');self.assertIsNone(q.deadline)
                q.request('ext2_exit');self.wait_for(lambda:q.ext2_sequence.state=='READY' and not q.pump.busy)
                q.request('command',dict(name='target',value=round(math.radians(-90)*65536),expected_motion_config=CONFIG))
                self.wait_for(lambda:any(o==38 for o,v in wire.writes) and wire.state==3 and not q.pump.busy)
                q.request('ext2_begin');self.wait_for(lambda:q.ext2_sequence.state=='RUNNING')
                self.assertEqual((wire.cfg,wire.speed),(CONFIG,SPEED))
                self.assertEqual([p for p in wire.writes if p[0] in (1,4,5,54,55,56)],[(4,0),(56,1),(56,0),(56,1)])
                q.request('stop');self.wait_for(lambda:wire.state==0 and not q.pump.busy)
                self.assertFalse(q.ext2_sequence.active);self.assertFalse(q.last_error,q.last_error)
                q.request('close');q.join(8);self.assertFalse(q.is_alive())
                self.assertTrue(list(Path(td).glob('*.commands.jsonl')))
                self.assertFalse(list(Path(td).rglob('*.fs')))
            finally:
                if q.is_alive():q.request('close');q.join(9)

class ActualQt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app=QApplication.instance() or QApplication([])
    def window(self):
        import launch_target
        with patch('serial.tools.list_ports.comports',return_value=[]):w=launch_target.TargetStudio()
        w.timer.stop();w.set_config_fields(CONFIG);w.motion_speed.setValue(30)
        w.motion_range_check.setChecked(True);w.full_turn_recovery.setChecked(True)
        return w
    def snap(self,stage='READY',active=False):
        r=row(state=4,flags=23|256) if active else row()
        return dict(connected=True,last_rx=time.monotonic(),qualification_ok=True,expected_bitstreams=[],qualification_note='SOFTWARE FIXTURE',d02_fresh=True,d02_info=info(ext2_prepared=active),latest=r,ext2=dict(state=stage,message='SOFTWARE FIXTURE',used=False,manual_capture=False,capture_ms=None))
    def test_shaky_automatic_capture_nonzero_entry_true_click_across_refresh(self):
        from PySide6 import QtCore,QtTest
        w=self.window();sent=[];w.send_request=lambda kind,payload=None:sent.append((kind,payload))
        try:
            w.show();self.app.processEvents();snap=self.snap();w.update_controls(snap)
            self.assertTrue(w.ext2_begin.isEnabled());self.assertEqual(w.run_seconds.value(),0)
            QtTest.QTest.mousePress(w.ext2_begin,QtCore.Qt.MouseButton.LeftButton)
            for _ in range(4):w.update_controls(snap);self.app.processEvents()
            QtTest.QTest.mouseRelease(w.ext2_begin,QtCore.Qt.MouseButton.LeftButton);self.app.processEvents()
            self.assertEqual(sent,[('ext2_begin',{'amplitude_deg':90.0,'period_s':37.7})])
            self.assertEqual(w.preparation_selection()['motion_config'],CONFIG)
            self.assertEqual(w.preparation_selection()['motion_speed_internal_q16'],SPEED)
        finally:w.close()
    def test_exit_button_and_regular_target_restore(self):
        from PySide6 import QtCore,QtTest
        w=self.window();sent=[];w.send_request=lambda kind,payload=None:sent.append((kind,payload))
        try:
            w.show();self.app.processEvents();w.update_controls(self.snap('RUNNING',True))
            self.assertTrue(w.ext2_begin.isEnabled());self.assertFalse(w.target_action.isEnabled())
            exit_button=getattr(w,'ext2_exit',w.ext2_hold)
            self.assertTrue(exit_button.isVisible());self.assertTrue(exit_button.isEnabled())
            QtTest.QTest.mouseClick(exit_button,QtCore.Qt.MouseButton.LeftButton);self.app.processEvents()
            self.assertEqual(sent,[('ext2_exit',None)])
            w.update_controls(self.snap('EXITING',True));self.assertFalse(w.target_action.isEnabled())
            w.update_controls(self.snap());self.assertTrue(w.target_action.isEnabled());self.assertTrue(w.ext2_begin.isEnabled())
        finally:w.close()

def run():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=P/'build/r40_host');args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    files=[*sorted((P/'host').glob('*.py')),*sorted((P/'host').glob('*.json')),*sorted((P/'capture_ui').glob('*.py')),*sorted((P/'target_ui_fix').glob('*.py')),P/'calibration/encoder_calibration.json',Path(__file__).resolve()]
    before={str(p.relative_to(P)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]))
    changed=[n for n,h in before.items() if hashlib.sha256((P/n).read_bytes()).hexdigest()!=h]
    receipt=dict(status='PASS' if result.wasSuccessful() and not changed else 'FAIL',tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),source=str(P),source_sha256=before,changed_during_test=changed,physical_hardware=False,gowin=False)
    (args.out/'HOST_RECEIPT.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    return 0 if receipt['status']=='PASS' else 1
if __name__=='__main__':sys.exit(run())
