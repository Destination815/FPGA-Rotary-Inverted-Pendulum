import os,time,math,json
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import launch
from PySide6 import QtTest,QtCore
app=launch.studio.W.QApplication([]);w=launch.ExplainedStudio();w.timer.stop();w.ready_check.setChecked(True);sent=[];w.send_request=lambda kind,payload=None:sent.append((kind,payload))
s=dict(connected=True,last_rx=time.monotonic(),qualification_ok=True,expected_bitstreams=['a'*64],qualification_note='SIMULATED',d02_fresh=True,d02_info=dict(build=0x4A070250,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=41981630),latest=dict(flags=15|128,state=3,fault=0,theta=0,dtheta=0,dphi=0,phi=0,adc=746,zero=746,reference=0))
w.firmware_info={'sha256':'a'*64};w.motion_range_check.setChecked(True)
w.show();app.processEvents();w.update_controls(s)
assert not w.target_button.isEnabled() and not w.trajectory_on.isEnabled()
assert '未应用' in w.motion_readback.text()
s['d02_info']['motion_config'] |= 1<<19
w.update_controls(s)
assert w.target_button.isEnabled() and w.trajectory_on.isEnabled() and not w.start_button.isEnabled()
w.target.setValue(3);w.update_controls(s)
QtTest.QTest.mousePress(w.target_button,QtCore.Qt.MouseButton.LeftButton)
for _ in range(4):
 QtTest.QTest.qWait(40);s['last_rx']=time.monotonic();w.update_controls(s)
QtTest.QTest.mouseRelease(w.target_button,QtCore.Qt.MouseButton.LeftButton)
assert sent[-1][1]=={'name':'target','value':round(math.radians(3)*65536),'seconds':0,'expected_motion_config':s['d02_info']['motion_config']},sent
s['latest']['flags']=15|256;s['latest']['state']=4;w.update_controls(s)
assert not w.target_button.isEnabled() and not w.trajectory_on.isEnabled() and w.trajectory_off.isEnabled()
QtTest.QTest.mouseClick(w.trajectory_off,QtCore.Qt.MouseButton.LeftButton);assert sent[-1][1]=={'name':'trajectory','value':0,'seconds':0}
s['qualification_ok']=False;w.update_controls(s);assert not w.trajectory_off.isEnabled()
s['qualification_ok']=True;s['d02_info']['build']-=1;w.update_controls(s);assert not w.target_button.isEnabled();assert f"{s['d02_info']['build']:08X}" in w.profile_status.text()
assert w.target.minimum()==-180 and w.target.maximum()==180
s['latest'].update(state=0,theta=3.1,flags=15);s['d02_info']['build']+=1;w.update_controls(s);assert w.start_button.isEnabled()
out=Path(__file__).resolve().parents[1]/'evidence/gui';out.mkdir(exist_ok=True);w.grab().save(str(out/'r13_motion_ui.png'));(out/'motion_ui.json').write_text(json.dumps(dict(status='PASS',cases=7,serial_opened=False),indent=2));w.close();print('PASS 7 motion GUI boundary/real click/identity cases')
