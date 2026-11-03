import copy,sys,time
from pathlib import Path
from PySide6 import QtWidgets,QtTest,QtCore
import launch
app=QtWidgets.QApplication([]);w=launch.ExplainedStudio();w.timer.stop();w.ready_check.setChecked(True)
snap=dict(connected=True,last_rx=time.monotonic(),qualification_ok=True,expected_bitstreams=['a'*64],qualification_note='TEST ONLY',d02_fresh=True,d02_info=dict(build=0x4A070250,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=42505918),latest=dict(flags=14,state=0,fault=0,theta=0,dtheta=0,dphi=0,phi=0,adc=746,zero=746,reference=0))
w.firmware_info={'sha256':'a'*64};w.motion_range_check.setChecked(True)
sent=[];w.send_request=lambda kind,payload=None:sent.append((kind,payload))
w.show();app.processEvents()
def exercise(widget,s,updates=None):
 s['last_rx']=time.monotonic();w.update_controls(s);app.processEvents()
 QtTest.QTest.mousePress(widget,QtCore.Qt.MouseButton.LeftButton)
 for i in range(3):
  QtTest.QTest.qWait(40);s['last_rx']=time.monotonic()
  if updates:s['latest'].update(updates)
  w.update_controls(s)
 QtTest.QTest.mouseRelease(widget,QtCore.Qt.MouseButton.LeftButton)
# Real clicked signal invokes original Studio.command and its ARM magic value.
exercise(w.arm_button,copy.deepcopy(snap));assert len(sent)==1 and sent[0][1]['name']=='arm' and sent[0][1]['value']==0x4a433131,sent
s=copy.deepcopy(snap);s['latest']['flags']=15
exercise(w.capture_button,s);assert len(sent)==2 and sent[1][1]['name']=='capture',sent
# A genuine final-gate failure while pressed must still cancel click.
s=copy.deepcopy(snap);s['latest']['flags']=15
exercise(w.capture_button,s,{'dtheta':.8});assert len(sent)==2,sent
s=copy.deepcopy(snap)
exercise(w.arm_button,s,{'fault':1});assert len(sent)==2,sent
# Ordinary mouse clicks also still work, and no motion is sent just by refresh.
s=copy.deepcopy(snap);s['last_rx']=time.monotonic();w.update_controls(s)
QtTest.QTest.mouseClick(w.arm_button,QtCore.Qt.MouseButton.LeftButton)
assert len(sent)==3 and sent[-1][1]['name']=='arm'
for _ in range(10):w.update_controls(s)
assert len(sent)==3
w.close()
print('PASS 6 mouse/signal scenarios: 120ms hold across 3 refreshes preserves ARM/CAPTURE; actual unsafe changes cancel; no automatic sends')
