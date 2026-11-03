import copy
import json
import math
import os
from pathlib import Path
import sys
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import launch
from diagnostics import explain
app=launch.studio.W.QApplication([])
window=launch.ExplainedStudio()
window.timer.stop()
window.ready_check.setChecked(True)
base=dict(connected=True,last_rx=time.monotonic(),qualification_ok=True,expected_bitstreams=['a'*64],qualification_note='测试资格快照（不连接设备）',
          d02_fresh=True,d02_info=dict(build=0x4A070250,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=42505918),busy=False,
          latest=dict(flags=15,state=0,fault=0,theta=math.radians(-.19),dtheta=-.026,dphi=0,phi=0,adc=746,zero=746,reference=0))
cases=[('ready',{},True,''),('unarmed',{'flags':14},False,'尚未使能'),('not_calibrated',{'flags':13},False,'尚未校准'),
       ('invalid_measurement',{'flags':11},False,'测量无效'),('angle',{'theta':math.radians(9)},False,'摆杆角度超限'),
       ('angular_speed',{'dtheta':.701},False,'摆杆角速度超限'),('arm_speed',{'dphi':1.01},False,'摆臂角速度超限'),
       ('state',{'state':3},False,'未处于 IDLE'),('fault',{'fault':1},False,'故障'),
       ('reference_not_stationary',{'flags':15,'ref_velocity':.4},True,''),
       ('nominal_threshold',{'theta':math.radians(1),'dtheta':.2,'dphi':.99},True,'')]
window.firmware_info={'sha256':'a'*64};window.motion_range_check.setChecked(True)
records=[]
for name,updates,enabled,reason in cases:
    s=copy.deepcopy(base);s['latest'].update(updates);s['last_rx']=time.monotonic()
    launch.studio.Studio.__mro__[1].update_controls(window,s)
    original=window.capture_button.isEnabled()
    window.update_controls(s)
    assert window.capture_button.isEnabled()==original==enabled,name
    assert reason in window.capture_hint.text(),(name,window.capture_hint.text())
    records.append(dict(case=name,enabled=enabled,original_gate_unchanged=True))
for name,updates,reason in [('busy',{'busy':True},'等待命令'),('old_data',{'last_rx':0},'遥测过期'),('qualification',{'qualification_ok':False},'资格未通过'),('old_board',{'d02_fresh':False},'标识未核对')]:
    s=copy.deepcopy(base);s['last_rx']=time.monotonic();s.update(updates)
    window.update_controls(s)
    assert not window.capture_button.isEnabled(),name
    assert reason in window.capture_hint.text(),name
    records.append(dict(case=name,enabled=False))
s=copy.deepcopy(base);s['last_rx']=time.monotonic();window.ready_check.setChecked(False);window.update_controls(s)
assert not window.capture_button.isEnabled() and '先勾选' in window.capture_hint.text()
records.append(dict(case='unchecked',enabled=False))
window.ready_check.setChecked(True)
window.show();app.processEvents();s['last_rx']=time.monotonic();s['latest']['flags']=14
window.update_controls(s)
app.processEvents()
output=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/"evidence/gui";output.mkdir(parents=True,exist_ok=True)
window.grab().save(str(output/'unarmed_ui.png'))
(output/'ui_tests.json').write_text(json.dumps(records,indent=2,ensure_ascii=False))
print('PASS',len(records),'UI cases; no serial connection, command gates unchanged')
window.close()
