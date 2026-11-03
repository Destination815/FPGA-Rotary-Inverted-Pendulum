import os,sys,time,copy,json,importlib.util
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='offscreen'
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'capture_ui'));import launch as original
from types import SimpleNamespace
m=SimpleNamespace(original=original,W=original.studio.W)
a=m.W.QApplication([]);w=original.ExplainedStudio();w.timer.stop();w.ready_check.setChecked(True);w.show();a.processEvents();build=original.CONTROL_BUILD
base=dict(connected=True,last_rx=time.monotonic(),qualification_ok=True,expected_bitstreams=['a'*64],d02_fresh=True,d02_info=dict(build=build,channel=1,motor_sign=-1,coordinate_verified=True,motion_config=42505918),busy=False,latest=dict(flags=15,state=0,fault=0,theta=0.,dtheta=0.,dphi=0,phi=0,adc=746,zero=746,reference=0))
w.firmware_info={'sha256':'a'*64};w.motion_range_check.setChecked(True)
records=[]
for width in [1280,1600,1920]:
 w.resize(width,900);a.processEvents();geo=[];gates=[]
 for i in range(240):
  s=copy.deepcopy(base);s['last_rx']=time.monotonic()
  if i%3==1:s['latest'].update(theta=.3,dtheta=2.,flags=0,state=3,fault=4);s.update(qualification_ok=False,qualification_note='多项原因长文本用于检查换行，不改变真实设备状态。'*3,connected=False,d02_fresh=False,busy=True)
  if i%3==2:s['latest'].update(theta=-.0001,dtheta=-.00001)
  w.update_controls(s);a.processEvents();r=w.capture_reason_viewport.parentWidget().geometry();b=w.capture_button.mapTo(w,m.original.studio.QtCore.QPoint(0,0));geo.append((r.height(),b.x(),b.y()));gates.append(w.capture_button.isEnabled())
  assert gates[-1]==(i%3!=1)
 assert len(set(geo[10:]))==1,(width,set(geo[10:]))
 records.append(dict(width=width,refreshes=240,geometry_variants=len(set(geo[10:])),command_gates_unchanged=True))
rootout=root/'evidence/gui';rootout.mkdir(exist_ok=True);w.grab().save(str(rootout/'layout.png'));(rootout/('layout_'+hex(build)+'.json')).write_text(json.dumps(records,indent=2));w.close();print('PASS',records)
