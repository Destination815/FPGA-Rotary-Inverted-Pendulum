import os,sys,json
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import launch
app=launch.studio.W.QApplication([]);w=launch.ExplainedStudio();w.timer.stop()
row=dict(theta=.1,dtheta=.4,phi=.05,dphi=.2,u=170,limit=180,state=3,fault=0,flags=31|1024|2048)
w.show_row(row);assert '18.0%' in w.cards['u'][1].text();assert '恢复升档中' in w.cards['state'][1].text()
row.update(u=0,limit=150,flags=14|2048|8192,state=0);w.show_row(row);assert '恢复曾触发' in w.cards['state'][1].text();assert '到位超时（仅提示）' in w.cards['state'][1].text()
row.update(theta=.01,dtheta=.1,phi=.01,dphi=.1,flags=31|4096,state=3);w.show_row(row);assert '实测稳定到位' in w.cards['state'][1].text()
w.show();app.processEvents();out=Path(__file__).resolve().parents[1]/'evidence/gui';out.mkdir(exist_ok=True);w.grab().save(str(out/'r10_status.png'));(out/'r10_status.json').write_text(json.dumps(dict(cases=3,status='PASS',serial_opened=False),indent=2));w.close();print('PASS 3 R10 dynamic-cap/status GUI scenarios')
