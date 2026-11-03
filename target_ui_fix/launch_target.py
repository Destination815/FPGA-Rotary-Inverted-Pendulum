"""Integrated R38T1 target UI: explicit click validation, no automatic motion."""
import sys,time,math,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'capture_ui'))
import launch as base
from motion_config import pack,unpack,validate,PERIODS
from coordinates import SCALE
from r01_gate import check,board_ok
from studio_session import permitted
W=base.studio.W
F_CONFIG=pack(190*SCALE,15*SCALE,1,20)

# Recheck readiness at actual send, including a command queued behind INFO.
_original_gate=base.studio_session.SerialSession.command_gate
def target_send_gate(self,req):
    ok,why=_original_gate(self,req)
    if ok and req.name=='target' and not (int(self.pump.latest.get('flags',0))&128):
        return False,'板端参考仍在运动或目标请求尚未处理；本次目标未发送'
    return ok,why
base.studio_session.SerialSession.command_gate=target_send_gate

class TargetPump(base.PollFriendlyPump):
    def submit(self,requests,label):
        requests=list(requests)
        if (len(requests)==1 and requests[0].name=='motioncfg' and self.busy
                and self.batch=='version_poll' and self.pending
                and self.pending['request'].name=='d02info' and not self.todo):
            if not self.fresh:raise ValueError('没有新鲜遥测')
            self.todo.extend(requests);self.batch=label;self.manual_wait_until=self.clock()+.3
            self.emit(dict(event='queued_after_info',name='motioncfg',deadline_ms=300))
            return
        return super().submit(requests,label)
base.studio_session.CommandPump=TargetPump

class TargetStudio(base.ExplainedStudio):
    def __init__(self):
        self._target_snapshot={};self._target_feedback='尚未发送目标'
        super().__init__()
        self.setWindowTitle(self.windowTitle()+' · 定位与配置')
        # Separate action button: the old refresh code cannot cancel its press.
        self.target_action=W.QPushButton('设置目标（检查后发送）')
        self.target_action.clicked.connect(self.send_target)
        parent=self.target_button.parentWidget();layout=parent.layout()
        old=layout.replaceWidget(self.target_button,self.target_action)
        assert old is not None
        self.target_button.hide()
        self.f_prepare=W.QPushButton('准备本次运行（使用上方用户配置）')
        self.f_prepare.clicked.connect(self.prepare_f)
        self.config_sync=W.QPushButton('将界面选项同步为板端配置（不运动）')
        self.config_sync.clicked.connect(self.sync_config)
        self.target_status=W.QPlainTextEdit();self.target_status.setReadOnly(True)
        self.target_status.setFixedHeight(118)
        # Place feedback directly under configuration, above target row.
        idx=layout.indexOf(self.motion_readback)+1
        for widget in (self.f_prepare,self.config_sync,self.target_status):
            layout.insertWidget(idx,widget);idx+=1
        self.target_action.setEnabled(False);self.f_prepare.setEnabled(False)
        self.f_prepare.setVisible(False)
        self.config_sync.setEnabled(False)
        for widget,signal in ((self.motion_workspace,'valueChanged'),(self.motion_amplitude,'valueChanged'),(self.motion_speed,'valueChanged'),(self.motion_period,'currentIndexChanged'),(self.motion_range_check,'toggled')):
            getattr(widget,signal).connect(self.publish_preparation)
        self.build_four_group_ui()

    def build_four_group_ui(self):
        self.setWindowTitle(self.windowTitle()+' · R43 四组测试')
        self.motion_apply.setText('应用本组速度并准备')
        group=self.motion_apply.parentWidget();group.setTitle('02  选速度 → 准备 → 起摆 → 定位')
        layout=group.layout()
        presets=W.QWidget();buttons=W.QGridLayout(presets);buttons.setContentsMargins(0,0,0,0)
        self.speed_presets=[]
        for number,speed in enumerate((15,30,45,60),1):
            b=W.QPushButton(f'第{number}组 {speed}°/s')
            b.clicked.connect(lambda checked=False,v=speed:self.select_test_speed(v))
            buttons.addWidget(b,(number-1)//2,(number-1)%2);self.speed_presets.append(b)
        layout.insertWidget(layout.indexOf(self.motion_apply),presets)
        self.swing_begin_button=W.QPushButton('使能并起摆')
        self.swing_begin_button.clicked.connect(self.begin_swing)
        layout.insertWidget(layout.indexOf(self.motion_readback)+1,self.swing_begin_button)
        targets=W.QWidget();row=W.QHBoxLayout(targets);row.setContentsMargins(0,0,0,0)
        self.target_presets=[]
        for value,text in ((90,'+90°'),(-90,'−90°'),(0,'回到0°')):
            b=W.QPushButton(text);b.clicked.connect(lambda checked=False,v=value:self.quick_target(v))
            row.addWidget(b);self.target_presets.append(b)
        layout.insertWidget(layout.indexOf(self.swing_begin_button)+1,targets)
        self.simple_status=W.QLabel();self.simple_status.setWordWrap(True)
        layout.insertWidget(layout.indexOf(targets)+1,self.simple_status)
        self.advanced_toggle=W.QCheckBox('更多操作：手动捕获 / 普通往返')
        layout.addWidget(self.advanced_toggle)
        form=next(layout.itemAt(i).layout() for i in range(layout.count()) if isinstance(layout.itemAt(i).layout(),W.QFormLayout))
        self.advanced_widgets=[self.arm_button,self.start_button,self.run_seconds,
            self.capture_button.parentWidget(),self.motion_amplitude,self.motion_period,
            form.labelForField(self.motion_amplitude),form.labelForField(self.motion_period),
            self.target,self.target_action,self.trajectory_on,self.trajectory_off,
            self.config_sync,self.target_status,self.motion_workspace,form.labelForField(self.motion_workspace),self.identity_query]
        self.advanced_widgets += [label for label in group.findChildren(W.QLabel) if label.text().startswith(('沿用本队','先准备'))]
        self.profile_reason_viewport.setFixedHeight(64);self.cal_reason_viewport.setFixedHeight(40)
        self.sidebar.setMaximumWidth(420)
        self.swing_begin_button.setToolTip('放杆下垂、撤手后点击；自动完成使能与启动。')
        self.advanced_toggle.toggled.connect(self.show_advanced)
        self.show_advanced(False)
        self.sine_amplitude=W.QDoubleSpinBox();self.sine_amplitude.setRange(.01,360*SCALE);self.sine_amplitude.setDecimals(2);self.sine_amplitude.setSuffix(' °');self.sine_amplitude.setValue(90)
        self.sine_period=W.QDoubleSpinBox();self.sine_period.setRange(1,3600);self.sine_period.setDecimals(2);self.sine_period.setSuffix(' 秒')
        sine_form=W.QFormLayout();sine_form.addRow('正弦半幅',self.sine_amplitude);sine_form.addRow('正弦周期',self.sine_period)
        self.ext2_begin.parentWidget().layout().insertLayout(0,sine_form)
        recommend=W.QPushButton('按本组速度计算周期');recommend.clicked.connect(self.recommend_sine_period)
        self.ext2_begin.parentWidget().layout().insertWidget(1,recommend)
        self.sine_readback=W.QLabel('板端正弦参数：等待回读');self.sine_readback.setWordWrap(True)
        self.ext2_begin.parentWidget().layout().insertWidget(2,self.sine_readback)
        self.ext2_begin.clicked.disconnect();self.ext2_begin.clicked.connect(self.begin_sine)
        self.ext2_begin.setText('应用并启动正弦')
        self.ext2_hold.setText('退出正弦并保持')
        self.target_action.setText('设置任意目标')
        self.trajectory_on.setText('普通往返');self.trajectory_off.setText('结束普通往返')
        self.config_sync.setText('同步板端配置')
        self.ext2_begin.parentWidget().setTitle('03  正弦运动（半幅、周期可修改）')
        self.ext2_status.setMinimumHeight(48)
        for label in self.ext2_begin.parentWidget().findChildren(W.QLabel):
            if label.text().startswith('统一准备'):label.hide()
        self.motion_readback.setFixedHeight(50);self.motion_readback.setWordWrap(True)
        self.select_test_speed(15)

    def recommend_sine_period(self):
        from sine_config import recommended_period
        self.sine_amplitude.interpretText();self.motion_speed.interpretText()
        self.sine_period.setValue(recommended_period(self.sine_amplitude.value(),self.motion_speed.value()))

    def select_test_speed(self,value):
        self.motion_speed.setValue(value)
        self.recommend_sine_period()

    def begin_sine(self):
        self.sine_amplitude.interpretText();self.sine_period.interpretText()
        self.publish_preparation()
        self.send_request('ext2_begin',dict(amplitude_deg=self.sine_amplitude.value(),period_s=self.sine_period.value()))

    def show_advanced(self,shown):
        for widget in self.advanced_widgets:widget.setVisible(shown)

    def apply_motion_config(self):
        self.motion_speed.interpretText()
        self.motion_workspace.interpretText();self.motion_amplitude.interpretText()
        self.experiment_name.setText(f'R43 速度{self.motion_speed.value():g} 四组流程')
        self.add_note()
        return super().apply_motion_config()

    def begin_swing(self):
        self.publish_preparation()
        why=self.preparation_issue(self._target_snapshot)
        if why:self.feedback('未启动：'+why);return
        self.send_request('swing_begin')

    def quick_target(self,value):
        self.target.setValue(value);self.send_target()

    def update_four_group_ui(self,snapshot):
        if not hasattr(self,'simple_status'):return
        r=snapshot.get('latest') or {};info=snapshot.get('d02_info') or {};ext=snapshot.get('ext2') or {}
        fresh=bool(snapshot.get('connected') and time.monotonic()-snapshot.get('last_rx',0)<.5 and not self.replay)
        qualified=bool(snapshot.get('qualification_ok') and board_ok(info,snapshot.get('d02_fresh',False)))
        stage=ext.get('state','READY');active=stage not in ('READY',None) or bool(info.get('ext2_prepared'))
        self.export_button.setText('结束本组并导出')
        if not fresh:mode='等待连接和实时状态'
        elif r.get('fault'):mode=f"故障 {r['fault']}：已停止；保存本组日志"
        elif info.get('ext2_prepared') and snapshot.get('d02_fresh') and int(r.get('flags',0))&256:
            mode='正弦运行中（板端已确认）' if stage not in ('EXIT_READY','EXIT_ACK','EXITING') else '正弦正在退出并回到入口中心'
        elif stage in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK'):mode='正弦未启动：正在自动关闭普通往返'
        elif stage=='WAIT_HOLD':mode='正弦未启动：等待捕获或定位参考结束'
        elif stage in ('WAIT_ACK','WAIT_FLAG'):mode='正弦未确认：等待板端启动回读'
        elif r.get('state') in (1,2,6,7):mode='正在起摆／恢复／捕获'
        elif ext.get('ordinary_active'):mode='普通往返运行中，尚未进入正弦'
        elif r.get('state') in (3,4):mode='定位／保持'
        else:mode='待机'
        self.ext2_status.setText(mode+'\n'+ext.get('message',''))
        if stage.startswith('CONFIG_'):self.ext2_status.setText('正弦尚未启动：正在写入并核对半幅、周期')
        a,step=info.get('sine_amplitude_internal_q16'),info.get('sine_phase_step')
        if info.get('sine_settings_fresh') and isinstance(a,int) and isinstance(step,int) and step>0:
            aa=math.degrees(a/65536*SCALE);tt=2**32/(1000*step)
            self.sine_readback.setText(f'板端：半幅 ±{aa:.2f}°；周期 {tt:.3f}秒；峰值速度 {2*math.pi*aa/tt:.2f}°/s')
        else:self.sine_readback.setText('板端正弦参数：等待回读')
        from ext2_sequence import capture_row
        sine_available=bool(fresh and qualified and not snapshot.get('busy') and not snapshot.get('finalizing') and not self.preparation_issue(snapshot) and capture_row(r) and stage in ('READY','RUNNING'))
        self.ext2_begin.setEnabled(sine_available)
        self.ext2_begin.setText('应用新参数并继续' if stage=='RUNNING' else '应用并启动正弦')
        speed=info.get('motion_speed_internal_q16')
        actual=f'{math.degrees(speed/65536*SCALE):.3f}°/s' if isinstance(speed,int) and speed>0 else '未回读'
        issue=self.preparation_issue(snapshot)
        self.motion_readback.setText(f'本组定位速度：{self.motion_speed.value():g}°/s；板端：{actual}\n'+('待准备：'+issue if issue else '本组准备完成'))
        self.simple_status.setText(self._target_feedback)
        allow=bool(fresh and qualified and not snapshot.get('busy') and not snapshot.get('finalizing'))
        start_row=dict(r,flags=int(r.get('flags',0))|1)
        self.swing_begin_button.setEnabled(bool(allow and not active and not issue and permitted('start',start_row,True)))
        for b in self.speed_presets:b.setEnabled(bool(allow and r.get('state')==0 and not int(r.get('flags',0))&1))
        for b in self.target_presets:b.setEnabled(bool(fresh and qualified and not active and not snapshot.get('finalizing')))

    def selected_config(self):
        return pack(self.motion_workspace.value(),self.motion_amplitude.value(),
                    2,PERIODS[self.motion_period.currentIndex()],recovery=self.full_turn_recovery.isChecked())

    def reason(self,name='target',value=None):
        s=self._target_snapshot;r=s.get('latest',{});info=s.get('d02_info',{})
        if s.get('ext2',{}).get('state') in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK','CONFIG_ACK','WAIT_HOLD','WAIT_ACK','WAIT_FLAG','RUNNING','EXIT_READY','EXIT_ACK','EXITING'):return '先点击退出正弦，随后可直接继续定位，无需重新捕获'
        if self.replay:return '当前为回放，不能发送'
        fresh=s.get('connected') and time.monotonic()-s.get('last_rx',0)<.5
        if not fresh:return '没有新鲜遥测；检查连接'
        if s.get('d02_busy'):return '正在读取诊断缓存；请等读取结束'
        if value is None:value=round(math.radians(self.target.value())*65536)
        ok,why=check(name,value,r,info,s.get('d02_fresh',False),s.get('qualification_ok',False))
        if not ok:return why
        if not permitted(name,r,fresh):
            if r.get('fault'):return f"板端故障 {r['fault']}；停止后处理故障"
            if name=='motioncfg':return '定位预置需要停止输出、未使能、无故障'
            return f"先使能并捕获，等进入保持；当前 state={r.get('state')}、flags={r.get('flags')}"
        if name=='target':
            why=self.preparation_issue(s)
            if why:return why
            if not int(r.get('flags',0))&128:return '板端参考尚未静止；等待现有运动结束（不是要求摆杆角度完全不变）'
            try:chosen=self.selected_config()
            except ValueError as e:return str(e)
            if chosen!=info.get('motion_config'):
                return '界面与板端配置不一致；停止并应用所选合法配置，等待四项回读一致'
        return ''

    def feedback(self,message):
        self._target_feedback=message
        self.alert(message)
        self.render_reason()
        try:
            p=ROOT/'r20_ui'/'logs';p.mkdir(parents=True,exist_ok=True)
            s=self._target_snapshot
            with (p/'target_actions.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(dict(time=time.time(),message=message,latest=s.get('latest'),info=s.get('d02_info'),busy=s.get('busy')),ensure_ascii=False)+'\n')
        except OSError:pass

    def send_target(self):
        self.target.interpretText()
        value=round(math.radians(self.target.value())*65536)
        why=self.reason(value=value)
        if why:self.feedback('未发送：'+why);return
        self.command('target',value)
        self.feedback(f'已提交目标 {self.target.value():+.2f}°；等待板端ACK，尚不代表已经运动')

    def set_config_fields(self,cfg):
        if cfg==75082174:
            self.ext2_selected=True;self.publish_preparation();return
        self.ext2_selected=False
        c=unpack(cfg)
        self.motion_workspace.setValue(c['workspace_deg']);self.motion_amplitude.setValue(c['amplitude_deg'])
        self.motion_speed.setValue(c['speed_deg_s']);self.motion_period.setCurrentIndex(PERIODS.index(c['period_s']));self.full_turn_recovery.setChecked(c['full_turn_recovery'])

    def sync_config(self):
        s=self._target_snapshot;cfg=s.get('d02_info',{}).get('motion_config')
        if not board_ok(s.get('d02_info',{}),s.get('d02_fresh',False)) or cfg is None or not validate(cfg)[0]:
            self.feedback('未同步：没有有效的新鲜板端配置');return
        self.set_config_fields(cfg)
        speed=s.get('d02_info',{}).get('motion_speed_internal_q16')
        if isinstance(speed,int) and speed>0:self.motion_speed.setValue(math.degrees(speed/65536*SCALE))
        self.feedback('仅同步界面选项；未发送运动或配置命令。物理速度以板端回读为准。')

    def prepare_f(self):
        self.apply_motion_config()

    def render_reason(self):
        if not hasattr(self,'target_status'):return
        reason=self.reason()
        preparation=self.preparation_issue(self._target_snapshot)
        message=('运动准备：'+preparation if preparation else '运动准备完成：板端配置、行程与数字速度回读一致')+'\n'+('定位可发送' if not reason else '定位条件：'+reason)+'\n'+self._target_feedback
        if self.target_status.toPlainText()!=message:self.target_status.setPlainText(message)

    def update_controls(self,snapshot):
        self._target_snapshot=snapshot
        super().update_controls(snapshot)
        if not hasattr(self,'target_action'):return
        # Clicking is available for diagnosis; the action and worker both validate.
        clickable=bool(snapshot.get('connected') and not self.replay)
        ext2=snapshot.get('ext2',{}).get('state') in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK','CONFIG_ACK','WAIT_HOLD','WAIT_ACK','WAIT_FLAG','RUNNING','EXIT_READY','EXIT_ACK','EXITING') or snapshot.get('d02_info',{}).get('ext2_prepared',False)
        self.target_action.setEnabled(clickable and not ext2)
        self.f_prepare.setEnabled(clickable);self.config_sync.setEnabled(clickable)
        for e in snapshot.get('events',[]):
            name=e.get('name');kind=e.get('event')
            if name in ('target','motioncfg') and kind in ('send','ack','timeout','queued_after_info'):
                if kind=='ack':self._target_feedback=f"{name} ACK："+('板端已接受' if e.get('status')==0 else f"板端拒绝，状态码{e.get('status')}")
                elif kind=='timeout':self._target_feedback=f'{name} 确认超时；请查看停止状态'
                elif kind=='send':self._target_feedback=f'{name} 已实际发送，等待ACK'
                else:self._target_feedback='点击已排队，等待后台查询结束（最多300ms）'
            elif kind in ('request_rejected','batch_failed'):
                self._target_feedback='操作未完成：'+str(e.get('reason','请看操作记录'))
        self.render_reason()
        self.update_four_group_ui(snapshot)

base.studio.Studio=TargetStudio
if __name__=='__main__':raise SystemExit(base.studio.main())
