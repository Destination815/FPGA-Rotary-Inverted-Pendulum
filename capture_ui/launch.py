"""Integrated R40 UI explanation overlay and stable buttons."""
import json
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from protocol import CONTROL_BUILD
if CONTROL_BUILD!=0x4A070257:
    raise SystemExit('This UI entry is for R43 4A070257 only. Do not use in other builds.')
import studio
import studio_session
from poll_queue import PollFriendlyPump
studio_session.CommandPump=PollFriendlyPump
from diagnostics import text,explain
from identity_status import describe
from stable_buttons import button,StableButton
studio.button=button

class ExplainedStudio(studio.Studio):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(self.windowTitle()+" · 固定布局")
        def viewport(label,lines,name):
         parent=label.parentWidget();layout=parent.layout();area=studio.W.QScrollArea(parent);area.setObjectName(name)
         area.setFrameShape(studio.W.QFrame.Shape.NoFrame);area.setWidgetResizable(True)
         area.setHorizontalScrollBarPolicy(studio.QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
         area.setVerticalScrollBarPolicy(studio.QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
         area.setFixedHeight(label.fontMetrics().lineSpacing()*lines+12)
         area.setSizePolicy(studio.W.QSizePolicy.Policy.Expanding,studio.W.QSizePolicy.Policy.Fixed)
         item=layout.replaceWidget(label,area);assert item is not None
         label.setMinimumWidth(0);label.setSizePolicy(studio.W.QSizePolicy.Policy.Ignored,studio.W.QSizePolicy.Policy.Preferred)
         area.setWidget(label)
         return area
        self.capture_reason_viewport=viewport(self.capture_hint,8,'capture_reason_viewport')
        self.profile_reason_viewport=viewport(self.profile_status,3,'qualification_reason_viewport')
        self.cal_reason_viewport=viewport(self.cal_hint,3,'calibration_reason_viewport')
        self.identity_query=button('重新查询板端身份（不启动）',self.query_identity)
        area=self.profile_reason_viewport;layout=area.parentWidget().layout();layout.insertWidget(layout.indexOf(area)+1,self.identity_query)
        area.setFixedHeight(self.profile_status.fontMetrics().lineSpacing()*6+12)
        self.capture_reason_viewport.setToolTip('捕获条件实时更新；说明区高度固定，可在区内滚动查看完整原因。')

    def query_identity(self):
        self.send_request('command',{'name':'d02info','value':0})

    def update_controls(self,snapshot):
        names=('cal_button','clear_button','arm_button','capture_button','start_button',
               'target_button','trajectory_on','trajectory_off')
        controls=[getattr(self,name) for name in names if isinstance(getattr(self,name,None),StableButton)]
        for control in controls:control.begin_enabled_update()
        failed=True
        try:
            super().update_controls(snapshot)
            failed=False
        finally:
            for control in controls:control.finish_enabled_update(failed)

        if hasattr(self,'identity_query'):
            code,message=describe(snapshot)
            self.profile_status.setText(message+'\n'+snapshot.get('qualification_note',''))
            preparation=snapshot.get('_preparation_issue','')
            self.profile_status.setText(self.profile_status.text()+'\n'+('运动准备：'+preparation if preparation else '运动准备：板端版本、行程与数字速度回读一致'))
            self.identity_query.setEnabled(bool(snapshot.get('connected') and snapshot.get('latest') and time.monotonic()-snapshot.get('last_rx',0)<.5 and not snapshot.get('busy') and not snapshot.get('d02_busy') and not self.replay))
        row=snapshot.get('latest') or {}
        checked=self.ready_check.isChecked()
        message=text(snapshot,checked,bool(self.replay))
        # Events are diagnostic only. Never infer armed=True from a successful click.
        for event in snapshot.get('events',[]):
            if (event.get('event') in ('send','ack','timeout','queued_after_info') and event.get('name')=='arm') or (event.get('event') in ('request_rejected','batch_failed')):
                self._last_arm_event=event
        event=getattr(self,'_last_arm_event',{})
        if event:
            kind=event.get('event')
            if kind=='ack':
                note='使能ACK：'+('已接受；以板端使能状态为准' if event.get('status')==0 else f"被板端拒绝，状态码 {event.get('status')}")
            elif kind=='send':note='使能命令已发出，等待板端ACK'
            elif kind=='queued_after_info':note='使能点击已保留，等待后台查询结束（最多300ms）'
            elif kind=='timeout':note='使能ACK超时；已请求停止'
            else:note='最近操作提示：'+str(event.get('reason',''))
            message+='\n'+note
        if row:
            message+=f"\n{'会话使能（非电机输出）' if row.get('state')==7 else '板端使能'}：{'是' if int(row.get('flags',0))&1 else '否'}；flags={int(row.get('flags',0))}"

        if not self.capture_button.isEnabled() and not explain(snapshot,checked,bool(self.replay)):
            message='原界面仍锁定捕获；请导出日志核对。\n'+message
        self.capture_hint.setText(message)
        rect=studio.QtCore.QRect(0,0,max(220,self.capture_hint.width()),10000)
        height=self.capture_hint.fontMetrics().boundingRect(rect,studio.QtCore.Qt.TextFlag.TextWordWrap,message).height()
        self.capture_hint.setFixedHeight(height+12)
        self.capture_hint.setAlignment(studio.QtCore.Qt.AlignmentFlag.AlignTop | studio.QtCore.Qt.AlignmentFlag.AlignLeft)
        self.capture_button.setToolTip(message)
        armed=bool(int(row.get('flags',0))&1)
        self.arm_button.setText('自动会话进行中（零输出等待）' if row.get('state')==7 else '已使能（等待局部捕获）' if armed else '① 使能会话（不会启动运动）')
        self.capture_button.setText('② 捕获并开始计时')
        self.arm_button.setToolTip(snapshot.get('_preparation_issue') or '校准后点击此按钮；收到板端已使能状态后，再点击捕获。勾选现场确认不等于使能。')
        self.start_button.setToolTip(snapshot.get('_preparation_issue') or '在有效下垂测量且已使能时点击；不会自动启动。')
        # Keep a small rolling local record for an exact explanation of intermittent lockouts.
        now=time.monotonic()
        if snapshot.get('connected') and now-getattr(self,'_diagnostic_saved',0)>=1:
            self._diagnostic_saved=now
            record=dict(time=time.time(),capture_enabled=self.capture_button.isEnabled(),
                        arm_enabled=self.arm_button.isEnabled(),checked=checked,
                        reasons=explain(snapshot,checked,bool(self.replay)),latest=row,
                        board=snapshot.get('d02_info'),board_fresh=snapshot.get('d02_fresh'),
                        busy=snapshot.get('busy'),qualification=snapshot.get('qualification_note'),last_arm_event=getattr(self,'_last_arm_event',{}))
            try:
                folder=ROOT/'r20_ui'/'logs';folder.mkdir(parents=True,exist_ok=True)
                log=folder/'button_diagnostics.jsonl'
                if log.exists() and log.stat().st_size>2_000_000:
                    log.replace(folder/'button_diagnostics.previous.jsonl')
                with log.open('a',encoding='utf-8') as f:f.write(json.dumps(record,ensure_ascii=False)+'\n')
            except (OSError,TypeError):
                self.capture_hint.setText(message+'\n按钮诊断文件保存失败；仍可使用原“导出实验包”。')

studio.Studio=ExplainedStudio
if __name__=='__main__':raise SystemExit(studio.main())
