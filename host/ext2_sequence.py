"""R40 worker-owned seamless Extension2 requests; one captured session.

The FPGA keeps the ordinary configuration and observer origin. Entry may wait
for CAPTURE or a position move to finish. No measured-stillness/time gate and
no automatic ARM/CAPTURE/STOP/reconfiguration are inserted by this helper.
"""
import math
import time
from coordinates import BUILD,SCALE
from motion_config import EXT2_CONFIG,unpack_internal
from sine_config import settings as make_settings,confirmed

def delta_ms(now,old):return (int(now)-int(old))&0xffffffff

def capture_row(row):
    fields=('theta','dtheta','phi','dphi','reference','ms')
    if any(not isinstance(row.get(k),(int,float)) or isinstance(row[k],bool) or not math.isfinite(row[k]) for k in fields):return False
    flags=int(row.get('flags',0))
    return bool(row.get('state') in (2,3,4) and row.get('fault',1)==0 and flags&7==7 and flags&16 and not flags&(64|512))

def hold_row(row):
    return capture_row(row) and row['state'] in (3,4) and bool(int(row['flags'])&128) and not int(row['flags'])&256

def operator_admission(row):
    ok=hold_row(row)
    return dict(schema='j280.r40.ext2-entry.v1',ok=ok,reason='' if ok else '等待本次捕获或定位参考结束；无需重新捕获或等待摆杆静稳',stability_gate=False,history_required=False)

class Ext2Sequence:
    def __init__(self,clock=time.monotonic,**_):
        self.clock=clock;self.state='READY';self.capture_ms=None;self.session_ext2=False
        self.used=False;self.movement_sent=False;self.pending_seq=None;self.actions=[];self.events=[]
        self.ordinary_active=False;self.trajectory_pending=None
        self.settings=None;self.settings_written=False;self.restart_pending=False
        self.deadline=None;self.message='完成起摆/捕获后可直接进入；以本轮定位目标为中心，未设置目标时使用首次进入参考往返，退出后继续定位。'
    @property
    def active(self):return self.state in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK','CONFIG_ACK','WAIT_HOLD','WAIT_ACK','WAIT_FLAG','RUNNING','EXIT_READY','EXIT_ACK','EXITING')
    def log(self,event,**data):self.events.append(dict(event=event,ext2_stage=self.state,**data))
    def drain(self):a,e=self.actions,self.events;self.actions=[];self.events=[];return a,e
    def normalize_event(self,event):return event
    def abort(self,why,stop=False):
        self.restart_pending=False;self.state='READY';self.actions=[];self.pending_seq=None;self.deadline=None;self.message=why
        self.log('ext2_cancelled',reason=why)
        if stop:self.actions.append(('stop',0))
    def event(self,event):
        kind,name=event.get('event'),event.get('name')
        if kind=='send' and name in ('sine_amplitude','sine_period') and self.state=='CONFIG_ACK':self.pending_seq=event['seq']
        if kind=='ack' and name in ('sine_amplitude','sine_period') and self.state=='CONFIG_ACK':
            if event.get('seq')!=self.pending_seq:return
            if event.get('status'):
                self.abort('正弦参数未被板端接受，未启动；请查看参数/行程');return
            self.state='CONFIG_PERIOD' if name=='sine_amplitude' else 'CONFIG_READBACK'
            self.deadline=self.clock()+3

        if kind=='send' and name=='trajectory':
            self.trajectory_pending=(event.get('seq'),event.get('value_i32'))
            if self.state=='ORDINARY_STOP_ACK':self.pending_seq=event.get('seq')
        if kind=='ack' and name=='trajectory' and self.trajectory_pending:
            seq,value=self.trajectory_pending
            if event.get('seq')==seq:
                self.trajectory_pending=None
                if event.get('status')==0:self.ordinary_active=bool(value)
                if self.state=='ORDINARY_STOP_ACK':
                    if event.get('status') or value!=0:
                        self.abort('普通往返未结束，正弦未启动；请查看板端返回状态');return
                    self.state='WAIT_HOLD';self.pending_seq=None;self.deadline=self.clock()+6
                    self.message='普通往返已关闭；等待参考结束后进入正弦。'
        if kind=='ack' and name in ('start','capture') and event.get('status')==0:
            self.capture_ms=int(event['ms'])
        if kind=='send' and name=='ext2' and self.state in ('WAIT_ACK','EXIT_ACK'):
            self.pending_seq=event['seq'];self.movement_sent=True
        if kind=='ack' and name=='ext2' and self.state in ('WAIT_ACK','EXIT_ACK'):
            if event.get('status') or event.get('seq')!=self.pending_seq:
                self.abort('板端未接受本次切换；当前控制保持，请查看返回状态');return
            self.state='WAIT_FLAG' if self.state=='WAIT_ACK' else 'EXITING'
            self.deadline=self.clock()+6
            self.log('ext2_switch_ack',ms=event.get('ms'))
        if kind=='send' and name=='stop':
            self.ordinary_active=False;self.trajectory_pending=None;self.abort('已STOP结束本次运行')
        if kind in ('reset','connected','firmware_changed','stale','error','timeout','unsafe_stop'):
            self.capture_ms=None
            if kind in ('reset','connected','firmware_changed'):self.ordinary_active=False;self.trajectory_pending=None
            if self.active:self.abort('连接或命令异常：'+str(event.get('reason',kind)))
        if kind in ('batch_failed','request_rejected') and self.state in ('ORDINARY_STOP_READY','ORDINARY_STOP_ACK','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK','CONFIG_ACK','WAIT_HOLD','WAIT_ACK','WAIT_FLAG','EXIT_READY','EXIT_ACK'):
            self.abort('切换未完成：'+str(event.get('reason','命令被取消')))
    def reason(self,row,info,qualified,fresh,idle):
        if self.active or info.get('ext2_prepared'):return '拓展2已申请或运行中；结束时点击“退出拓展2，继续保持”'
        if not qualified or not fresh or not idle:return '等待本版本资格、新鲜回读和当前命令确认'
        if (info.get('build'),info.get('channel'),info.get('motor_sign'),info.get('coordinate_verified'))!=(BUILD,1,-1,True):return '等待R40板端身份、方向和物理坐标核对'
        if not capture_row(row):return '先完成起摆或手动捕获；本次运行中无需重新捕获'
        return ''
    def begin(self,row,info,qualified,fresh,idle,settings=None):
        chosen=make_settings(**settings) if settings is not None else None
        if self.state=='RUNNING' and chosen is not None and qualified and fresh and idle and capture_row(row):
            self.settings=chosen;self.settings_written=False;self.restart_pending=True
            return self.exit()
        why=self.reason(row,info,qualified,fresh,idle)
        if why:return why
        self.settings=chosen;self.settings_written=False;self.restart_pending=False
        self.state='WAIT_HOLD';self.session_ext2=True;self.movement_sent=False;self.pending_seq=None;self.deadline=None
        if info.get('sine_settings_fresh'):self.ordinary_active=bool(info.get('sine_ordinary_running'))
        if self.ordinary_active:self.state='ORDINARY_STOP_READY'
        self.message='正弦尚未启动：先自动关闭普通往返，再切换正弦。' if self.ordinary_active else '正弦尚未启动：等待本次捕获或定位参考结束。'
        self.log('ext2_requested',ms=int(row['ms']),configuration_unchanged=True,recapture=False)
        return ''
    def exit(self):
        if self.state in ('WAIT_HOLD','ORDINARY_STOP_READY','CONFIG_AMPLITUDE','CONFIG_PERIOD','CONFIG_READBACK'):self.abort('已取消尚未执行的正弦申请');return ''
        if self.state not in ('RUNNING','WAIT_FLAG'):return '拓展2尚未运行或正在退出'
        self.state='EXIT_READY';self.deadline=None;self.message='拓展2正在平滑退出并回到本轮中心，随后可继续定位。'
        return ''
    def begin_hold(self,*args,**kwargs):return self.exit()
    def command_reason(self,req,row,info,history):
        if req.name in ('stop','d02info'):return ''
        if req.name in ('sine_amplitude','sine_period') and self.state=='CONFIG_ACK':return ''
        if req.name=='trajectory' and req.value==0 and self.state=='ORDINARY_STOP_ACK':return ''
        if req.name=='ext2':
            if req.value==1 and self.state!='WAIT_ACK':return '请使用“进入拓展2”按钮'
            if req.value==0 and self.state!='EXIT_ACK':return '请使用“退出拓展2”按钮'
            return ''
        if self.active and req.name in ('target','trajectory','start','capture','arm','cal','motioncfg','motion_speed'):
            return '先点击“退出拓展2，继续保持”，退出完成后直接继续定位，无需STOP或重新捕获'
        return ''
    def tick(self,row,info,history,qualified,fresh,info_fresh,idle):
        if not self.active:return
        if not qualified or not fresh or not info_fresh:return
        if not capture_row(row):
            self.abort('本次平衡已退出或进入自动恢复；取消拓展2，板端继续原恢复保护');return
        if self.state=='ORDINARY_STOP_READY' and idle:
            self.state='ORDINARY_STOP_ACK';self.deadline=self.clock()+2
            self.actions.append(('trajectory',0));self.log('ext2_stop_ordinary_sent',ms=int(row['ms']))
        elif self.state=='WAIT_HOLD' and idle and hold_row(row):
            cfg=info.get('motion_config')
            try:bound=unpack_internal(cfg)['workspace_deg']*1144-11438-(self.settings['amplitude_internal_q16'] if self.settings is not None else info.get('sine_amplitude_internal_q16',48620))
            except (TypeError,ValueError,KeyError):self.abort('缺少有效行程配置');return
            # Only the board owns the retained session center after recovery.
            # A telemetry reference is the fade origin, not that center.
            if bound<0:
                self.abort('正弦半幅超出已设行程：减小半幅或修改机械行程');return
            if self.settings is not None and not self.settings_written:
                self.state='CONFIG_AMPLITUDE';self.deadline=self.clock()+3;return
            self.state='WAIT_ACK';self.deadline=self.clock()+2
            self.actions.append(('ext2',1));self.log('ext2_entry_sent',ms=int(row['ms']),entry_reference_physical_rad=row['reference'])
        elif self.state in ('CONFIG_AMPLITUDE','CONFIG_PERIOD') and idle:
            name='sine_amplitude' if self.state=='CONFIG_AMPLITUDE' else 'sine_period'
            value=self.settings['amplitude_physical_q16' if name=='sine_amplitude' else 'phase_step']
            self.state='CONFIG_ACK';self.deadline=self.clock()+3;self.actions.append((name,value))
        elif self.state=='CONFIG_READBACK' and idle:
            if confirmed(info,self.settings):
                self.settings_written=True;self.state='WAIT_HOLD';self.deadline=None
                self.log('sine_parameters_confirmed',**self.settings)
            else:self.actions.append(('d02info',0))
        elif self.state=='WAIT_FLAG' and info.get('ext2_prepared') and int(row['flags'])&256:
            self.state='RUNNING';self.deadline=None;self.message='正弦运行中（板端已确认）；半幅和周期以板端回读为准。'
            self.log('ext2_running_confirmed',ms=int(row['ms']))
        elif self.state=='EXIT_READY' and idle:
            self.state='EXIT_ACK';self.deadline=self.clock()+2;self.actions.append(('ext2',0))
        elif self.state=='EXITING' and not info.get('ext2_prepared') and hold_row(row):
            self.state='READY';self.deadline=None;self.message='已回到本轮中心并继续保持；可定位、普通往返或再次进入拓展2。'
            self.log('ext2_exit_complete',ms=int(row['ms']),reference_physical_rad=row['reference'])
            if self.restart_pending:
                self.restart_pending=False;self.state='WAIT_HOLD';self.message='已平滑退出；正在应用新半幅/周期并重新进入正弦'
        elif self.state=='RUNNING' and not info.get('ext2_prepared'):
            self.abort('拓展2已退出；可继续普通功能')
        if self.deadline is not None and self.clock()>self.deadline:
            self.abort('切换回读超时，已请求STOP',stop=True)
