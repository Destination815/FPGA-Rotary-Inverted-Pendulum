"""Serial engine: worker-owned port/logs; bounded UI buffers; explicit ACK state."""
import math
import json
import hashlib
from pathlib import Path
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

from ext2_sequence import Ext2Sequence, EXT2_CONFIG
from stop_tail import StopTailGate
from protocol import Decoder, OPS, command, D03_BUILD, CONTROL_BUILD
from coordinates import PROFILE,internal_row,descriptor_valid
from d02_data import Capture
from studio_data import RunWriter, validate_profile
from r01_gate import qualification,check,BUILD,board_ok,host_dir
from session_preparation import guarded,reviewed_hashes,reason as preparation_reason,swing_reason,automatic_bitstream

@dataclass(frozen=True)
class Request:
    name: str
    value: int = 0
    expected_motion_config: int | None = None

def profile_requests(profile):
    p=validate_profile(profile)
    return [Request('stop'),Request('clear'),Request('trajectory',0),Request('target',0),
            *[Request('k'+k,round(p[k]*65536)) for k in ('theta','dtheta','phi','dphi')],
            Request('limit',p['limit']),Request('swing',p['swing'])]

def permitted(name,row,fresh=True):
    row=internal_row(row)
    if name=='stop':return True
    if not fresh or not row:return False
    flags=int(row['flags']);idle=row['state']==0 and row['fault']==0
    armed=bool(flags&1);cal=bool(flags&2);valid=bool(flags&4)
    if name=='d02info':return True
    if name=='d02read':return False
    if name=='d02start':return False
    if name in ('motioncfg','motion_speed'):return idle and not armed
    if name in ('sine_amplitude','sine_period'):return row['fault']==0 and row['state'] in (0,2,3,4)
    if name=='ext2':return row['fault']==0 and row['state'] in (3,4) and armed and cal and valid
    if name=='clear':return not armed
    if name=='cal':return idle and not armed and bool(flags&8)
    if name=='arm':return idle and not armed and cal and valid
    if name=='jog':return False
    if name=='start':return idle and armed and cal and valid and abs(row['theta'])>2.5 and abs(row['dtheta'])<2 and abs(row['dphi'])<.5
    if name=='capture':return idle and armed and cal and valid and abs(row['theta'])<1144/65536 and abs(row['dtheta'])<13108/65536 and abs(row['dphi'])<1.0
    if name in ('ktheta','kdtheta','kphi','kdphi','limit','swing'):return False
    return name in ('target','trajectory') and row['fault']==0 and row['state'] in (3,4) and armed and cal and valid

class CommandPump:
    """No automatic retries. STOP preempts; confirmation must follow send in board time."""
    def __init__(self,write,emit,clock=time.monotonic):
        self.write=write;self.emit=emit;self.clock=clock;self.gate=lambda req:(req.name in ('stop','d02info'),'R01 gate 未初始化')
        self.latest={};self.received_at=0.;self.seq=0;self.pending=None
        self.todo=deque();self.batch=None;self.last_ms=None;self.frame_id=0;self.generation=0
    @property
    def fresh(self):return bool(self.latest) and self.clock()-self.received_at<.5
    @property
    def busy(self):return bool(self.pending or self.todo or self.batch)
    def cancel(self,why):
        self.todo.clear();self.pending=None
        if self.batch:self.emit(dict(event='batch_failed',label=self.batch,reason=why))
        self.batch=None
    def submit(self,requests,label):
        if self.busy:raise ValueError('仍有命令等待确认，请等待或点击停止')
        if not self.fresh:raise ValueError('尚无新鲜遥测，不能执行命令')
        self.todo.extend(requests);self.batch=label
    def stop(self):
        self.cancel('被停止操作取消');self.batch='stop';self.todo.append(Request('stop'))
    def feed(self,row):
        ms=int(row['ms']);reset=False
        if self.last_ms is not None and ms<self.last_ms:
            reset=not(self.last_ms>0xF0000000 and ms<0x10000000)
        self.frame_id+=1;self.latest=dict(row);self.received_at=self.clock();self.last_ms=ms
        if reset:
            self.generation+=1;self.cancel('设备复位');self.emit(dict(event='reset'))
            self.stop();return
        p=self.pending
        if p and self.frame_id>p['frame_id'] and ((ms-p['ms'])&0xffffffff)>0 and ((ms-p['ms'])&0xffffffff)<0x80000000 and int(row['ack_seq'])==p['seq']:
            status=int(row['ack_status']);self.pending=None
            self.emit(dict(event='ack',seq=p['seq'],name=p['request'].name,status=status,ms=ms))
            if status:
                self.cancel(f'FPGA 拒绝 {p["request"].name}，状态码 {status}')
            elif p['request'].name=='stop' and ((int(row['flags'])&1) or row['u']!=0):
                self.cancel('STOP 已确认但尚未观测到未使能/u=0');self.emit(dict(event='unsafe_stop'))
    def tick(self):
        now=self.clock()
        if self.pending:
            if now>=self.pending['deadline']:
                old=self.pending['request'].name
                self.emit(dict(event='timeout',name=old));self.cancel('命令确认超时')
                if old!='stop':self.stop()
            return
        if not self.todo:
            if self.batch:
                label=self.batch;self.batch=None;self.emit(dict(event='batch_done',label=label))
            return
        req=self.todo[0]
        allowed,why=self.gate(req)
        if not allowed:self.cancel(why);return
        if not permitted(req.name,self.latest,self.fresh):
            self.cancel(f'{req.name} 的状态条件不满足或遥测过期');return
        self.todo.popleft()
        latest_seq=int(self.latest.get('ack_seq',0));self.seq=(max(self.seq,latest_seq)+1)&255
        if self.seq==latest_seq:self.seq=(self.seq+1)&255
        frame=command(self.seq,OPS[req.name],req.value)
        self.emit(dict(event='send',seq=self.seq,opcode=OPS[req.name],name=req.name,value_i32=req.value))
        if self.write(frame)!=len(frame):raise IOError('串口写入不完整')
        self.pending=dict(request=req,seq=self.seq,deadline=now+1.5,ms=int(self.latest.get('ms',0)),frame_id=self.frame_id)

class SerialSession(threading.Thread):
    def __init__(self,port,directory,metadata=None,serial_factory=None,clock=time.monotonic):
        super().__init__(daemon=True,name='J280 serial / recorder')
        self.port=port;self.directory=directory;self.metadata=metadata or {};self.factory=serial_factory;self.clock=clock
        self.requests=queue.Queue(maxsize=32);self.stop_flag=threading.Event();self.close_flag=threading.Event()
        self.lock=threading.Lock();self.view_rows=deque(maxlen=3000);self.events=deque(maxlen=500)
        self.latest={};self.last_rx=0.;self.connected=False;self.active_path=None;self.busy=False
        self.last_error='';self.decoder=Decoder();self.received_bytes=0;self.recorder=None;self.pump=None
        self.qualification_ok,self.qualification_note=qualification();self.info_requested_at=0.
        self.qualification_snapshot={}
        host_path=host_dir()
        for key,path in [('source_manifest',host_path.parent/'source_manifest.json'),('qualification_receipt',host_path/'qualification.json'),('sta_review',host_path/'sta_review.json'),('observer_constants',host_path.parent/'observer/constants.json')]:
            try:
                raw=path.read_bytes();self.qualification_snapshot[key]=dict(sha256=hashlib.sha256(raw).hexdigest(),contents=json.loads(raw))
            except (OSError,ValueError):self.qualification_snapshot[key]=dict(available=False)

        self.profile=None;self.profile_confirmed=False;self.pending_profile=None;self.deadline=None
        self.finalize=False;self.closing=False;self.close_deadline=None;self.finalize_not_before=0.;self.motion_seconds=0
        self.sine_info={};self.sine_info_at=0.;self.coordinate_info={};self.coordinate_info_at=0.;self.d02_info={};self.d02_info_at=0.;self.d02_expected=None;self.d02_capture=None
        self.d02_pending=False;self.d02_read_due=False;self.d02_reading=False;self.d02_since=0.;self.d02_path='';self.d03_requested=None
        self.preparation=dict(selected_bitstream=self.metadata.get('selected_bitstream',{}),motion_config=None,range_confirmed=False)
        self.expected_bitstreams=reviewed_hashes(self.qualification_snapshot)
        self.ext2_sequence=Ext2Sequence(self.clock);self.stop_tail=StopTailGate();self.ext2_tail_required=False
        self.ext2_prepare_stage=None;self.ext2_prepare_deadline=None

    def set_preparation(self,preparation):
        # GUI publishes only local selections. Worker rechecks against fresh INFO at send.
        with self.lock:self.preparation=dict(preparation,selected_bitstream=dict(preparation.get('selected_bitstream') or {}))

    def request(self,kind,payload=None):
        if kind=='stop':self.stop_flag.set();return
        if kind=='close':self.close_flag.set();return
        self.requests.put_nowait((kind,payload))
    def _board_info(self):
        info=dict(self.d02_info,**getattr(self,'sine_info',{}),sine_settings_fresh=self.clock()-getattr(self,'sine_info_at',0)<.6,coordinate_verified=bool(self.coordinate_info.get('verified') and self.clock()-self.coordinate_info_at<.6))
        if info['coordinate_verified'] and 'motion_speed_internal_q16' in self.coordinate_info:info['motion_speed_internal_q16']=self.coordinate_info['motion_speed_internal_q16']
        return info
    def background_info_only(self):
        p=self.pump
        return bool(p and p.batch=='version_poll' and p.pending and p.pending['request'].name=='d02info' and not p.todo)

    def snapshot(self):
        with self.lock:
            events=list(self.events);self.events.clear()
            return dict(connected=self.connected,latest=dict(self.latest),last_rx=self.last_rx,
                        rows=list(self.view_rows),events=events,busy=(self.busy and not self.background_info_only()) or bool(self.ext2_prepare_stage),active_path=self.active_path,
                        profile_confirmed=self.profile_confirmed,acknowledged_profile=self.profile,last_error=self.last_error,
                        qualification_ok=self.qualification_ok,qualification_note=self.qualification_note,
                        d02_info=self._board_info(),d02_fresh=self.clock()-self.d02_info_at<.6,
                        d02_busy=self.d02_pending or self.d02_reading or self.d02_read_due,
                        expected_bitstreams=list(self.expected_bitstreams),
                        ext2_tail_required=self.ext2_tail_required,finalizing=self.finalize,
                        ext2=dict(ordinary_active=self.ext2_sequence.ordinary_active,state=self.ext2_sequence.state,message=self.ext2_sequence.message,used=self.ext2_sequence.used,manual_capture=self.ext2_sequence.capture_ms is not None,capture_ms=self.ext2_sequence.capture_ms),
                        d02_progress=len(self.d02_capture.rows) if self.d02_capture else 0,d02_path=self.d02_path)
    def emit(self,event):
        event=dict(self.ext2_sequence.normalize_event(event),time=time.time())
        self.ext2_sequence.event(event);self.stop_tail.event(event)
        if self.recorder:self.recorder.event(event)
        with self.lock:self.events.append(event)
        if event['event'] in ('reset','timeout','batch_failed'):
            self.ext2_prepare_stage=None
            self.profile_confirmed=False;self.pending_profile=None
        if event['event']=='reset':
            self.d03_requested=None
            self.deadline=None;self.sine_info={};self.sine_info_at=0;self.coordinate_info={};self.coordinate_info_at=0;self.decoder.coordinates=None;self.d02_info={};self.d02_info_at=0;self.d02_expected=None
            self.save_d02_partial('设备复位');self.d02_pending=False;self.d02_reading=False;self.d02_read_due=False
        if (event['event']=='ack' and event['name']=='d02start' and event['status']) or (event['event']=='timeout' and event.get('name')=='d02start'):
            self.d02_pending=False;self.d02_expected=None

        if event['event']=='batch_done' and event['label']=='profile':
            self.profile=self.pending_profile;self.pending_profile=None;self.profile_confirmed=True
            if self.recorder:self.recorder.meta['last_acknowledged_profile']=self.profile
        if event['event']=='ack' and event['name']=='ext2' and event['status']==0:self.ext2_tail_required=True
        if event['event']=='ack' and event['name'] in ('capture','start') and event['status']==0:
            self.deadline=self.clock()+self.motion_seconds if self.motion_seconds else None
    def save_d02_partial(self,why):
        if self.d02_capture and self.recorder:
            if why and not self.d02_capture.complete:self.d02_capture.errors.append(why)
            self.d02_path=str(self.d02_capture.save(self.recorder.path))

    def command_gate(self,req):
        prepare_allowed=('stop','d02info','motioncfg','motion_speed') if str(self.ext2_prepare_stage).startswith('ORDINARY_') else ('stop','d02info','motioncfg','ext2')
        if self.ext2_prepare_stage and req.name not in prepare_allowed:
            return False,'拓展2准备中，等待配置、坐标速度和独立许可回读完成'
        why=self.ext2_sequence.command_reason(req,self.pump.latest if self.pump else {},self._board_info(),list(self.view_rows))
        if why:return False,why
        if guarded(req.name,req.value):
            with self.lock:prepared=dict(self.preparation)
            if req.expected_motion_config is None:return False,'命令缺少当前界面配置；请通过本版上位机重新点击'
            why=preparation_reason(prepared,self.expected_bitstreams,self._board_info(),self.clock()-self.d02_info_at<.6,req.expected_motion_config)
            if why:return False,why
            if req.name=='start':
                why=swing_reason(prepared,self.d02_info)
                if why:return False,why
        if req.expected_motion_config is not None and req.name in ('target','trajectory') and not (req.name=='trajectory' and req.value==0):
            if self.d02_info.get('motion_config')!=req.expected_motion_config:
                return False,'所选运动配置尚未板端回读确认；停止并应用配置后再操作'
        return check(req.name,req.value,self.pump.latest,self._board_info(),self.clock()-self.d02_info_at<.6,self.qualification_ok)

    def diagnostic_packet(self,p):
        if p.get('kind')=='sine_settings':
            self.sine_info={k:v for k,v in p.items() if k.startswith('sine_')};self.sine_info_at=self.clock()
            if self.recorder:self.recorder.event(dict(event='sine_settings_readback',**p))
            return
        if p.get('kind')=='coordinates':
            self.coordinate_info=dict(p);self.coordinate_info_at=self.clock()
            if self.recorder:
                self.recorder.meta['coordinate_profile']=PROFILE
                self.recorder.meta['coordinate_descriptor']=dict(p)
                self.recorder.meta['coordinate_units']={'public_angles':'physical rad/rad_s','raw_wire_type5':'phi physical Q16; arm velocity/reference/target legacy Q16','passive_wire':'legacy internal with raw encoder; public CSV reconstructed using capture-bound encoder_zero','observer_checkpoint':'legacy_internal z/integral/origin; no reinterpretation','internal_extra_fields':'original legacy radians/rad_s for exact replay'}
                self.recorder.event(dict(event='coordinate_descriptor',**p))
            if not p.get('verified') and self.pump:self.pump.stop()
            return
        if p.get('kind')=='compact_info':
            if self.recorder:self.recorder.event(dict(event='compact_cache_identity',**p))
            return
        if p.get('kind')=='compact_data':return
        if p.get('kind') in ('ext2_checkpoint_b','ext2_checkpoint_c'):
            if self.recorder:self.recorder.event(dict(event=p['kind'],**p))
            return
        if p.get('kind')=='observer_checkpoint':
            if self.recorder:self.recorder.event(dict(event='observer_checkpoint',**p))
            return
        if self.recorder and p.get('filter_format'):
            self.recorder.meta['adc_filter_info_snapshot']=dict(p)
            self.recorder.meta['measurement_semantics']={'theta':'FPGA sub-ms ADC Q4 mean angle; raw fallback in invalid blocks; gap prediction if flags bit9', 'dtheta':'FPGA 1kHz alpha-beta velocity; rad/s', 'adc':'legacy 16-conversion integer diagnostic mean', 'zero':'integer part of fractional calibration; exact zero in adc_filter_info_snapshot.zero_q4 /16', 'host_smoothing':False}
        if p.get('kind')=='info' and self.d02_info and self.d02_info.get('build')!=p.get('build') and self.pump:
            self.coordinate_info={};self.coordinate_info_at=0;self.pump.stop();self.emit(dict(event='firmware_changed',reason='板端版本变化，已请求停止'))
        if p['build']!=D03_BUILD:
            if p['kind']=='info':
                self.d02_info=dict(p);self.d02_info_at=self.clock()
                if self.recorder:
                    self.recorder.meta['firmware_readback']=dict(build=p['build'],channel=p['channel'],motor_sign=p['motor_sign'],role='R01 full control candidate' if p['build']==CONTROL_BUILD else 'unrecognized',full_bitstream_verified=False)
                    self.recorder.meta['motion_config_readback']=p.get('motion_config')
                    identity={k:p.get(k) for k in ('build','capture_id','count','phase','motion_config')}
                    if identity!=self.recorder.meta.get('last_cache_identity'):
                        self.recorder.event(dict(event='cache_identity',**identity))
                        self.recorder.meta['last_cache_identity']=identity
            return
        if p['kind']=='info':
            self.d02_info=dict(p);self.d02_info_at=self.clock()
            if self.recorder:self.recorder.meta['d02_firmware_readback']=dict(build=p['build'],channel=p['channel'],motor_sign=p['motor_sign'],limit=p['limit'],capability_verified=True)
            if self.d02_pending and p['capture_id']==self.d02_expected and p['phase'] in (4,5):
                self.d02_capture=Capture(p)
                if p['amplitude']!=self.d03_requested:self.d02_capture.errors.append('请求幅值与板端记录不一致')
                self.d02_pending=False;self.d02_read_due=bool(p['count']);self.d02_since=self.clock()
                if not p['count']:
                    self.save_d02_partial('启动即中止，无有效样本');self.emit(dict(event='d02_failed',reason='采集已中止，无有效样本'))
        elif self.d02_reading and self.d02_capture:
            self.d02_capture.feed(p)
            if self.d02_capture.complete:
                self.save_d02_partial('');self.d02_reading=False;self.d02_expected=None
                result=self.d02_capture.result()
                good=result.get('acceptance',{}).get('passed',False)
                self.emit(dict(event='d02_complete',path=self.d02_path,reason='数据完整，记录检查通过' if good else '记录已保存，但中止或检查未通过；请查看详情',summary=result.get('summary',{}),acceptance=result.get('acceptance',{}),amplitude=self.d02_capture.info['amplitude']))

    def start_d02(self,value):
        info=self.d02_info
        if value not in (-150,-100,0,100,150):raise ValueError('D03 只允许 0 / ±10% / ±15%')
        if self.d02_pending or self.d02_reading or self.d02_read_due:raise ValueError('上一轮高速记录尚未完成，请勿重复点击')
        if info.get('build')!=D03_BUILD or self.clock()-self.d02_info_at>=.6 or not info.get('ready') or info.get('dumping'):
            raise ValueError('需要 D03 固件、下垂静止、摆臂 ±1°内且无传输任务')
        if info.get('channel')!=1 or info.get('motor_sign')!=-1 or info.get('limit')!=150:
            raise ValueError('板端不是已确认的 XH1 / 方向 -1 / 限幅 150 配置')
        if not self.profile_confirmed or self.profile.get('limit')!=150:raise ValueError('请先应用 D03 15% 配置')
        if not permitted('d02start',self.pump.latest,self.pump.fresh):raise ValueError('D03 状态条件不满足')
        self.pump.submit([Request('d02start',value)],'d02start')
        self.d02_capture=None;self.d02_expected=(info['capture_id']+1)&0xffffffff
        self.d02_pending=True;self.d02_since=self.clock();self.d03_requested=value
        self.emit(dict(event='d02_requested',capture_id=self.d02_expected,amplitude=value))

    def retry_d02(self):
        info=self.d02_info
        if self.d02_pending or self.pump.busy:raise ValueError('请等待采集/命令结束')
        if info.get('build')!=D03_BUILD or self.clock()-self.d02_info_at>=.6 or info.get('phase') not in (4,5) or not info.get('count'):
            raise ValueError('板上没有可读回的冻结记录')
        self.d02_capture=Capture(info);self.d02_expected=info['capture_id']
        if self.d03_requested is not None and info['amplitude']!=self.d03_requested:
            self.d02_capture.errors.append('请求幅值与板端记录不一致')
        self.d02_reading=False;self.d02_read_due=True;self.d02_since=self.clock()

    def new_run(self):
        self.ext2_sequence=Ext2Sequence(self.clock);self.stop_tail=StopTailGate();self.ext2_tail_required=False
        self.ext2_prepare_stage=None;self.ext2_prepare_deadline=None
        self.run_start_bytes=self.received_bytes;self.run_start_bad_frames=self.decoder.bad_frames
        self.recorder=RunWriter(self.directory,dict(self.metadata,port=self.port,baud=115200,candidate='R40 seamless six-function host; qualification state recorded separately; operator STOP default',expected_build='4A070257',qualification_ok=self.qualification_ok,qualification_note=self.qualification_note,qualification_snapshot=self.qualification_snapshot,parameter_policy='R40 FPGA control; no host gain writes',host_runtime_fix=dict(name='R39 session preparation and heartbeat',build='4A070257',source='host/studio_session.py',change='No local bitstream prerequisite; fresh configuration/speed readback and explicit 300ms INFO heartbeat')))
        if self.profile_confirmed:self.recorder.meta['last_acknowledged_profile']=self.profile
        self.active_path=str(self.recorder.path)
        self.emit(dict(event='run_started',path=self.active_path))
    def finish_run(self,reason):
        if not self.recorder:return
        self.save_d02_partial('日志结束，数据未收齐')
        path=str(self.recorder.path)
        self.recorder.close(dict(host_bad_frames=self.decoder.bad_frames-self.run_start_bad_frames,
            received_bytes=self.received_bytes-self.run_start_bytes,
            receiver_errors=[self.last_error] if self.last_error else [],end_reason=reason))
        self.recorder=None;self.active_path=None
        with self.lock:self.events.append(dict(event='run_finalized',path=path,reason=reason,ext2_tail_required=self.ext2_tail_required,time=time.time()))
    def ext2_stop_tail(self):
        """A391: matched STOP ACK plus a continuous received five-second tail."""
        if not self.pump or not self.pump.fresh:return False
        return self.stop_tail.complete

    def begin_finalize(self,closing=False,force_stop=False):
        self.finalize=True;self.closing=closing or self.closing;self.deadline=None
        special=self.ext2_tail_required
        self.close_deadline=self.clock()+(8.0 if special else 2.0)
        if not special:
            self.pump.stop();return
        reuse=not force_stop and not self.pump.busy and self.stop_tail.ack_ms is not None and self.pump.fresh
        if not reuse:self.pump.stop()
        self.emit(dict(event='ext2_close_tail_reuse' if reuse else 'finalize_stop_requested',
                       five_second_tail_verified=bool(reuse and self.ext2_stop_tail())))

    def poll_finalize(self):
        if not self.finalize:return False
        special=self.ext2_tail_required
        if special:
            if not self.pump.fresh:self.stop_tail.feed(self.pump.latest,False)
            if self.stop_tail.complete and not self.pump.busy:
                if self.recorder:self.recorder.meta['stop_tail_evidence']=self.stop_tail.evidence()
                self.finish_run('confirmed_stop');self.finalize=False
                return self.closing
            if self.clock()>=self.close_deadline:
                self.finalize=False
                if self.closing:
                    self.finish_run('stop_tail_incomplete_on_close');return True
                self.emit(dict(event='finalize_blocked',reason='STOP匹配确认及连续5秒零输出尾尚未完整；原日志继续保留，可重试保存，不会重发运动',stop_tail=self.stop_tail.evidence()))
            return False
        stopped=self.pump.fresh and not(int(self.pump.latest.get('flags',1))&1) and self.pump.latest.get('u',1)==0
        if (not self.pump.busy and stopped) or self.clock()>=self.close_deadline:
            self.finish_run('confirmed_stop' if stopped and not self.pump.busy else 'stop_not_confirmed_use_SW4')
            self.finalize=False
            if self.closing:return True
        return False

    def service_ext2(self):
        if not self.pump:return
        if self.ext2_prepare_stage:
            if self.clock()>self.ext2_prepare_deadline:
                self.ext2_prepare_stage=None;self.pump.stop();self.emit(dict(event='ext2_prepare_failed',reason='本次配置/速度/权限回读3秒内未齐，已STOP'))
            elif not self.pump.busy:
                info=self._board_info();fresh=self.clock()-self.d02_info_at<.6
                if self.ext2_prepare_stage=='ORDINARY_CONFIG_READBACK' and fresh and info.get('coordinate_verified') and info.get('motion_config')==self.ordinary_prepare_config:
                    # Only write ordinary speed after the board has left the A domain.
                    self.ext2_prepare_stage='ORDINARY_SPEED_READBACK'
                    self.pump.submit([Request('motion_speed',self.ordinary_prepare_speed),Request('d02info')],'prepare_session_speed')
                elif self.ext2_prepare_stage=='ORDINARY_SPEED_READBACK' and fresh and info.get('coordinate_verified') and info.get('motion_config')==self.ordinary_prepare_config:
                    from coordinates import target_internal_q16
                    if info.get('motion_speed_internal_q16')==target_internal_q16(self.ordinary_prepare_speed):
                        self.ext2_prepare_stage=None;self.emit(dict(event='session_prepared',motion_config=self.ordinary_prepare_config,motion_speed_physical_q16=self.ordinary_prepare_speed))
                    elif self.pump.fresh:self.pump.submit([Request('d02info')],'prepare_session_speed_readback')
                elif self.pump.fresh:
                    self.pump.submit([Request('d02info')],'ext2_prepare_readback')
        with self.lock:rows=list(self.view_rows)
        self.ext2_sequence.tick(self.pump.latest,self._board_info(),rows,self.qualification_ok,
                                self.pump.fresh,self.clock()-self.d02_info_at<.6,not self.pump.busy)
        actions,events=self.ext2_sequence.drain()
        for event in events:self.emit(event)
        for name,value in actions:
            if name=='stop':self.deadline=None;self.pump.stop()
            elif name in ('ext2','trajectory','sine_amplitude','sine_period','d02info'):
                if self.pump.busy:self.ext2_sequence.abort('切换命令暂未发送；当前命令尚未结束');continue
                self.pump.submit([Request(name,value,self._board_info().get('motion_config'))],'ext2_switch')

    def info_poll_due(self):
        """Explicit300ms CRC INFO is also the R36 board communication heartbeat.
        Unsolicited board INFO cannot prove that the PC->board path is alive.
        """
        now=self.clock()
        return bool(self.pump and not self.pump.busy and self.pump.fresh
                    and now-self.info_requested_at>.3
                    and not self.finalize and not self.closing)
    def run(self):
        ser=None
        try:
            if self.factory is None:
                import serial
                self.factory=serial.Serial
            ser=self.factory(self.port,115200,timeout=.02,write_timeout=.2)
            self.pump=CommandPump(ser.write,self.emit,self.clock)
            self.pump.gate=self.command_gate
            self.new_run();self.connected=True;self.emit(dict(event='connected',port=self.port))
            stale_reported=False
            while True:
                chunk=ser.read(min(max(1,ser.in_waiting),8192));self.received_bytes+=len(chunk)
                if self.recorder:self.recorder.rx(chunk)
                for row in self.decoder.feed(chunk):
                    if self.recorder:self.recorder.row(row)
                    self.pump.feed(row)
                    self.stop_tail.feed(row,self.pump.fresh)
                    with self.lock:
                        self.latest=dict(row);self.last_rx=self.clock();self.view_rows.append(dict(row))
                    stale_reported=False
                for packet in self.decoder.take_diagnostics():self.diagnostic_packet(packet)
                if self.stop_flag.is_set() or self.close_flag.is_set():
                    self.ext2_prepare_stage=None
                    closing=self.close_flag.is_set();explicit_stop=self.stop_flag.is_set();self.stop_flag.clear();self.close_flag.clear()
                    while True:
                        try:self.requests.get_nowait()
                        except queue.Empty:break
                    if closing:self.begin_finalize(True,explicit_stop)
                    else:self.pump.stop();self.deadline=None
                if not self.finalize and not self.closing:
                    try:kind,payload=self.requests.get_nowait()
                    except queue.Empty:kind=None;payload=None
                    if kind:
                        if not self.recorder:self.new_run()
                        try:
                            if (self.d02_pending or self.d02_reading or self.d02_read_due) and kind not in ('note','finalize','d02_retry'):
                                raise ValueError('D02 尚在采集/接收，先等待或停止')
                            if kind=='d02_start':self.start_d02(int(payload))
                            elif kind=='d02_retry':self.retry_d02()
                            elif kind in ('prepare_ext2','ext2_begin'):
                                with self.lock:prepared=dict(self.preparation)
                                why=preparation_reason(prepared,self.expected_bitstreams,self._board_info(),self.clock()-self.d02_info_at<.6,self._board_info().get('motion_config'))
                                if why:raise ValueError(why)
                                why=self.ext2_sequence.begin(self.pump.latest,self._board_info(),self.qualification_ok,self.pump.fresh,not self.pump.busy or self.background_info_only(),settings=payload)
                                if why:raise ValueError(why)
                            elif kind in ('ext2_exit','ext2_hold'):
                                why=self.ext2_sequence.exit()
                                if why:raise ValueError(why)
                            elif kind=='swing_begin':
                                r=self.pump.latest
                                if not permitted('start',dict(r,flags=int(r.get('flags',0))|1),self.pump.fresh):
                                    raise ValueError('先校准、准备速度，放杆下垂并撤手停稳，再点使能并起摆')
                                with self.lock:prepared=dict(self.preparation)
                                cfg=prepared.get('motion_config')
                                why=swing_reason(prepared,self._board_info())
                                if why:raise ValueError(why)
                                requests=[]
                                if not int(r.get('flags',0))&1:requests.append(Request('arm',0x41524d31,cfg))
                                requests.append(Request('start',0,cfg))
                                for req in requests:
                                    ok,why=self.command_gate(req)
                                    if not ok:raise ValueError(why)
                                self.pump.submit(requests,'swing_begin');self.motion_seconds=0
                            elif kind=='prepare_session':
                                selected,why=automatic_bitstream(host_dir().parent)
                                if why:raise ValueError(why)
                                cfg=int(payload['motion_config']);speed=int(payload['motion_speed'])
                                if cfg==EXT2_CONFIG:raise ValueError('拓展2请使用专用准备入口')
                                req=Request('motioncfg',cfg);ok,why=self.command_gate(req)
                                if not ok or not permitted(req.name,self.pump.latest,self.pump.fresh):raise ValueError(why or '先停止并确认未使能，再准备本次运行')
                                # Validate the requested speed for its destination configuration.
                                # The real command gate is rechecked after actual board readback.
                                ok,why=check('motion_speed',speed,self.pump.latest,dict(self._board_info(),motion_config=cfg),self.clock()-self.d02_info_at<.6,self.qualification_ok)
                                if not ok:raise ValueError(why)
                                self.pump.submit([req,Request('d02info')],'prepare_session_config')
                                self.ordinary_prepare_config=cfg;self.ordinary_prepare_speed=speed
                                self.ext2_prepare_stage='ORDINARY_CONFIG_READBACK';self.ext2_prepare_deadline=self.clock()+3
                                self.metadata['selected_bitstream']=selected
                                if self.recorder:self.recorder.meta['selected_bitstream']=selected
                                self.emit(dict(event='prepare_session_requested',motion_config=cfg,motion_speed_physical_q16=speed,selected_bitstream=selected))
                            elif kind=='profile':
                                raise ValueError('R01 控制参数固化在 FPGA；禁止导入/发送旧增益配置')
                                if self.pump.latest.get('state') not in (0,5) or int(self.pump.latest.get('flags',1))&1:
                                    raise ValueError('请先停止并确认未使能，再应用配置')
                                profile=validate_profile(payload)
                                self.pump.submit(profile_requests(profile),'profile')
                                self.pending_profile=profile;self.profile_confirmed=False
                                self.emit(dict(event='profile_requested',profile=profile))
                            elif kind=='command':
                                name=payload['name'];value=int(payload.get('value',0))
                                seconds=float(payload.get('seconds',0))
                                if not math.isfinite(seconds) or seconds<0:raise ValueError('停止计时须为非负有限值；0表示手动停止')
                                allowed,why=check(name,value,self.pump.latest,self._board_info(),self.clock()-self.d02_info_at<.6,self.qualification_ok)
                                if not allowed:raise ValueError(why)
                                req=Request(name,value,payload.get('expected_motion_config'))
                                allowed,why=self.command_gate(req)
                                if not allowed:raise ValueError(why)
                                self.pump.submit([req],name)
                                if name in ('start','capture'):self.motion_seconds=seconds
                                if name=='stop':self.deadline=None
                            elif kind=='finalize':
                                self.begin_finalize()
                            elif kind=='note':
                                import json
                                try:
                                    note=json.loads(payload)
                                    if isinstance(note,dict):
                                        updates={k:note[k] for k in ('experiment_name','question','change','operator_notes','selected_bitstream') if k in note}
                                        self.metadata.update(updates)
                                        if self.recorder:self.recorder.meta.update(updates)
                                except (ValueError,TypeError):pass
                                self.emit(dict(event='operator_note',text=str(payload)[:4000]))
                            else:raise ValueError('未知操作')
                        except (ValueError,KeyError,TypeError) as e:self.emit(dict(event='request_rejected',reason=str(e)))
                self.service_ext2()
                if self.deadline and self.clock()>=self.deadline:
                    self.deadline=None;self.emit(dict(event='auto_stop',reason='本轮观察时间已到'))
                    self.pump.stop()
                if self.pump.latest and not self.pump.fresh and not stale_reported:
                    stale_reported=True;self.profile_confirmed=False
                    self.emit(dict(event='stale',reason='遥测超过 0.5 秒未更新；尝试停止，请检查 SW4'))
                    self.pump.stop();self.deadline=None
                if self.d02_pending and self.clock()-self.d02_since>3:
                    self.d02_pending=False;self.d02_expected=None;self.pump.stop()
                    self.emit(dict(event='d02_failed',reason='采集状态等待超时，已请求停止；不自动重试运动'))
                if self.d02_read_due and not self.pump.busy and self.pump.fresh:
                    self.pump.submit([Request('d02read',0)],'d02read');self.d02_read_due=False;self.d02_reading=True;self.d02_since=self.clock()
                if self.d02_reading and self.clock()-self.d02_since>20:
                    self.save_d02_partial('接收超时/缺包');self.d02_reading=False;self.d02_expected=None
                    self.emit(dict(event='d02_failed',reason='数据未收齐；可重新读取上次数据，不必再次点动',path=self.d02_path))
                if self.info_poll_due():
                    self.pump.submit([Request('d02info')],'version_poll');self.info_requested_at=self.clock()
                self.pump.tick();self.busy=self.pump.busy
                if self.poll_finalize():break
        except Exception as error:
            self.last_error=str(error);self.profile_confirmed=False
            if ser is not None:
                try:ser.write(command(((self.pump.seq if self.pump else 0)+1)&255,OPS['stop'],0))
                except Exception:pass
            with self.lock:self.events.append(dict(event='error',reason=f'{error}；停止未确认，请使用 SW4',time=time.time()))
        finally:
            self.connected=False
            try:self.finish_run('transport_or_file_error' if self.last_error else 'closed')
            except Exception as error:
                self.last_error=str(error)
                if self.recorder:
                    for f in (self.recorder.log,self.recorder.events,self.recorder.raw_rx):
                        try:f.close()
                        except Exception:pass
                with self.lock:self.events.append(dict(event='error',reason=f'日志收尾失败：{error}，保留原始文件',time=time.time()))
            if ser is not None:
                try:ser.close()
                except Exception:pass
            with self.lock:self.events.append(dict(event='disconnected',time=time.time()))
