"""Portable data/config layer. No Qt, serial access, or control side effects."""
import csv
import hashlib
import io
import json
import math
import os
import sys
import zipfile
from datetime import datetime
from pathlib import Path
from coordinates import EXTRA_FIELDS,PROFILE,SCHEMA

APP_VERSION = 'R42 1.0.0'

def load_output_contract(build):
    """Attach only to the exact board version; expected UI identity is not proof."""
    try:
        value=int(build,16) if isinstance(build,str) else int(build)
    except (TypeError,ValueError):
        return None
    if value!=0x4A070256:
        return None
    contract_path=(Path(sys.executable).resolve().parent/'host/output_contract.json') if getattr(sys,'frozen',False) else Path(__file__).with_name('output_contract.json')
    contract=json.loads(contract_path.read_text(encoding='utf-8'))
    if contract.get('build')!='4A070256':
        raise ValueError('Output contract does not match R40 firmware')
    return contract

DEFAULT_PROFILE = dict(name='完整反馈调试基线（尚未通过实物稳摆）',
    theta=-2843.4562, dtheta=-338.6367, phi=-154.0032, dphi=-128.2967,
    limit=150, swing=800)
FIELDS = ('ms theta dtheta phi dphi reference ref_velocity u adc zero state fault flags '
          'ack_seq ack_status bad_commands encoder target limit encoder_illegal dropped').split()
FAULT_CN = {0:'无故障',1:'摆臂超限',2:'测量无效',3:'控制更新超时',4:'跌倒保护',5:'起摆超时',6:'摆臂超速',7:'输出/运算保护',8:'120秒会话超时',9:'参考指令拒绝',10:'运动恢复失效/持续外逃保护'}
STATE_CN = {0:'待机',1:'起摆',2:'捕获',3:'保持',4:'跟踪',5:'故障',6:'接管制动',7:'零输出重获'}
ACTIVE = {1,2,3,4,6,7}
BALANCE = {2,3,4}

def output_issue(row):
    """Check telemetry command invariants; this does not measure physical PWM."""
    try:
        command_value=float(row['u']); limit=float(row['limit'])
    except (KeyError,TypeError,ValueError):
        return '输出/限幅缺少数值'
    if not math.isfinite(command_value) or not math.isfinite(limit):
        return '输出/限幅为非有限值'
    if command_value!=int(command_value) or limit!=int(limit) or not 0<=limit<=1000:
        return '输出/限幅格式或范围无效'
    if abs(command_value)>limit:
        return '输出超出本帧限幅'
    if int(row.get('state',-1))==7 and (command_value!=0 or int(row.get('flags',0))&16):
        return '零输出重获状态仍有驱动'
    return None

def storage_dir():
    """Never put user logs into a PyInstaller temporary resource directory."""
    override = os.environ.get('J280_STUDIO_DATA')
    return Path(override).expanduser() if override else Path(__file__).resolve().parents[1]/'logs'

def resource_dir():
    return Path(getattr(sys, '_MEIPASS', Path(__file__).resolve().parent))

def validate_profile(value):
    if not isinstance(value, dict):
        raise ValueError('配置必须是 JSON 对象')
    out = {'name':str(value.get('name','自定义配置'))[:120]}
    for name in ('theta','dtheta','phi','dphi','limit','swing'):
        try: v = float(value[name])
        except (KeyError,TypeError,ValueError): raise ValueError(f'配置缺少有效数值：{name}')
        if not math.isfinite(v): raise ValueError(f'{name} 不能为 NaN / Infinity')
        if name in ('limit','swing'):
            lo,hi = (20,600) if name=='limit' else (0,2000)
            if v != int(v) or not lo<=v<=hi: raise ValueError(f'{name} 必须为 {lo}～{hi} 的整数')
            out[name]=int(v)
        else:
            if abs(round(v*65536))>1000000000: raise ValueError(f'{name} 超出 FPGA 系数范围')
            out[name]=v
    return out

def csv_rows(stream):
    reader=csv.DictReader(stream)
    if not reader.fieldnames or not set(FIELDS)<=set(reader.fieldnames):
        raise ValueError('不是本队 64 字节遥测对应的 CSV：缺少字段')
    rows=[]
    for n,r in enumerate(reader,2):
        try: row={k:float(r[k]) for k in FIELDS+EXTRA_FIELDS if k in r and r[k] not in ('',None)}
        except (ValueError,TypeError): raise ValueError(f'CSV 第 {n} 行数值无效或未写完整')
        if not all(math.isfinite(v) for v in row.values()): raise ValueError(f'CSV 第 {n} 行有非有限数值')
        if int(row.get('coordinate_schema',0))==SCHEMA and not set(EXTRA_FIELDS)<=set(row):raise ValueError(f'CSV 第 {n} 行缺少新坐标的内部回放字段')
        rows.append(row)
    if not rows: raise ValueError('日志没有遥测数据')
    return rows

def load_run(path):
    path=Path(path)
    if path.suffix.lower()=='.zip':
        with zipfile.ZipFile(path) as z:
            if sum(i.file_size for i in z.infolist())>256*1024*1024:
                raise ValueError('实验包超过 256 MB，请解压后选择单个 CSV')
            names=[n for n in z.namelist() if n.endswith('.csv') and not n.startswith('__MACOSX/')]
            if len(names)!=1: raise ValueError('实验包需包含一份 CSV；多份日志请分别打开')
            name=names[0]
            rows=csv_rows(io.StringIO(z.read(name).decode('utf-8-sig')))
            stem=name[:-4]
            meta=json.loads(z.read(stem+'.meta.json')) if stem+'.meta.json' in z.namelist() else {}
            commands=[json.loads(s) for s in z.read(stem+'.commands.jsonl').decode('utf-8').splitlines()] if stem+'.commands.jsonl' in z.namelist() else []
    else:
        with path.open(encoding='utf-8-sig',newline='') as f: rows=csv_rows(f)
        mp=path.with_suffix('.meta.json'); cp=path.with_suffix('.commands.jsonl')
        meta=json.loads(mp.read_text(encoding='utf-8-sig')) if mp.exists() else {}
        commands=[json.loads(s) for s in cp.read_text(encoding='utf-8-sig').splitlines()] if cp.exists() else []
    schemas={int(row.get('coordinate_schema',0)) for row in rows}
    if len(schemas)>1:raise ValueError('记录中混入不同角度坐标，禁止合并评分')
    summary=summarize(rows,is_simulation=bool(meta.get('is_simulation')) or meta.get('kind')=='rtl_nonlinear_simulation')
    summary['coordinate_note']='摆臂为M07标定后的实际角度；内部回放字段另存' if schemas=={SCHEMA} else '历史旧摆臂坐标（未自动标定）；不要当作已校正的实际角度'
    if summary['is_simulation']:
        summary['source_label']=meta.get('source_label','RTL 非线性仿真，非实物');summary['simulation_metrics']=meta.get('simulation_metrics',{})
        summary['sensor_estimated_frames']=None  # per-row gap flags were not saved in this simulation CSV
        summary['counter_ranges']={}  # no serial/encoder health counters were recorded
        for segment in summary['captures']+summary['motions']:
            segment['control_evaluation_eligible']=False
            segment['sensor_estimated_frames']=None
            segment['evaluation_note']='模型真值与RTL输出的离线回放，不作为实物稳摆验收'
            segment['sampled_duration_note']='20 ms仿真回放抽样，末点保留；非板端串口采集'
            if segment['reason']=='日志结束，未观察到退出':segment['reason']='仿真记录结束'
    return dict(path=str(path),rows=rows,meta=meta,commands=commands,summary=summary)

def elapsed_ms(rows):
    times=[0.0]; resets=[]
    for i in range(1,len(rows)):
        delta=rows[i]['ms']-rows[i-1]['ms']
        if delta<0 and rows[i-1]['ms']>0xF0000000 and rows[i]['ms']<0x10000000: delta+=2**32
        elif delta<=0:
            resets.append(i);delta=20
        times.append(times[-1]+delta)
    return times,resets

def stationary_noise_candidate(rows,times,resets,gaps):
    """Longest >=2 s idle segment; never infer physical rod stillness.

    Use status/encoder continuity, NOT angle or velocity thresholds: otherwise
    noisy measurements would be selectively excluded from their own diagnosis.
    Missing historical fields mean unavailable, not zero noise.
    """
    boundaries=set(resets)|set(gaps); best=None; start=None
    required=('ms','state','flags','u','adc','zero','encoder','theta','dtheta')
    def eligible(row):
        try:
            if not all(math.isfinite(float(row[k])) for k in required):return False
            flags=int(row['flags'])
            return row['state']==0 and not(flags&1) and bool(flags&4) and not(flags&512) and row['u']==0
        except (KeyError,TypeError,ValueError,OverflowError):return False
    def retain(a,b):
        nonlocal best
        if a is not None and times[b-1]-times[a]>=2000:
            if best is None or times[b-1]-times[a]>times[best[1]-1]-times[best[0]]:best=(a,b)
    for i,row in enumerate(rows):
        ok=eligible(row)
        if start is not None and (not ok or i in boundaries or row['zero']!=rows[i-1]['zero'] or row['encoder']!=rows[i-1]['encoder']):
            retain(start,i);start=None
        if ok and start is None:start=i
    retain(start,len(rows))
    if best is None:return None
    a,b=best; rr=rows[a:b]
    angle=[math.degrees(r['theta']) for r in rr];mean=sum(angle)/len(angle)
    return dict(start_index=a,end_index=b,frames=len(rr),duration_s=(times[b-1]-times[a])/1000,
                adc_min=min(r['adc'] for r in rr),adc_max=max(r['adc'] for r in rr),
                theta_std_deg=math.sqrt(sum((v-mean)**2 for v in angle)/len(angle)),
                dtheta_rms_rad_s=math.sqrt(sum(r['dtheta']**2 for r in rr)/len(rr)),
                dtheta_peak_rad_s=max(abs(r['dtheta']) for r in rr),
                zero=rr[0]['zero'],encoder=rr[0]['encoder'],physical_stillness_confirmed=False,
                selection_note='候选静止段：IDLE/未使能/零输出/测量有效，编码器及零点不变，连续至少2秒；不据此认定摆杆物理静止')

def summarize(rows,is_simulation=False):
    if not rows: return {'frames':0,'captures':[],'motions':[],'resets':[],
                         'is_simulation':bool(is_simulation),'stationary_noise_candidate':None}
    times,resets=elapsed_ms(rows)
    gaps=[i for i in range(1,len(rows)) if i not in resets and times[i]-times[i-1]>30]
    captures=[]; motions=[]
    invalid_indices=[i for i,r in enumerate(rows) if output_issue(r)]
    def segments(states):
        start=None
        for i,r in enumerate(rows):
            if i in resets and start is not None:
                yield start,i,'设备复位/时间不连续';start=None
            if int(r['state']) in states:
                if start is None:start=i
            elif start is not None:
                yield start,i,FAULT_CN.get(int(r['fault']),'未知故障') if r['fault'] else '停止/状态退出'
                start=None
        if start is not None:yield start,len(rows),'日志结束，未观察到退出'
    for states,target in ((BALANCE,captures),(ACTIVE,motions)):
        for a,b,reason in segments(states):
            rr=rows[a:b]; initial=rr[0]; last_index=min(b,len(rows)-1)
            duration=(times[last_index]-times[a])/1000
            over=next(((times[a+i]-times[a])/1000 for i,r in enumerate(rr) if abs(math.degrees(r['theta']))>5),None)
            invalid=sum(output_issue(r) is not None for r in rr)
            estimated=sum(bool(int(r['flags'])&512) for r in rr)
            coast=sum(int(r['state'])==7 for r in rr)
            valid=len(rr)-invalid
            sat=sum(output_issue(r) is None and abs(r['u'])==r['limit'] and r['limit']>0 for r in rr)
            affected_gap=any(a<i<=b for i in gaps)
            target.append(dict(start_index=a,end_index=b,start_ms=initial['ms'],duration_s=duration,
                initial_theta_deg=math.degrees(initial['theta']),initial_phi_deg=math.degrees(initial['phi']),
                first_over_5deg_s=over,max_abs_theta_deg=max(abs(math.degrees(r['theta'])) for r in rr),
                rms_theta_deg=math.sqrt(sum(math.degrees(r['theta'])**2 for r in rr)/len(rr)),
                arm_displacement_deg=math.degrees(rows[last_index]['phi']-initial['phi']),
                saturated_frames=sat,active_frames=len(rr),valid_output_frames=valid,
                recovery_boost_observed_frames=sum(bool(int(r['flags'])&1024) for r in rr),
                recovery_budget_used_observed=any(int(r['flags'])&2048 for r in rr),
                settled_observed_frames=sum(bool(int(r['flags'])&4096) for r in rr),
                settle_timeout_observed=any(int(r['flags'])&8192 for r in rr),
                invalid_output_frames=invalid,sensor_estimated_frames=estimated,coast_observed_frames=coast,saturation_percent=100*sat/valid if valid else None,
                output_valid=not invalid,control_evaluation_eligible=not invalid and not affected_gap and not estimated and not coast and b<len(rows) and b not in resets,
                evaluation_note='输出状态异常：排查实现/协议，不据此评价增益' if invalid else ('采样间隔异常：本段需检查数据连续性' if affected_gap else ('包含零输出重获等待：会话仍在运行，不计作保持或有效闭环' if coast else ('包含传感盲区桥接估计：勿将这些帧视作实测角度' if estimated else '未发现上述记录异常；不自动判定稳摆通过'))),
                reason=reason,complete=b<len(rows) and b not in resets,
                sampled_duration_note='50 Hz 遥测；日志截断时只是观测时长，不代表成功'))
    return dict(frames=len(rows),duration_s=times[-1]/1000,captures=captures,motions=motions,
                is_simulation=bool(is_simulation),
                stationary_noise_candidate=None if is_simulation else stationary_noise_candidate(rows,times,resets,gaps),
                resets=resets,gap_count=len(gaps),invalid_output_frames=len(invalid_indices),
                invalid_output_indices=invalid_indices,sensor_estimated_frames=sum(bool(int(r['flags'])&512) for r in rows),
                coast_observed_frames=sum(int(r['state'])==7 for r in rows),
                counter_ranges={k:[min(r[k] for r in rows),max(r[k] for r in rows)] for k in ('bad_commands','encoder_illegal','dropped')})

def summary_text(summary):
    simulated=summary.get('is_simulation') or bool(summary.get('source_label'))
    lines=[f"{'仿真回放' if simulated else '遥测'} {summary['frames']} 帧；间隔异常 {summary.get('gap_count',0)} 处；时间重置 {len(summary.get('resets',[]))} 次。"]
    if summary.get('coordinate_note'):lines.insert(0,summary['coordinate_note'])
    if summary.get('source_label'):
        m=summary.get('simulation_metrics',{})
        lines.insert(0,summary['source_label']+' · 模型真值/RTL控制量；非串口实测')
        lines.insert(1,f"原始1ms实验运行 {m.get('duration','?')}s，首次捕获 {m.get('capture_s','?')}s；结束原因 {m.get('stop_reason','?')}")
        if 'gap_samples' in m:lines.insert(2,f"源仿真盲区桥接 {m['gap_samples']} 个1ms步，最长 {m.get('gap_max_ms','?')}ms；逐帧回放flags未记录，不能按0次解释。")
    if summary.get('sensor_estimated_frames'):lines.append(f"角度短时估计 {summary['sensor_estimated_frames']} 帧（起摆/制动中的已知盲区桥接，原始flags保留）。")
    if summary.get('coast_observed_frames'):lines.append(f"零输出重获等待 {summary['coast_observed_frames']} 个遥测帧：会话仍使能、电机输出关闭；这些帧不计作捕获/保持，50Hz记录不能证明32个连续1ms真实回读。")
    if summary.get('invalid_output_frames'):
        lines.append(f"输出异常 {summary['invalid_output_frames']} 帧：单列为实现/数据问题，不计为正常饱和。")
    noise=summary.get('stationary_noise_candidate')
    if simulated:
        lines.append('模拟数据：不计算实物候选静止段的噪声统计。')
    elif noise:
        lines.append(f"候选静止段（未使能且编码器不动，不等于摆杆物理静止）：{noise['duration_s']:.2f} s / {noise['frames']} 帧\n"
            f"  ADC {noise['adc_min']:.0f}～{noise['adc_max']:.0f}；板端角度标准差 {noise['theta_std_deg']:.4f}°\n"
            f"  板端角速度 RMS {noise['dtheta_rms_rad_s']:.4f} rad/s；绝对峰值 {noise['dtheta_peak_rad_s']:.4f} rad/s。")
    else:
        lines.append('候选静止段噪声统计：暂无连续至少2秒且满足条件的记录；不代表噪声为零。')
    for n,c in enumerate(summary['captures'],1):
        outside='未超过' if c['first_over_5deg_s'] is None else f"{c['first_over_5deg_s']:.2f} s"
        saturation='无合法输出样本' if c['saturation_percent'] is None else f"{c['saturation_percent']:.1f}%"
        lines.append(f"捕获 {n}：{c['duration_s']:.2f} s → {c['reason']}\n"
            f"  起始 θ={c['initial_theta_deg']:+.2f}° / φ={c['initial_phi_deg']:+.2f}°；首次超 ±5°：{outside}\n"
            f"  控制段最大 |θ|={c['max_abs_theta_deg']:.2f}°；RMS={c['rms_theta_deg']:.2f}°；"
            f"合法样本中饱和 {c['saturated_frames']}/{c.get('valid_output_frames',c['active_frames'])}（{saturation}）；"
            f"摆臂位移 {c['arm_displacement_deg']:+.2f}°\n"
            f"  非法输出 {c.get('invalid_output_frames',0)} 帧；{c.get('evaluation_note','')}")
    if not summary['captures']:lines.append('尚无捕获/保持/跟踪区间。')
    lines.append(('以上曲线是RTL非线性仿真；参考和输出来自RTL，位置/速度为模型真值，不代表实物验收。' if summary.get('source_label') else '以上为模拟数据，不代表实物验收。') if simulated else '时长由采样计算；保持/跟踪状态本身不代表实物稳摆已通过。')
    return '\n'.join(lines)

def archive_run(csv_path, extra_paths=()):
    """Package a finalized run without modifying source files or extracting archives."""
    p=Path(csv_path); run=load_run(p); base=p.with_suffix('')
    summary_path=Path(str(base)+'.summary.json'); summary_path.write_text(json.dumps(run['summary'],ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    text_path=Path(str(base)+'.summary.md'); text_path.write_text(summary_text(run['summary'])+'\n',encoding='utf-8')
    files=[p,p.with_suffix('.commands.jsonl'),p.with_suffix('.meta.json'),p.with_suffix('.raw_RX.bin'),summary_path,text_path,*map(Path,extra_paths)]
    files+=sorted(p.parent.glob(p.stem+'.d02_*.json'))
    files=[f for f in files if f.exists() and f.is_file()]
    output=Path(str(base)+'.zip');tmp=output.with_suffix('.zip.tmp')
    with zipfile.ZipFile(tmp,'w',zipfile.ZIP_DEFLATED) as z:
        for f in files:z.write(f,f.name)
        z.writestr('SHA256.json',json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
    tmp.replace(output)
    return output,run['summary']

class RunWriter:
    """Owned solely by serial worker; writes never run on the Qt UI thread."""
    def __init__(self,directory,metadata):
        directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
        self.path=directory/('run_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.csv')
        self.log=self.path.open('x',newline='',encoding='utf-8')
        try:self.events=self.path.with_suffix('.commands.jsonl').open('x',encoding='utf-8')
        except Exception:self.log.close();raise
        try:self.raw_rx=self.path.with_suffix('.raw_RX.bin').open('xb')
        except Exception:self.log.close();self.events.close();raise
        self.raw_bytes=0
        self.writer=csv.DictWriter(self.log,fieldnames=FIELDS+EXTRA_FIELDS);self.writer.writeheader()
        self.meta=dict(metadata,app_version=APP_VERSION,firmware_verified=False,
                       coordinate_profile_capability=PROFILE,coordinate_schema_capability=SCHEMA,
                       parameter_source='R40 控制核按明确接口使用旧内部坐标；源清单与资格回执见qualification_snapshot，非板端逐项增益读回')
        self.count=0; self.closed=False
    def rx(self,chunk):
        if chunk:self.raw_rx.write(chunk);self.raw_bytes+=len(chunk)
    def row(self,row):
        self.writer.writerow({k:row.get(k,'') for k in FIELDS+EXTRA_FIELDS});self.count+=1
        if self.count%10==0:self.log.flush()
    def event(self,event):
        self.events.write(json.dumps(event,ensure_ascii=False)+'\n');self.events.flush()
    def close(self,extra=None):
        if self.closed:return
        self.closed=True
        try:
            self.log.close();self.events.close();self.raw_rx.close()
        finally:
            self.meta.update(extra or {});self.meta['frames']=self.count;self.meta['raw_rx_bytes']=self.raw_bytes
            contract=load_output_contract(self.meta.get('firmware_readback',{}).get('build'))
            if contract is not None:
                self.meta['output_contract']=contract
                self.meta['output_contract_binding']='firmware_readback.build; not a full bitstream readback'
            self.path.with_suffix('.meta.json').write_text(json.dumps(self.meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
