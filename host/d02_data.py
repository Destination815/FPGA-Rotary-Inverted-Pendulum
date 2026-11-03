"""Finite diagnostic records, isolated from live telemetry and command ACKs."""
import json
import math
import zipfile
from pathlib import Path
from protocol import D02_BUILD, D03_BUILD, DIAGNOSTIC_BUILDS

REASONS={0:'正常完成',1:'现场/软件停止',2:'测量无效',3:'摆臂相对位移超过本固件保护范围',4:'预采样期间机构移动',5:'采样更新超时'}
PHASES={0:'尚无记录',1:'预采样',2:'100 ms 点动',3:'停止后记录',4:'记录完成',5:'已中止'}

def describe(result):
    info=result.get('info',{});s=result.get('summary',{});check=result.get('acceptance',{})
    lines=[f"缓存 #{info.get('capture_id','?')} · 收到 {result.get('received',0)}/{info.get('count',0)} 点",
           REASONS.get(info.get('reason'),'未知结束原因')]
    if result.get('simulation'):lines.insert(0,'模拟数据，仅验证采集/回放，不是实物测量')
    if s:
        lines.extend([f"驱动采样 {s['drive_samples']} 点 · 最大输出 {s['max_abs_output']/10:.1f}%",
                      f"脉冲末编码器变化 {s['encoder_at_pulse_end_delta']:+d} 计数",
                      f"最大位置变化 {s['encoder_max_displacement_counts']} 计数"])
    lines.append('记录检查通过（不代表模型或稳摆验收通过）' if check.get('passed') else '记录中止/检查未通过')
    lines.extend(result.get('errors',[]));lines.extend(check.get('issues',[]))
    return '\n'.join(lines)

class Capture:
    def __init__(self,info):
        self.info=dict(info);self.rows={};self.errors=[];self.duplicates=0;self.path=None
    def feed(self,packet):
        if packet['build']!=self.info['build'] or packet['build'] not in DIAGNOSTIC_BUILDS or packet['capture_id']!=self.info['capture_id']:return
        i=packet['index'];r=packet['row']
        if not 0<=i<self.info['count'] or i!=r['sample_index']:
            self.errors.append('样本索引越界/不一致');return
        if packet['count']!=self.info['count'] or packet['reason']!=self.info['reason']:
            self.errors.append('记录元信息冲突');return
        if i in self.rows:
            self.duplicates+=1
            if self.rows[i]!=r:self.errors.append('重复样本内容冲突')
        else:self.rows[i]=r
    @property
    def complete(self):return len(self.rows)==self.info['count'] and not self.errors
    def result(self):
        rows=[self.rows[i] for i in sorted(self.rows)]
        gaps=[i for i in range(1,len(rows)) if (rows[i]['ms']-rows[i-1]['ms'])&0xffffffff!=1 or rows[i]['sample_index']!=rows[i-1]['sample_index']+1]
        errors=list(self.errors)
        if gaps:errors.append('时间/索引不连续')
        if self.info.get('interval_ms')!=1 or self.info.get('sample_bytes')!=40:errors.append('采样格式不匹配')
        if self.info['phase']==4 and self.info['count']!=1024:errors.append('正常完成但样本数不是 1024')
        out=dict(schema='j280.d02.v1',info=self.info,rows=rows,complete=self.complete and not errors,
                 received=len(rows),missing=self.info['count']-len(rows),errors=errors,duplicate_packets=self.duplicates)
        if rows:
            active=[r for r in rows if r['flags']&16]
            enc0=rows[0]['encoder'];after=next((r for r in rows if r['sample_index']==150),rows[-1])
            out['summary']=dict(reason=REASONS.get(self.info['reason'],'未知中止原因'),
                drive_samples=len(active),max_abs_output=max(abs(r['u']) for r in rows),
                encoder_at_pulse_end_delta=after['encoder']-enc0,
                encoder_max_displacement_counts=max(abs(r['encoder']-enc0) for r in rows),
                invalid_measurement_samples=sum(not(r['flags']&4) for r in rows),
                pre_encoder_span=max(r['encoder'] for r in rows[:50])-min(r['encoder'] for r in rows[:50]),
                pre_adc_span=max(r['adc'] for r in rows[:50])-min(r['adc'] for r in rows[:50]),
                initial_phi_deg=math.degrees(rows[0]['phi']),
                note='applied 是 FPGA 锁存输出，不是电流/电压；首次计数变化不是纯电机延迟。')
            checks=[]
            limit=150 if self.info['build']==D03_BUILD else 100
            amp=self.info['amplitude']
            if self.info['build'] not in DIAGNOSTIC_BUILDS:checks.append('未知固件标识')
            if amp not in ((0,100,-100,150,-150) if limit==150 else (0,100,-100)):checks.append('非法请求幅值')
            if self.info.get('limit')!=limit:checks.append('固件限幅不匹配')
            if self.info.get('channel')!=1 or self.info.get('motor_sign')!=-1:checks.append('驱动通道/方向不是已确认的 XH1 配置')
            if max(abs(r['u']) for r in rows)>limit:checks.append('输出超过诊断上限')
            for r in rows:
                i=r['sample_index']
                expected=(1 if amp>0 else -1)*min(abs(amp),20*(i-49)) if amp and 50<=i<150 else 0
                if r['u']!=expected or r['applied']!=expected*self.info.get('motor_sign',-1):
                    checks.append('输出爬升/幅值/锁存方向与请求不一致');break
                if bool(r['flags']&16)!=bool(expected):checks.append('驱动使能与请求不一致');break
            if any((r['flags']&16 or r['u']) and not 50<=r['sample_index']<150 for r in rows):checks.append('输出出现在规定点动窗口外')
            if self.info['amplitude']==0 and any(r['u'] or r['applied'] or r['flags']&16 for r in rows):checks.append('无驱动验证出现输出')
            if self.info['phase']==4 and self.info['amplitude'] and len(active)!=100:checks.append('驱动采样数不是 100')
            if any(not(r['flags']&4) for r in rows):checks.append('有测量无效样本')
            if any(r['encoder_illegal']!=rows[0]['encoder_illegal'] for r in rows):checks.append('编码器非法计数增加')
            out['acceptance']=dict(passed=out['complete'] and self.info['phase']==4 and self.info['reason']==0 and not checks,issues=checks)
        return out
    def save(self,base):
        if self.path is None:
            p=Path(base).with_suffix(f'.d02_{self.info["capture_id"]:04d}_{self.info["start_ms"]:010d}.json')
            original=p;n=2
            while p.exists():p=original.with_name(original.stem+f'_{n}.json');n+=1
            self.path=p
        tmp=self.path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(self.result(),ensure_ascii=False,indent=2)+'\n',encoding='utf-8');tmp.replace(self.path)
        return self.path

def load_captures(path):
    p=Path(path)
    if p.suffix.lower()=='.zip':
        with zipfile.ZipFile(p) as z:
            names=[n for n in z.namelist() if '.d02_' in n and n.endswith('.json')]
            if sum(z.getinfo(n).file_size for n in names)>64*1024*1024:raise ValueError('高速记录超过 64 MB')
            records=[json.loads(z.read(n)) for n in names]
    else:records=[json.loads(p.read_text(encoding='utf-8-sig'))]
    for r in records:
        if r.get('schema')!='j280.d02.v1' or len(r.get('rows',[]))>1024:raise ValueError('不是有效的 D02 文件')
        for row in r['rows']:
            if not all(math.isfinite(float(row[k])) for k in ('ms','encoder','adc','u','applied')):raise ValueError('D02 数据含非法数值')
    if not records:raise ValueError('实验包中没有 D02 高速记录')
    return records
