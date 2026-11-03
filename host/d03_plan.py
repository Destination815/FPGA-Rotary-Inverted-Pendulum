"""Guided experiment order. Advancing a card never sends a motor command."""
VALUES=(0,100,150,-100,-150,150,-150)
LABELS=('D03-00 无驱动检查','D03-01 正向 10% 基准','D03-02 正向 15% 建模',
        'D03-03 反向 10% 基准','D03-04 反向 15% 建模','D03-05 正向 15% 验证','D03-06 反向 15% 验证')

class ExperimentPlan:
    def __init__(self):
        self.index=0;self.pending=False;self.halted='';self.results=[];self.seen=set()
    @property
    def value(self):return VALUES[self.index] if self.index<len(VALUES) else None
    @property
    def label(self):return LABELS[self.index] if self.index<len(LABELS) else '本组完成，请结束并导出'
    def begin(self,value):
        if self.halted or self.pending or self.value is None or value!=self.value:
            raise ValueError('实验卡不匹配、尚在执行或本组已结束')
        self.pending=True
    def consume(self,result):
        key=(result['info']['capture_id'],result['info']['start_ms'])
        if key in self.seen:return
        if not self.pending:return  # Reading an unrelated frozen record cannot advance the plan.
        if not result.get('complete'):
            if result.get('missing',0)>0:return  # Missing packet: allow read-only retry.
            self.halted='记录存在冲突或格式错误；结束并导出';self.pending=False;return
        self.seen.add(key)
        if not result.get('acceptance',{}).get('passed'):
            self.halted='本轮中止或记录检查失败；结束并导出';self.pending=False;return
        if result['info']['amplitude']!=self.value:
            self.halted='实验卡与板端请求幅值不一致；结束并导出';self.pending=False;return
        self.results.append(result);self.pending=False;self.index+=1
        if self.index==5:  # Four driven trials complete; require both 15% trials to resolve >=8 counts.
            high=[r for r in self.results if abs(r['info']['amplitude'])==150]
            if any(abs(r['summary']['encoder_at_pulse_end_delta'])<8 for r in high):
                self.halted='至少一个 15% 试次不足 8 个计数；不做验证轮，请结束并导出'
