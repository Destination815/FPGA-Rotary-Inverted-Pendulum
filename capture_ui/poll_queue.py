"""Preserve explicit user intent through a brief background info query.
No retries, no automatic ARM/CAPTURE; all original send-time gates remain active.
"""
from studio_session import CommandPump

class PollFriendlyPump(CommandPump):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.manual_wait_until=None

    def submit(self,requests,label):
        requests=list(requests)
        pending_info=self.pending and self.pending['request'].name=='d02info'
        if (self.busy and self.batch=='version_poll' and pending_info and not self.todo
                and ((len(requests)==1 and requests[0].name in ('arm','capture','start','cal','clear','target','trajectory')) or
                     (label=='prepare_session' and [r.name for r in requests]==['motioncfg','motion_speed','d02info']) or
                     (label in ('prepare_ext2','prepare_session_config') and [r.name for r in requests]==['motioncfg','d02info']) or
                     (label=='swing_begin' and [r.name for r in requests] in (['arm','start'],['start'])))):
            if not self.fresh:raise ValueError('尚无新鲜遥测，不能执行命令')
            self.todo.extend(requests)
            self.batch=label
            self.manual_wait_until=self.clock()+.3
            self.emit(dict(event='queued_after_info',name=requests[0].name,deadline_ms=300,
                           reason='人工点击已保留；等待当前后台查询结束，最多300ms；发送前重新核对状态'))
            return
        return super().submit(requests,label)

    def cancel(self,why):
        self.manual_wait_until=None
        return super().cancel(why)

    def tick(self):
        if self.manual_wait_until is not None and self.clock()>=self.manual_wait_until:
            self.cancel('后台查询未在300ms内完成；本次人工命令已取消，请查看连接状态')
            return
        super().tick()
        if self.pending and self.pending['request'].name!='d02info':
            self.manual_wait_until=None
        if not self.busy:self.manual_wait_until=None
