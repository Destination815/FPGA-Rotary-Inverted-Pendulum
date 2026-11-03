"""Read-only explanation. Does not change any command or qualification gate."""
import time
from protocol import CONTROL_BUILD
EXPECTED=CONTROL_BUILD

def describe(snapshot,now=None):
    now=time.monotonic() if now is None else now
    p=snapshot.get('d02_info') or {}
    expected=f'{EXPECTED:08X} / XH1 / −1'
    if not snapshot.get('connected'):
        return 'disconnected','未连接：连接 CH340 后读取实际板端身份。'
    if not p:
        return 'missing','尚未收到身份帧；期望 '+expected+'。点击“重新查询板端身份”，不发送运动命令。'
    build=p.get('build');actual=f'{build:08X}' if isinstance(build,int) else str(build)
    actual+=f" / XH{p.get('channel','?')} / {p.get('motor_sign','?')}"
    fresh=bool(snapshot.get('d02_fresh'))
    age=snapshot.get('identity_age_s')
    suffix=('新鲜' if fresh else '已过期')+(f'，{age:.2f}秒前' if isinstance(age,(int,float)) else '')
    base='实际板端 '+actual+'（'+suffix+'）；期望 '+expected+'。'
    if build!=EXPECTED or p.get('channel')!=1 or p.get('motor_sign')!=-1:
        return 'mismatch',base+'请停止并核对串口/版本；R36必须先取得新STA及绑定资格后才能下载，窗口标题不是板端版本。'
    if not fresh:
        return 'stale',base+'点击重新查询；仍过期则导出本轮实验包，不反复使能。'
    if not p.get('coordinate_verified'):
        return 'coordinate_missing',base+'坐标描述缺失/过期或标度不一致，运动锁定。'
    if not snapshot.get('latest') or now-snapshot.get('last_rx',0)>=.5:
        return 'telemetry_stale',base+'普通遥测过期，操作仍锁定。'
    return 'matched',base+'身份已匹配，校准/清故障继续按板端状态判断。'
