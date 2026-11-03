from coordinates import internal_row,SCALE
"""Explanations only; original GUI and worker remain authoritative command gates."""
import math
import time
from protocol import CONTROL_BUILD

def explain(snapshot, checked, offline=False, now=None):
    now=time.monotonic() if now is None else now
    row=internal_row(snapshot.get('latest') or {})
    flags=int(row.get('flags',0))
    why=[]
    if snapshot.get('_preparation_issue'):why.append(snapshot['_preparation_issue'])
    if offline:why.append('正在离线回放；回到实时连接')
    if not snapshot.get('connected'):why.append('尚未连接设备')
    if not row or now-snapshot.get('last_rx',0)>=.5:why.append('遥测过期/尚未收到；等待新鲜遥测')
    if not snapshot.get('qualification_ok'):why.append('运动资格未通过：'+snapshot.get('qualification_note','请查看实验页资格'))
    info=snapshot.get('d02_info') or {}
    if not(snapshot.get('d02_fresh') and info.get('build')==CONTROL_BUILD and info.get('channel')==1 and info.get('motor_sign')==-1):
        why.append(f'板端标识未核对/过期；应为 {CONTROL_BUILD:08X} / XH1 / −1')
    if not info.get('coordinate_verified'):why.append('物理坐标描述缺失、过期或标度不一致')
    if snapshot.get('busy') or snapshot.get('d02_busy'):why.append('正在等待命令/诊断完成')
    if not checked:why.append('先点击“确认现场畅通并准备本次运行”')
    if row:
        if row.get('state')==7:why.append('自动会话正在零输出重获；此时不接受新的手动捕获')
        elif row.get('state')!=0:why.append('板端未处于 IDLE；先停止')
        if row.get('fault')!=0:why.append('存在板端故障；查看故障原因')
        if not flags&2:why.append('尚未校准；扶直静止后点击校准')
        if not flags&4:why.append('测量无效；先检查测量状态')
        if not flags&1:why.append('尚未使能：先点击上方“使能会话”，确认回执后再捕获')
        for key,limit,title,unit in [('theta',1144/65536,'摆杆角度','°'),('dtheta',13108/65536,'摆杆角速度','rad/s'),('dphi',1.,'摆臂角速度（旧内部值）','内部rad/s')]:
            value=float(row.get(key,float('nan')))
            if not math.isfinite(value):why.append(title+'数据无效')
            elif abs(value)>=limit:
                displayed=math.degrees(value) if key=='theta' else value
                bound=1 if key=='theta' else limit
                why.append(f'{title}超限：{displayed:+.3f} {unit}，要求绝对值 < {bound:g}')
    return why

def text(snapshot, checked, offline=False, now=None):
    row=internal_row(snapshot.get('latest') or {});why=explain(snapshot,checked,offline,now)
    if row.get('state')==7:
        flags=int(row.get('flags',0))
        result='自动起摆：零输出等待真实测量重获。\n会话仍使能，电机输出关闭；最长600 ms，12秒起摆预算继续计时。\n重获后板端自动续起摆；这不是保持成功，可随时STOP或SW4停止。'
        if row.get('u',0) or flags&16:result+='\n输出状态异常：请立即停止并导出。'
        if not snapshot.get('qualification_ok'):result+='\n'+snapshot.get('qualification_note','本版运动资格未通过，运动仍锁定。')
        return result
    result=('捕获暂不可用：\n'+'\n'.join('• '+x for x in why)) if why else '捕获条件满足：点击“捕获并开始计时”才启动控制。'
    if row:
        result+=f"\n当前 θ={math.degrees(row.get('theta',0)):+.2f}° / 门限 ±1°\n摆杆速度 {row.get('dtheta',0):+.3f} / ±0.200 rad/s\n摆臂实际速度 {row.get('dphi',0)*SCALE:+.3f} / ±{SCALE:.3f} rad/s"
    return result
