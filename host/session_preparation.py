"""Board-checked session preparation; a local bitstream file is not required."""
from motion_config import validate


def automatic_bitstream(root):
    """Compatibility API: no local file lookup, hashing, registration or claim."""
    return {}, ''


def guarded(name,value=0):
    return name in ('arm','start','capture','target') or (name in ('trajectory','ext2') and value==1)


def reviewed_hashes(snapshot):
    """Historical bitstream hashes are evidence, not a runtime prerequisite."""
    return []


def reason(preparation,expected_hashes,info,info_fresh,requested_config=None):
    chosen=preparation.get('motion_config')
    if chosen is None or not validate(chosen)[0]:return '界面运动配置无效；检查行程、幅度、速度和周期'
    if not preparation.get('range_confirmed'):return '先点击“确认现场畅通并准备本次运行”'
    if requested_config is not None and requested_config!=chosen:
        return '排队期间界面配置已改变，本次命令取消；核对后重新点击'
    if not info_fresh:return '板端配置回读已过期；等待新鲜回读再使能/启动'
    if info.get('motion_config')!=chosen:
        return '界面与板端配置不一致；停止后准备本次运行，等待配置回读一致'
    speed=preparation.get('motion_speed_internal_q16')
    if speed is not None and info.get('motion_speed_internal_q16')!=speed:
        return '定位速度尚未由板端回读确认；等待本次准备完成'
    return ''


def swing_reason(preparation, info):
    """Require the configured recovery mode and board readback, as before."""
    selected=preparation.get('motion_config')
    if not isinstance(selected,int) or not (selected & (1<<28)):
        return '自动起摆恢复配置未启用；确认本次现场畅通后停止并重新准备，等待板端回读'
    if info.get('motion_config')!=selected:
        return '自动起摆整圈配置尚未由板端确认；停止后应用并等待回读'
    return ''
