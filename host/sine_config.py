"""Physical operator settings; FPGA generates all real-time reference samples."""
import math
from coordinates import SCALE,target_internal_q16

DEFAULT_AMPLITUDE_DEG=90.0
def settings(amplitude_deg,period_s):
    a,t=float(amplitude_deg),float(period_s)
    if not math.isfinite(a) or a<=0:raise ValueError('正弦半幅须为正数')
    if not math.isfinite(t) or not 1<=t<=3600:raise ValueError('周期可设为1～3600秒')
    q=round(math.radians(a)*65536)
    if not 0<q<(1<<31):raise ValueError('半幅超出数据表示范围')
    step=round(2**32/(1000*t))
    return dict(amplitude_physical_q16=q,amplitude_internal_q16=target_internal_q16(q),phase_step=step,
                amplitude_deg=math.degrees(target_internal_q16(q)/65536*SCALE),period_s=2**32/(1000*step))

def recommended_period(amplitude_deg,peak_speed_deg_s):
    return max(1.0,min(3600.0,round(2*math.pi*amplitude_deg/peak_speed_deg_s,2)))

def confirmed(info,selected):
    return bool(info.get('sine_settings_fresh') and
                info.get('sine_amplitude_internal_q16')==selected['amplitude_internal_q16'] and
                info.get('sine_phase_step')==selected['phase_step'])
