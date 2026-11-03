"""Atomic, board-checked motion configuration. No output or arming side effects."""
import math
from coordinates import SCALE,KAPPA,target_internal_q16
EXT2_CONFIG=75082174
EXT2_AMPLITUDE_Q16=48620
EXT2_AMPLITUDE_DEG=math.degrees(EXT2_AMPLITUDE_Q16/65536*SCALE)
PERIODS=(8,10,12,16,20,30,40,60,90,120)
DEFAULT=190|(150<<8)|(20<<21)
SPEED_MIN_DEG_S=1.0
SPEED_MAX_DEG_S=60.0
DEFAULT_SPEED_DEG_S=20*SCALE
def speed_q16(deg_s):
 if not math.isfinite(float(deg_s)) or not SPEED_MIN_DEG_S<=deg_s<=SPEED_MAX_DEG_S:raise ValueError('定位速度须为1～60物理度/秒')
 return round(math.radians(deg_s)*65536)
def speed_internal_q16(deg_s):return target_internal_q16(speed_q16(deg_s))
def unpack_internal(value):
 if value==EXT2_CONFIG:return dict(workspace_deg=190,amplitude_deg=math.degrees(48620/65536),speed=3,period_s=3.5,full_turn_recovery=False,ext2=True)
 return dict(workspace_deg=(value&255)|(((value>>29)&1)<<8),amplitude_deg=((value>>8)&2047)/10,speed=(value>>19)&3,period_s=(value>>21)&127,full_turn_recovery=bool(value&(1<<28)))
def validate(value):
 if value==EXT2_CONFIG:return True,''
 if not isinstance(value,int)or value<0 or value>>30:return False,'配置编码无效'
 c=unpack_internal(value);w,a,k,t=(c[n] for n in ('workspace_deg','amplitude_deg','speed','period_s'))
 if not 20<=w<=360 or not .1<=a<=min(204.7,w-10) or t not in PERIODS:return False,'行程20–360内部度，幅度须留10内部度余量；周期/速度档无效'
 if 9*a>[10,20,30,60][k]*t:return False,'当前幅度/周期过快；请加长周期或减小幅度'
 return True,''
def pack_internal(workspace,amplitude,speed,period,recovery=False):
 if not 20<=int(workspace)<=360:raise ValueError('机械半行程须为20–360内部度')
 v=(int(workspace)&255)|((int(workspace)>>8)<<29)|(round(amplitude*10)<<8)|(int(speed)<<19)|(int(period)<<21)|(int(bool(recovery))<<28)
 ok,why=validate(v)
 if not ok:raise ValueError(why)
 return v


def _reference_seconds_internal(delta,speed):
 """Display only; same discrete polynomial duration rule as board/reference model."""
 if speed not in (0,1,2,3):raise ValueError('speed')
 for ms in (2000,4000,6000,8000,12000,16000,20000,24000,32000,48000,64000,96000):
  t=ms/1000
  if abs(round(delta*262144))<=round(min(math.radians([10,20,30,60][speed])*t/1.875,math.radians(30)*t*t/5.774)*262144):return max(4.,t) if speed==0 else t
 return 96.


def unpack(value):
 c=unpack_internal(value)
 return dict(c,workspace_internal_deg=c['workspace_deg'],amplitude_internal_deg=c['amplitude_deg'],workspace_deg=c['workspace_deg']*SCALE,amplitude_deg=c['amplitude_deg']*SCALE,speed_deg_s=[10,20,30,60][c['speed']]*SCALE)

def pack(workspace,amplitude,speed,period,recovery=False):
 """Physical inputs, nearest original hardware bin. Readback shows applied values."""
 if not all(math.isfinite(float(x)) for x in (workspace,amplitude,speed,period)):raise ValueError('Non-finite motion setting')
 if not 20*SCALE-.000501<=workspace<=360*SCALE+.000501:raise ValueError(f'物理半行程须为{20*SCALE:.3f}～{360*SCALE:.3f}°')
 if speed!=int(speed) or period!=int(period):raise ValueError('速度档/周期须为整数')
 return pack_internal(round(workspace*KAPPA),round(amplitude*KAPPA*10)/10,int(speed),int(period),recovery)

def target_bound_deg(value):
 c=unpack_internal(value)
 # Exact old fixed-point guard, scaled to physical; stay one physical Q16 LSB inside.
 return max(0,math.degrees(((c['workspace_deg']*1144-11438)*SCALE-1)/65536))

def reference_seconds(delta,speed):return _reference_seconds_internal(delta*KAPPA,speed)

def reference_seconds_numeric(delta,deg_s):
 """Physical display estimate matching numeric velocity and retained acceleration cap."""
 v=speed_internal_q16(deg_s)*4
 d=abs(round(delta*KAPPA*262144))
 for seconds in (2,4,6,8,12,16,20,24,32,48,64,96,192,384,768,1536):
  acceleration_bound=round(math.radians(30)*seconds*seconds/5.774*262144)
  if d*15<=v*8*seconds and d<=acceleration_bound:return float(seconds)
 return 1536.
