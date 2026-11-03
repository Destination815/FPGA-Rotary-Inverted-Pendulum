"""Fail-closed local qualification plus fresh board identity. STOP never blocked."""
from pathlib import Path
import hashlib,json,math,time,sys
BUILD=0x4A070257
from motion_config import validate,unpack_internal,EXT2_CONFIG
from coordinates import internal_row,target_internal_q16,SCHEMA
ALLOWED={'sine_amplitude','sine_period','ext2','motioncfg','motion_speed','stop','cal','clear','arm','start','capture','target','trajectory','d02info'}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def host_dir():
 return Path(sys.executable).resolve().parent/'host' if getattr(sys,'frozen',False) else Path(__file__).resolve().parent
def qualification(host=None):
 host=Path(host or host_dir());root=host.parent
 try:
  q=json.loads((host/'qualification.json').read_text(encoding='utf-8-sig'))
  if q.get('schema')!='r01.motion-release.v1' or q.get('build')!='4A070257' or q.get('motion_release') is not True:return False,'编译候选：运动已锁定，等待新实现 STA 审查及配套资格回执'
  manifest=root/'source_manifest.json';review=host/'sta_review.json'
  if digest(manifest)!=q['source_manifest_sha256'] or digest(review)!=q['sta_review_sha256']:return False,'资格回执与源码/时序报告不一致'
  r=json.loads(review.read_text(encoding='utf-8-sig'));m=json.loads(manifest.read_text(encoding='utf-8-sig'))
  if not r.get('accepted') or r.get('build')!='4A070257' or r.get('source_manifest_sha256')!=digest(manifest):return False,'时序审查未接受此源码版本'
  for name,sha in m['files'].items():
   path=(root/name).resolve()
   if not path.is_relative_to(root.resolve()) or digest(path)!=sha:return False,'配套文件有改变：'+name
  return True,'源码及 STA 资格已核对；仍需现场显式使能/启动'
 except (OSError,ValueError,KeyError,TypeError):return False,'资格文件缺失或无效，运动保持锁定'
def board_ok(info,fresh):
 return bool(fresh and info.get('build')==BUILD and info.get('channel')==1 and info.get('motor_sign')==-1 and info.get('coordinate_verified') is True)
def check(name,value,row,info,info_fresh,qualified):
 row=internal_row(row)
 if name=='stop':return True,''
 if name=='d02info':return True,''
 if name not in ALLOWED:return False,'R40不接受旧增益/限幅/点动/诊断命令'
 if not qualified:return False,'当前发布为仅编译候选；运动和参数操作锁定'
 if not info_fresh:return False,'板端身份回读缺失或已过期；重新查询身份并等待新鲜回读'
 if not info or info.get('build')!=BUILD or info.get('channel')!=1 or info.get('motor_sign')!=-1:
  return False,'实际板端版本/接口/方向不匹配；需要 4A070257 / XH1 / −1'
 if info.get('coordinate_verified') is not True:return False,'板端物理坐标描述缺失、过期或不一致；重新查询身份，无需反复校准'
 if name not in ('cal','clear','motioncfg','motion_speed','ext2') and row.get('phi_valid',1)!=1:return False,'物理位置超出记录格式范围；先停止'
 if name=='motioncfg':return validate(value)
 if name in ('sine_amplitude','sine_period'):
  if info.get('ext2_prepared'):return False,'正在平滑退出正弦，退出后自动应用新参数'
  if row.get('fault',1)!=0 or row.get('state') not in (0,2,3,4):return False,'等待本次起摆/恢复完成'
  if not isinstance(value,int):return False,'参数须为整数编码'
  if name=='sine_period':return (1193<=value<=4294967,'周期编码超出1～3600秒范围')
  try:bound=unpack_internal(info['motion_config'])['workspace_deg']*1144-11438;v=target_internal_q16(value)
  except (KeyError,ValueError,TypeError):return False,'缺少有效行程配置'
  return (0<v<=bound,'半幅超过已设置机械行程')
 if name=='ext2':
  if value not in (0,1):return False,'拓展2开关仅接受0或1'
  if row.get('fault',1)!=0 or row.get('state') not in (3,4) or int(row.get('flags',0))&7!=7:return False,'需要本次有效平衡状态，无需重新捕获'
  if not int(info.get('capabilities',0))&128:return False,'板端未报告拓展2能力'
  if value==1:
   if info.get('ext2_prepared'):return False,'拓展2已运行'
   if not int(row.get('flags',0))&128 or int(row.get('flags',0))&256:return False,'等待当前参考运动完成'
   try:bound=unpack_internal(info['motion_config'])['workspace_deg']*1144-11438-info.get('sine_amplitude_internal_q16',48620)
   except (KeyError,TypeError,ValueError):return False,'缺少行程配置'
   # The retained session center belongs to the board, not this telemetry reference.
   # Top validates exact Q18 center geometry before acknowledging entry.
   if bound<0:return False,'正弦半幅超出已设行程：减小半幅或修改机械行程'
  elif not info.get('ext2_prepared'):return False,'拓展2未运行'
  return True,''
 if name=='motion_speed':
  if info.get('motion_config')==EXT2_CONFIG:return False,'拓展2固定正弦不接受数值定位速度；先准备普通五功能配置'
  if not isinstance(value,int) or not round(math.radians(1)*65536)<=value<=round(math.radians(60)*65536):return False,'定位速度须为1～60物理度/秒'
  return True,''
 if info.get('ext2_prepared') and (name=='target' or (name=='trajectory' and value==1)):return False,'请先平滑退出拓展2，再直接继续定位'
 if name=='target' or (name=='trajectory' and value==1):
  cfg=info.get('motion_config')
  if cfg is None or not validate(cfg)[0]:return False,'缺少板端运动配置回读'
  bound=unpack_internal(cfg)['workspace_deg']*1144-11438
  if name=='target':
   try:value=target_internal_q16(value)
   except ValueError:return False,'目标换算溢出'
  if name=='target' and not -bound<=value<=bound:return False,'目标超出已确认行程（保留10°余量）'
 if name=='trajectory' and value==1 and abs(row.get('reference',99))>1167/(1<<16):return False,'开始往返前先定位0°，参考应在±1.02°内'
 if name=='trajectory' and value not in (0,1):return False,'往返开关仅接受 0 或 1'
 return True,''
