#!/usr/bin/env python3
"""Compile one frozen R43 candidate once and archive its complete vendor evidence.

No qualification, programming, serial, motion or local-bitstream gate. Original
project is read-only; Gowin runs only in a new isolated source snapshot.
"""
from pathlib import Path,PurePosixPath
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote
import argparse,datetime,hashlib,json,os,re,shutil,signal,subprocess,sys,time,zipfile
ROOT=Path(__file__).resolve().parents[1]
EXPECTED_VERSION='1.9.12.03'
EXPECTED_BUILD='4A070257'
HEARTBEAT_SECONDS=30

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def dump(p,j):Path(p).write_text(json.dumps(j,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def safe(root,name):
 n=PurePosixPath(name)
 if n.is_absolute()or '..'in n.parts or ':'in name or '\\'in name:raise ValueError('Unsafe relative source path: '+name)
 p=root/name
 if p.is_symlink()or not p.resolve().is_relative_to(root.resolve()):raise ValueError('Unsafe source/symlink: '+name)
 return p
def version_tuple(s):
 m=re.search(r'(?<!\d)(\d+)\.(\d+)\.(\d+)\.(\d+)(?!\d)',s)
 return tuple(map(int,m.groups()))if m else None

def windows_version(path):
 """Read actual executable VERSIONINFO; no executing an unverified vendor tool."""
 if os.name!='nt':return {}
 import ctypes
 from ctypes import wintypes
 v=ctypes.windll.version
 v.GetFileVersionInfoSizeW.argtypes=[wintypes.LPCWSTR,ctypes.POINTER(wintypes.DWORD)];v.GetFileVersionInfoSizeW.restype=wintypes.DWORD
 v.GetFileVersionInfoW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p];v.GetFileVersionInfoW.restype=wintypes.BOOL
 v.VerQueryValueW.argtypes=[ctypes.c_void_p,wintypes.LPCWSTR,ctypes.POINTER(ctypes.c_void_p),ctypes.POINTER(wintypes.UINT)];v.VerQueryValueW.restype=wintypes.BOOL
 dummy=wintypes.DWORD();n=v.GetFileVersionInfoSizeW(str(path),ctypes.byref(dummy))
 if not n:return {}
 buf=ctypes.create_string_buffer(n)
 if not v.GetFileVersionInfoW(str(path),0,n,buf):return {}
 out={}
 ptr=ctypes.c_void_p();size=wintypes.UINT()
 if v.VerQueryValueW(buf,'\\',ctypes.byref(ptr),ctypes.byref(size))and size.value>=52:
  class Fixed(ctypes.Structure):_fields_=[(n,wintypes.DWORD)for n in ('signature','structver','fileMS','fileLS','productMS','productLS','mask','flags','os','type','subtype','dateMS','dateLS')]
  q=ctypes.cast(ptr,ctypes.POINTER(Fixed)).contents
  for name,ms,ls in [('FileVersion',q.fileMS,q.fileLS),('ProductVersion',q.productMS,q.productLS)]:out[name]='.'.join(map(str,(ms>>16,ms&65535,ls>>16,ls&65535)))
 if v.VerQueryValueW(buf,'\\VarFileInfo\\Translation',ctypes.byref(ptr),ctypes.byref(size)):
  translations=ctypes.cast(ptr,ctypes.POINTER(ctypes.c_ushort))
  for i in range(min(size.value//4,8)):
   lang,page=translations[2*i],translations[2*i+1]
   for name in ('FileVersion','ProductVersion'):
    key=f'\\StringFileInfo\\{lang:04x}{page:04x}\\{name}'
    if v.VerQueryValueW(buf,key,ctypes.byref(ptr),ctypes.byref(size))and size.value:out[f'{name}_{lang:04x}_{page:04x}']=ctypes.wstring_at(ptr,size.value).rstrip('\x00')
 return out

def discover_gowin(explicit=None):
 paths=[]
 if explicit:paths.append(Path(explicit))
 env=os.environ.get('GOWIN_GW_SH')
 if env:paths.append(Path(env))
 paths.append(Path('C:/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'))
 for name in ('gw_sh.exe','gw_sh'):
  p=shutil.which(name)
  if p:paths.append(Path(p))
 if os.name=='nt':
  bases=[Path(x)for x in ('C:/Gowin','C:/GowinEDA','D:/Gowin','D:/GowinEDA',os.environ.get('ProgramFiles','C:/Program Files'),os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))]
  for base in bases:
   if base.is_dir():
    for pattern in ('Gowin*/IDE/bin/gw_sh.exe','Gowin*/*/IDE/bin/gw_sh.exe','*/Gowin*/IDE/bin/gw_sh.exe','IDE/bin/gw_sh.exe'):
     paths.extend(base.glob(pattern))
 found=[];seen=set()
 for path in paths:
  path=path.expanduser().resolve()
  if str(path).casefold()in seen or not path.is_file():continue
  seen.add(str(path).casefold());note=''
  try:meta=windows_version(path)
  except Exception as e:meta={};note=type(e).__name__+': '+str(e)
  expected=any(version_tuple(v)==version_tuple(EXPECTED_VERSION)for v in meta.values())
  found.append((path,meta,note,expected))
 if found:
  # Optional executable metadata must not prevent launching an existing tool.
  # Real generated synthesis/PnR report versions remain the acceptance check.
  path,meta,note,expected=next((x for x in found if x[3]),found[0])
  return dict(path=str(path),sha256=sha(path),version_info=meta,version_info_error=note,
   metadata_version_matches=expected,metadata_is_launch_gate=False,required_version=EXPECTED_VERSION,
   version_verification='actual_generated_vendor_reports_after_compile',synthetic=False)
 if os.name=='nt' and not explicit:
  import tkinter as tk
  from tkinter.filedialog import askopenfilename
  t=tk.Tk();t.withdraw()
  try:chosen=askopenfilename(title='Select existing Gowin IDE/bin/gw_sh.exe',filetypes=[('Gowin Tcl executable','gw_sh.exe')])
  finally:t.destroy()
  if chosen:return discover_gowin(chosen)
 raise RuntimeError('Gowin executable file not found. Set GOWIN_GW_SH or pass --gowin PATH; VERSIONINFO is not required.')

class Frames(HTMLParser):
 def __init__(self):super().__init__();self.refs=[]
 def handle_starttag(self,t,a):
  if t.lower()in('frame','iframe')and'src'in dict(a):self.refs.append(dict(a)['src'])

def inputs(root):
 manifest=root/'source_manifest.json';m=json.loads(manifest.read_text(encoding='utf-8-sig'));build=m['build']
 if build!=EXPECTED_BUILD:raise ValueError('Expected frozen R43 build '+EXPECTED_BUILD+', got '+str(build))
 fpga={n:h for n,h in m['files'].items()if n.startswith('fpga/')}
 if len(fpga)!=len({n.casefold()for n in fpga}):raise ValueError('Case-colliding FPGA source paths')
 for n,h in fpga.items():
  f=safe(root,n)
  if not f.is_file()or sha(f)!=h:raise ValueError('Frozen FPGA source changed/missing: '+n)
 if not fpga or not all(n in fpga for n in('fpga/j280_draft.gprj','fpga/configure_project.tcl','fpga/constraints/j280.sdc','fpga/constraints/j280.cst')):raise ValueError('Incomplete compile input manifest')
 from xml.etree import ElementTree as ET
 declared=[]
 for item in ET.parse(root/'fpga/j280_draft.gprj').iter('File'):
  if item.get('enable','1')=='0':continue
  n=(root/'fpga'/item.get('path','').replace('\\','/')).resolve()
  if not n.is_relative_to((root/'fpga').resolve())or n.relative_to(root).as_posix()not in fpga:raise ValueError('GPRJ input outside frozen FPGA source set')
  declared.append(str(n))
 if len(declared)!=len(set(declared)):raise ValueError('Duplicate GPRJ input')
 return dict(build=build,source_manifest_sha256=sha(manifest),source_sha256=fpga)

def tq(p):return '"'+str(Path(p).resolve()).replace('\\','/').replace('"','\\"').replace('$','\\$').replace('[','\\[').replace(']','\\]')+'"'
def script(work,diag):
 return '''# One frozen candidate, one run all. No seed or implementation-option search.
set evidence_dir '''+tq(diag)+'''
file mkdir $evidence_dir
open_project '''+tq(work/'fpga/j280_draft.gprj')+'''
source '''+tq(work/'fpga/configure_project.tcl')+'''
# Explicit supported physical implementation algorithm; original50MHz/CST/SDC remain.
set_option -place_option 1
set_option -route_option 0
set_option -timing_driven 1
set_option -cst_warn_to_error 1
# Preserve hold correction. Extra diagnostic exports are optional and never block run.
foreach {name value} {gen_posp 1 gen_text_timing_rpt 1 show_all_warn 1} {
 if {[catch {set_option -$name $value} note]} {puts "R43_OPTIONAL_EXPORT_UNAVAILABLE $name $note"}
}
# Save actual effective options; complete implementation must still be reviewed.
if {[catch {saveto -all_options [file join $evidence_dir effective_options.tcl]} saved]} {
 puts "R43_OPTIONS_EXPORT_UNAVAILABLE $saved"
} else {puts "R43_OPTIONS_EXPORTED"}
puts "R43_SINGLE_CANDIDATE_BEGIN"
set code [catch {run all} result options]
puts "R43_RUN_ALL_RESULT $code $result"
if {$code} {
 puts "R43_VENDOR_FLOW_FAILED $result"
 exit 2
}
puts "R43_SINGLE_CANDIDATE_FINISHED"
exit 0
'''

def screen(work,console,returncode,timed_out):
 impl=work/'fpga/impl';reports=[];errors=[];alltext=[];pnrtext='';missing_frames=[];sta=[];sta_texts=[]
 for f in sorted(impl.rglob('*'))if impl.exists()else[]:
  if not f.is_file()or f.suffix.lower()not in('.txt','.html','.htm','.log','.tr','.sta','.timing_paths'):continue
  t=f.read_text(encoding='utf-8-sig',errors='replace');n=f.relative_to(work).as_posix();reports.append(n);alltext.append(t)
  if f.name.endswith('.rpt.txt')and'/pnr/'in n:pnrtext+='\n'+t
  if f.suffix.lower()=='.log':errors.extend(dict(file=n,line=x.strip())for x in t.splitlines()if re.search(r'\bERROR\s*\(',x,re.I))
  if f.name.lower().endswith(('.tr','.tr.html','.tr.htm'))or f.name.lower().endswith('_tr_content.html'):sta.append(n);sta_texts.append(t)
  if f.suffix.lower()in('.html','.htm'):
   q=Frames();q.feed(t)
   for ref in q.refs:
    u=urlsplit(ref);target=(f.parent/unquote(u.path).replace('\\','/')).resolve()
    if u.scheme or u.netloc or not target.is_relative_to(impl.resolve())or not target.is_file():missing_frames.append(dict(file=n,reference=ref))
 text='\n'.join(alltext);log=console.read_text(encoding='utf-8-sig',errors='replace')if console.exists()else''
 errors.extend(dict(file='vendor_console.log',line=x.strip())for x in log.splitlines()if re.search(r'\bERROR\s*\(',x,re.I))
 counts=list(map(int,re.findall(r'(\d+)\s+unrouted\s+nets',text+'\n'+log,re.I)))
 complete='Placement and routing completed'in text
 sta_plain=re.sub(r'\s+',' ',re.sub(r'<[^>]*>',' ','\n'.join(sta_texts)))
 sta_labels=('Setup Delay Model','Hold Delay Model','Numbers of Paths Analyzed','Numbers of Endpoints Analyzed','Numbers of Setup Violated Endpoints','Numbers of Hold Violated Endpoints')
 sta_summary_complete=all(label in sta_plain for label in sta_labels)
 sta_counts={}
 for label in sta_labels[2:]:
  match=re.search(re.escape(label)+r'\s+(\d+)',sta_plain)
  if match:sta_counts[label]=int(match[1])
 resources={}
 for k in('Logic','Register','CLS','BSRAM','DSP','PRIMARY','LW'):
  q=re.search(r'^\s*'+k+r'\s*\|\s*(\d+)\s*/\s*(\d+)',pnrtext,re.M)
  if q:resources[k]=dict(used=int(q[1]),capacity=int(q[2]),free=int(q[2])-int(q[1]))
 versions=sorted(set(re.findall(r'V\d+\.\d+\.\d+\.\d+',text)))
 known_versions=[version_tuple(v)for v in versions]
 if timed_out or returncode!=0:status='VENDOR_PROCESS_FAILED'
 elif counts and max(counts)>0:status='ROUTING_FAILED'
 elif errors:status='VENDOR_REPORTED_ERRORS'
 elif any(v['free']<0 for v in resources.values()):status='RESOURCE_LIMIT_EXCEEDED'
 elif not complete or not sta_summary_complete or missing_frames or not all(k in resources for k in('Logic','Register','CLS','BSRAM','DSP')):status='INCOMPLETE_VENDOR_EVIDENCE'
 elif not versions or any(v!=version_tuple(EXPECTED_VERSION)for v in known_versions):status='VENDOR_VERSION_REQUIRES_REVIEW'
 else:status='COMPILED_REQUIRES_FULL_STA_REVIEW'
 return dict(status=status,process_returncode=returncode,timed_out=timed_out,reported_unrouted_nets=max(counts)if counts else None,route_completed_message=complete,resources=resources,vendor_errors=errors,reported_tool_versions=versions,STA_files=sta,full_STA_summary_labels_present=sta_summary_complete,reported_STA_summary=sta_counts,missing_STA_frames=missing_frames,all_text_reports=reports,acceptance='NO_AUTOMATIC_ACCEPTANCE_OR_QUALIFICATION',motion_release=False,local_bitstream_required=False,note='Reported count is null if tool did not explicitly report a count. Complete Setup/Hold, all timing coverage and actual routing acceptance remain B review; low Logic/CLS or CLI return0 never grants qualification.')

def archive(result_dir):
 dest=result_dir.parent/(result_dir.name+'_RETURN.zip');entries={}
 for f in sorted(result_dir.rglob('*')):
  if not f.is_file():continue
  if f.is_symlink():raise ValueError('Refuse symlink in return archive: '+str(f))
  if f.suffix.lower()=='.fs':continue
  n=f.relative_to(result_dir).as_posix();entries[n]=dict(bytes=f.stat().st_size,sha256=sha(f))
 with zipfile.ZipFile(dest,'x',zipfile.ZIP_DEFLATED,compresslevel=6)as z:
  for n,v in entries.items():
   f=result_dir/n
   if sha(f)!=v['sha256']:raise ValueError('Diagnostic output changed during archive: '+n)
   z.write(f,n)
  z.writestr('ARCHIVE_MANIFEST.json',json.dumps(dict(schema='j280.r40.compile-return.v1',files=entries,local_bitstream_required=False),ensure_ascii=False,indent=2)+'\n')
 with zipfile.ZipFile(dest)as z:
  if z.testzip()is not None:raise ValueError('Return archive CRC failed')
  for n,v in entries.items():
   h=hashlib.sha256()
   with z.open(n)as f:
    for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
   if h.hexdigest()!=v['sha256']:raise ValueError('Return archive hash failed: '+n)
 return dict(path=str(dest),sha256=sha(dest),bytes=dest.stat().st_size,entries=len(entries)+1)

def execute(root,out,tool,timeout=3600):
 """Internal injection of _fixture_command is used only by explicit software tests."""
 root=root.resolve();out=out.resolve()
 if out.is_relative_to(root):raise ValueError('Output must be outside frozen project')
 input_snapshot=inputs(root);out.mkdir(parents=True,exist_ok=False);work=out/'work';diag=out/'diagnostics';work.mkdir();diag.mkdir()
 record=dict(schema='j280.r40.one-compile.v1',build=input_snapshot['build'],compile_attempts=1,synthetic_fixture=tool.get('synthetic_fixture',False),vendor_tool={k:v for k,v in tool.items()if not k.startswith('_')},input_snapshot=input_snapshot,motion_release=False,downloaded=False,hardware_access=False,local_bitstream_required=False)
 dump(out/'INPUTS.json',record)
 for n,h in input_snapshot['source_sha256'].items():
  d=safe(work,n);d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(safe(root,n),d)
  if sha(d)!=h:raise ValueError('Source snapshot copy failed: '+n)
 shutil.copy2(root/'source_manifest.json',work/'source_manifest.json')
 tcl=out/'compile_once.tcl';tcl.write_text(script(work,diag),encoding='utf-8')
 command=tool.get('_fixture_command',[tool['path']])+[str(tcl)]
 dump(out/'ACTUAL_COMMAND.json',dict(argv=command,cwd=str(work/'fpga'),timeout_seconds=timeout,script_sha256=sha(tcl),synthetic_fixture=tool.get('synthetic_fixture',False)))
 print('Gowin executable: '+tool['path'],flush=True)
 print('Frozen build: '+input_snapshot['build']+'; ONE compile attempt, no retry/seed search.',flush=True)
 print('Result/work directory: '+str(out),flush=True)
 print('Vendor output is preserved in vendor_console.log; progress heartbeat every30 seconds.',flush=True)
 env=dict(os.environ);env['J280_COMPILE_ONCE_OUT']=str(out)
 log=out/'vendor_console.log';start=time.monotonic();code=None;timed_out=False;process=None
 try:
  with log.open('wb')as f:
   opts=dict(cwd=work/'fpga',stdout=f,stderr=subprocess.STDOUT,env=env)
   if os.name=='nt':opts['creationflags']=subprocess.CREATE_NEW_PROCESS_GROUP
   else:opts['start_new_session']=True
   process=subprocess.Popen(command,**opts)
   deadline=time.monotonic()+timeout
   try:
    while True:
     remaining=deadline-time.monotonic()
     if remaining<=0:raise subprocess.TimeoutExpired(command,timeout)
     try:code=process.wait(timeout=min(HEARTBEAT_SECONDS,remaining));break
     except subprocess.TimeoutExpired:
      if time.monotonic()>=deadline:raise
      print('['+str(int(time.monotonic()-start))+'s] Gowin仍在运行；仅此一次编译，请勿重复启动。',flush=True)
   except(subprocess.TimeoutExpired,KeyboardInterrupt):
    timed_out=True
    if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=f,stderr=subprocess.STDOUT,check=False)
    else:os.killpg(process.pid,signal.SIGKILL)
    code=process.wait()
 except OSError as e:
  log.write_text('PROCESS_LAUNCH_ERROR '+type(e).__name__+': '+str(e),encoding='utf-8');code=-1
 original_after=inputs(root)
 original_unchanged=original_after==input_snapshot
 work_after={n:sha(safe(work,n))for n in input_snapshot['source_sha256']}
 changed=[n for n,h in input_snapshot['source_sha256'].items()if work_after[n]!=h]
 unsafe_changed=[n for n in changed if not n.endswith('.gprj')]
 result=screen(work,log,code,timed_out);result.update(record)
 result.update(elapsed_seconds=round(time.monotonic()-start,3),original_source_unchanged=original_unchanged,post_run_input_snapshot=original_after,work_source_sha256_after=work_after,work_project_normalization=[n for n in changed if n.endswith('.gprj')],unsafe_work_source_changes=unsafe_changed)
 if not original_unchanged or unsafe_changed:result['status']='SOURCE_CHANGED_REQUIRES_REVIEW'
 impl=work/'fpga/impl';result['implementation_artifact_inventory']={f.relative_to(work).as_posix():dict(bytes=f.stat().st_size,sha256=sha(f))for f in sorted(impl.rglob('*'))if f.is_file()}
 result['optional_bitstream_archive_only']={n:v for n,v in result['implementation_artifact_inventory'].items()if Path(n).suffix.lower()=='.fs'}
 result['netlist_and_database_present']={k:any(Path(n).suffix.lower()==k for n in result['implementation_artifact_inventory'])for k in('.vg','.db')}
 result['compile_output_complete_scope']='All existing isolated fpga/impl files archived except optional .fs bodies; .fs hashes only, never required. No report suffix whitelist.'
 # Read-only actual mapped-memory inventory; no route/STA acceptance inferred.
 checker=root/'tools/check_r39t4_control_roms.py'
 vg=work/'fpga/impl/gwsynthesis/j280_draft.vg'
 if checker.is_file() and vg.is_file():
  try:
   part=diag/'control_roms.json'
   checkrun=subprocess.run([sys.executable,'-B',str(checker),str(vg),'--out',str(part)],capture_output=True,text=True,timeout=120)
   (diag/'control_roms_console.log').write_text(checkrun.stdout+checkrun.stderr,encoding='utf-8')
   result['control_rom_diagnostic']=json.loads(part.read_text()) if part.is_file() else dict(status='CHECKER_ERROR',returncode=checkrun.returncode)
  except Exception as e:result['control_rom_diagnostic']=dict(status='CHECKER_ERROR',error=type(e).__name__+': '+str(e))
 else:result['control_rom_diagnostic']=dict(status='NO_MAPPED_NETLIST_OR_CHECKER',source_unchanged=True)
 dump(out/'RESULT.json',result)
 (out/'结果说明.md').write_text('# R43一次真实编译回传\n\n状态：'+result['status']+'。单候选、一次run all，不扫描参数或seed；仅编译，不安装资格、不下载、不运动。\n\n原始输入和实际命令在INPUTS.json/ACTUAL_COMMAND.json，完整impl在work/fpga/impl。所有STA、映射VG、数据库、日志与有效选项均按实际生成结果归档；缺少即在RESULT标明，不伪造。低逻辑/CLS或返回码0不能单独判定通过，完整资源/布通及Setup/Hold须B审查。原项目只读；新fs仅可选哈希归档，不要求本机有或登记特定位流。\n'+('**这次为显式软件测试合成夹具，不是真实Gowin结果。**\n'if tool.get('synthetic_fixture')else''),encoding='utf-8')
 receipt=archive(out);return result,receipt

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--project',type=Path,default=ROOT);ap.add_argument('--out',type=Path);ap.add_argument('--gowin',type=Path);ap.add_argument('--timeout',type=int,default=3600)
 a=ap.parse_args();root=a.project.expanduser().resolve()
 if a.timeout<1:ap.error('timeout must be positive')
 base=(a.out or root.parent/'R43_vendor_runs').expanduser().resolve()
 if base.is_relative_to(root):ap.error('output must be outside frozen project')
 stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');out=base/('R43_compile_'+stamp)
 try:
  inputs(root)
  tool=discover_gowin(a.gowin)
  result,receipt=execute(root,out,tool,a.timeout)
 except Exception as e:
  out.mkdir(parents=True,exist_ok=True);dump(out/'SETUP_ERROR.json',dict(status='SETUP_OR_COLLECTION_ERROR',error=type(e).__name__+': '+str(e),compile_attempts=int((out/'ACTUAL_COMMAND.json').is_file()),motion_release=False,downloaded=False,hardware_access=False,local_bitstream_required=False));receipt=archive(out);result=dict(status='SETUP_OR_COLLECTION_ERROR')
 fs=out/'work/fpga/impl/pnr/j280_draft.fs'
 print('本次编译输出目录：'+str(out))
 print(('本次位流完整路径：'+str(fs)) if fs.is_file() else '本次未生成位流，请回传报告包。')
 print('Result: '+result['status']);print('Return ONE ZIP: '+receipt['path']);print('SHA256: '+receipt['sha256']);print('Compile only; no qualification, programming or motion.')
 return 0 if result['status']=='COMPILED_REQUIRES_FULL_STA_REVIEW'else 2
if __name__=='__main__':raise SystemExit(main())
