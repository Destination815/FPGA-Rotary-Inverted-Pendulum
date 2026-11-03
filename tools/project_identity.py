"""Read the current package identity; no local bitstream or machine binding."""
from pathlib import Path
import json,re
ROOT=Path(__file__).resolve().parents[1]

def package_identity(root=ROOT):
 root=Path(root).resolve()
 profile=json.loads((root/'calibration/encoder_calibration.json').read_text(encoding='utf-8-sig'))
 build=str(profile.get('build','')).upper().removeprefix('0X')
 if not re.fullmatch('[0-9A-F]{8}',build):raise ValueError('Invalid package coordinate build identity')
 candidate={}
 for name in ['source_manifest.json','candidate.json']:
  file=root/name
  if not file.is_file():continue
  item=json.loads(file.read_text(encoding='utf-8-sig'))
  if str(item.get('build','')).upper().removeprefix('0X')!=build:raise ValueError('Package identity mismatch: '+name)
  if name=='candidate.json':candidate=item
 header=root/'fpga/src/j280_coordinates.vh'
 if header.is_file():
  match=re.search(r"`define\s+J280_C_BUILD\s+32'h([0-9a-fA-F]{8})",header.read_text(encoding='utf-8-sig'))
  if not match or match[1].upper()!=build:raise ValueError('FPGA coordinate identity differs from package')
 entry=str(candidate.get('default_entry',''))
 match=re.fullmatch(r'start_(R[0-9A-Za-z]+)_windows\.cmd',entry)
 release=match[1] if match else 'J280_'+build
 return build,release

def current_build(root=ROOT):return int(package_identity(root)[0],16)
def current_build_text(root=ROOT):return package_identity(root)[0]
