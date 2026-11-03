"""Summarize actual Gowin mapped hard memories/FFs for the two changed modules.

No placement or routing claims. Raw VG and all impl remain the source of truth.
"""
from pathlib import Path
from collections import Counter
import argparse,hashlib,json,re

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('vg',type=Path);ap.add_argument('--out',required=True,type=Path);a=ap.parse_args()
 raw=a.vg.read_bytes();s=raw.decode('utf-8-sig');modules={}
 for name in ['r10_observer_core','r01_reference']:
  m=re.search(r'^module\s+'+name+r'\b.*?^endmodule',s,re.M|re.S)
  if not m:modules[name]={'status':'MODULE_NOT_FOUND'};continue
  cells=re.findall(r'^\s{2}(\w+)\s+(\\\S+|[\w$]+)\s*\(',m[0],re.M)
  counts=Counter(t for t,n in cells)
  memories=[{'type':t,'instance':n} for t,n in cells if t in {'pROM','SP','SPX9','DPB','DPX9B','SDPB','ROM','RAM16SDP4'}]
  modules[name]={'status':'MAPPED_INVENTORY_ONLY','primitive_counts':dict(counts),'memories':memories,'flipflops':sum(n for t,n in counts.items() if t.startswith('DFF'))}
 result={'schema':'j280.r39t4.mapped-memory-inventory.v1','vg_sha256':hashlib.sha256(raw).hexdigest(),'vg_bytes':len(raw),'modules':modules,'root_cause_proven':False,'route_or_STA_acceptance':False,'local_bitstream_required':False,'scope':'Syntactic named-primitive module inventory; no inferred ROM identity from instance names, no post-place coordinates or unrouted IDs.'}
 a.out.parent.mkdir(exist_ok=True,parents=True);a.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
