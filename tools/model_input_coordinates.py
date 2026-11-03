"""Explicit nonmutating input views for model/replay consumers, never raw-log rewriting."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'host'))
from coordinates import SCHEMA,internal_row,EXTRA_FIELDS

def legacy_internal_rows(rows):
 """Original fitted-model inputs from legacy records OR R31C1 public rows.
 Returns derived rows; source bytes and original dictionaries are untouched.
 Old rows remain original legacy coordinates, not silently treated as physical.
 """
 out=[]
 for row in rows:
  schema=int(row.get('coordinate_schema',0))
  if schema not in (0,SCHEMA):raise ValueError('Unsupported coordinate schema')
  if schema==SCHEMA:
   required=[k+'_internal' for k in ('phi','dphi','reference') if k in row]
   if not all(k in row for k in required) or row.get('phi_valid',1)!=1:raise ValueError('Incomplete or saturated new-coordinate record')
  r=internal_row(row)
  for k in list(r):
   if k.endswith('_internal') or k in ('coordinate_schema','phi_valid'):r.pop(k)
  out.append(r)
 return out
