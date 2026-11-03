"""Portable numerical/source-interface replay, not plant performance approval."""
from pathlib import Path
import json
import numpy as np
from local_narx import LocalPlant,forecast
P=Path(__file__).resolve().parent
fixtures=json.loads((P/'fixtures.json').read_text());maxerr=0;rows=[]
for f in fixtures:
    p=forecast(f['past_angles_5ms'],f['past_duty_5ms'],f['future_known_duty_5ms'],f['variant'],pressure=True)
    err=float(np.max(abs(p-np.asarray(f['frozen_prediction']))));maxerr=max(err,maxerr)
    assert err<1e-12,(f['name'],err)
    e=p-np.asarray(f['actual_future_angles']);e[:,0]=np.arctan2(np.sin(e[:,0]),np.cos(e[:,0]))
    rows.append(dict(name=f['name'],variant=f['variant'],rms_deg=(np.sqrt(np.mean(e*e,0))*180/np.pi).tolist()))
f=fixtures[0];p=LocalPlant(f['past_angles_5ms'],f['past_duty_5ms'])
for _ in range(20):p.step5(0)
try:p.step5(0)
except ValueError:pass
else:raise AssertionError('100ms horizon limit not enforced')
for duty in [90,-90,float('nan')]:
    try:LocalPlant(f['past_angles_5ms'],f['past_duty_5ms']).step5(duty)
    except ValueError:pass
    else:raise AssertionError('Invalid duty accepted')
print(json.dumps(dict(status='PASS_PORTABLE_INTERFACE_NOT_PERFORMANCE',fixtures=len(fixtures),maximum_numerical_difference_rad=maxerr,local_error_rows=rows),indent=2))
