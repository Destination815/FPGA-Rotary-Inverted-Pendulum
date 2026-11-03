"""Portable API transport check. No data fitting or hardware performance claim."""
import json
from pathlib import Path
import numpy as np
from local_api import seed_from_history, Plant
from nominal_startup import NominalPrefix

for variant in ('baseline', 'updated'):
    seed = seed_from_history(np.zeros((31, 2)), np.zeros(150), variant=variant)
    p = Plant(seed)
    assert np.max(abs(p.observe())) < 1e-9
    for _ in range(20):
        assert np.isfinite(p.step(.01)).all()
p = NominalPrefix([0, 0])
q = json.loads((Path(__file__).parent / 'nominal_prefix.json').read_text())
p.step(q['logical_pwm'][0])
rejected = False
try:
    p.step(q['logical_pwm'][1] + .001)
except ValueError:
    rejected = True
assert rejected
print('PASS: two local APIs and unchanged-input nominal replay/refusal. No performance claim.')
