#!/usr/bin/env python3
"""Actual controller/CPU RTL commit/cancellation checks; no plant or hardware."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import time

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    p, out = a.project.resolve(), a.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    rtl = p / 'fpga/src'
    sources = [p/'sim/r01_control_harness.v', rtl/'controller.v', rtl/'state_estimator.v',
               *sorted(rtl.glob('r01*.v')), *sorted(rtl.glob('r10*.v')),
               *sorted(rtl.glob('r29*.v')), *sorted(rtl.glob('s23*.v')), p/'sim/dpb_sim_model.v']
    bench = p/'sim/test_r40_fixed_commit.cpp'
    bound = [*sources, *sorted(rtl.glob('*.vh')), bench, Path(__file__).resolve()]
    hashes = {f.relative_to(p).as_posix(): sha(f) for f in bound}
    started = time.monotonic()
    command = ['verilator', '--cc', '--exe', '--build', '--assert', '--public-flat-rw',
               '-j', '3', '-Wno-fatal', '--top-module', 'r01_control_harness',
               '-I'+str(rtl), '--Mdir', str(out/'obj'), *map(str, sources), str(bench), '-o', 'run']
    with (out/'build.log').open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    result = subprocess.run([str(out/'obj/run')], cwd=out, capture_output=True, text=True)
    (out/'run.log').write_text(result.stdout+result.stderr)
    changed = [n for n,h in hashes.items() if sha(p/n) != h]
    receipt = dict(status='PASS' if result.returncode == 0 and not changed else 'FAIL',
                   source_sha256=hashes, changed_during_test=changed,
                   returncode=result.returncode, elapsed_s=time.monotonic()-started,
                   output=result.stdout+result.stderr,
                   scope='Actual controller/CPU RTL; forced measurements, automatic next-frame and manual done-edge commit, STOP/disarm/reset, overrun/measurement loss, target retention and same-session sine. No plant or vendor STA.')
    (out/'RECEIPT.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    print(result.stdout+result.stderr, flush=True)
    if receipt['status'] != 'PASS':
        raise SystemExit(1)

if __name__ == '__main__':
    main()
