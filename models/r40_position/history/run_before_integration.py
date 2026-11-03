#!/usr/bin/env python3
"""Portable ordinary-motion fixed-point CPU + real estimator RTL/model comparison.
This is not a complete controller RTL simulation and never opens hardware.
"""
from pathlib import Path
import argparse, ctypes as C, hashlib, json, subprocess, sys, time
import numpy as np

A = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

def command(cmd, log):
    with log.open('a') as f:
        subprocess.run(list(map(str, cmd)), check=True, stdout=f, stderr=subprocess.STDOUT)

def build(out):
    src = A / 'src'
    vr = Path(subprocess.check_output(['verilator', '--getenv', 'VERILATOR_ROOT'], text=True).strip())
    est = out / 'estimator'
    command(['verilator', '--cc', '--build', '-j', '2', '-Wno-fatal', '-CFLAGS', '-O3 -fPIC',
             '--Mdir', est, '--top-module', 'state_estimator', '-I' + str(src/'estimator'),
             src/'estimator/state_estimator.v'], out/'build.log')
    libs = {}
    for policy in ('base', 'candidate'):
        libpath = out / (policy + ('.dylib' if sys.platform == 'darwin' else '.so'))
        cmd = ['c++', '-O3', '-shared', '-fPIC', '-std=c++17', '-I'+str(src/policy),
               '-I'+str(src), '-I'+str(est), '-I'+str(vr/'include'), '-I'+str(vr/'include/vltstd')]
        if policy == 'base': cmd += ['-DBASELINE']
        command(cmd + [src/'estimator_api.cpp', est/'Vstate_estimator__ALL.a',
                       vr/'include/verilated.cpp', vr/'include/verilated_threads.cpp',
                       src/'probe.cpp', '-o', libpath], out/'build.log')
        lib = C.CDLL(str(libpath))
        ptr = C.POINTER(C.c_double)
        lib.simulate.argtypes = [ptr, ptr, C.c_int, C.c_int, C.c_int, ptr, ptr]
        libs[policy] = lib
    return libs

def metrics(c, policy, trace, fault):
    r = dict(case=c['case'], policy=policy, fault=int(fault), duration_ms=len(trace),
             active_ms=int(np.sum(trace[:,13])), peak_theta=float(max(abs(trace[:,1]))), endpoints=[])
    product = trace[:,6] * trace[:,8]
    active = trace[:,13] != 0
    paused = active & (product >= 0)
    opposed = active & (product < 0)
    r['paused_active_ms'] = int(np.sum(paused))
    r['resume_below_trigger'] = int(np.sum(paused[:-1] & opposed[1:] & (abs(trace[1:,6]) < .05)))
    for j, (st, en) in enumerate(((5000,30000), (30000,57000), (57000,80000))):
        q = trace[st:en]
        if not len(q): continue
        moving = np.flatnonzero(q[:,4] != 0)
        stop = min(len(q)-1, moving[-1]+1) if len(moving) else 0
        target = [90*c['sign'], -90*c['sign'], 0][j]
        tail, arr = q[-2000:], q[stop:stop+2000]
        r['endpoints'].append(dict(
            error_rms_deg=float(np.sqrt(np.mean((tail[:,2]-target)**2))),
            theta_rms_deg=float(np.sqrt(np.mean(tail[:,1]**2))),
            arrival2_rms_deg=float(np.sqrt(np.mean((arr[:,2]-target)**2))),
            # This legacy directional maximum includes an already-past-target start;
            # report arrival RMS as primary, never call this pure transient overshoot.
            directional_max_including_start_deg=float(max((q[:,2]-target)*(c['sign'] if j!=1 else -c['sign'])))))
    return r

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--set', choices=['regression16', 'final8'], default='final8')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--policy', choices=['both','base','candidate'], default='both')
    args = ap.parse_args(); out = args.out.resolve(); out.mkdir(parents=True, exist_ok=False)
    for file, expected in json.loads((A/'INPUT_MANIFEST.json').read_text()).items():
        assert sha(A/file) == expected, 'Frozen input mismatch: '+file
    plan_path = A/'inputs'/(args.set+'.json'); plan = json.loads(plan_path.read_text())
    libs = build(out); rows = []; ptr = C.POINTER(C.c_double); begin = time.monotonic()
    for c in plan['cases']:
        for policy in ['base','candidate'] if args.policy=='both' else [args.policy]:
            p = np.array(c['p']); cfg = np.array([1,1,1,0,5,np.radians(c['origin_deg']),.05,
                    int(c['inject_retained_integral']),c['integral_first'],c['integral_return']], float)
            result = np.zeros(2); trace = np.zeros((80000,16))
            libs[policy].simulate(p.ctypes.data_as(ptr), cfg.ctypes.data_as(ptr), c['speed'], c['sign'],
                    80000, result.ctypes.data_as(ptr), trace.ctypes.data_as(ptr))
            trace = trace[:int(result[1])]; r = metrics(c,policy,trace,result[0]); rows.append(r)
            np.savez_compressed(out/f'{c["case"]}_{policy}.npz', data=trace)
            receipt = dict(plan_sha256=sha(plan_path), input_manifest_sha256=sha(A/'INPUT_MANIFEST.json'),
                    runner_sha256=sha(__file__), cases=rows, elapsed_s=time.monotonic()-begin,
                    scope='Native fixed-point observer/CPU, real state_estimator RTL, quantized synthetic ADC Q4, M05 nonlinear plant; ideal quintic reference, declared retained-I injection at 30/57 seconds. Not full control/reference RTL and not physical performance.')
            (out/'RESULTS.json').write_text(json.dumps(receipt,indent=2))
            print(c['case'],policy,r['fault'],round(r['peak_theta'],3),r['active_ms'],r['paused_active_ms'],r['resume_below_trigger'],flush=True)
    (out/'columns.json').write_text(json.dumps(['time_s','theta_deg','phi_physical_deg','ref_deg','ref_deg_s',
            'pwm_permille','integral_normalized','motion_phase','lp_normalized','10n1','n3','omega_rad_s',
            'cap_permille','unwind_active','measured_theta_deg','measured_phi_deg'],indent=2))

if __name__ == '__main__': main()
