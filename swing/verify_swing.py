from pathlib import Path
import sys,subprocess,json,math,random,time,hashlib
import numpy as np
from swing_fixed import IntegerSwing,q,Q,generate
HERE=Path(__file__).resolve().parent
BUILD=HERE/'build';EXE=BUILD/'Vr01_swing_core'
MODEL_ROOT=HERE.parents[1] if (HERE.parents[1]/'robust_swingup_20261002').exists() else HERE.parent/'models'
sys.path.insert(0,str(HERE.parent/'models'))

from swing_fixed import CFG

def equivalence():
    commands=[];expected=[];tags=[];r=random.Random(20261002);dut=None;float_diffs=[]
    def add(cmd,tag=''):
        nonlocal dut
        v=cmd.split();op=v[0];args=list(map(int,v[1:]));commands.append(cmd);tags.append(tag)
        if op=='S':dut=IntegerSwing(args[0],bool(args[1]),args[2],args[3])
        elif op=='U':dut.update(*args)
        else:dut.stop()
        expected.append((dut.e['prevu'],dut.mode,dut.reference,int(dut.fault)))
    # Both signs: fallback at6s with deterministic high-excitation measured input;
    # sample100ms around6s captured separately in JSON.
    for sg in (-1,1):
        add(f'S {sg} 0 0 0')
        
        for j in range(8000):
            th=sg*(2.1+.9*math.sin(j*.002));ph=.3*math.sin(j*.006)
            w=sg*(2+5*math.sin(j*.017));v=.7*math.sin(j*.015)
            args=[q(x) for x in (th,ph,w,v)];add('U '+' '.join(map(str,args)),f'fallback{sg}:{j}')

        add('A')
    # Explicit brake->release,30consecutive capture, manual initialization,
    # midmicrosequence abort, illegal input, and freshsession clear.
    for sg in(-1,1):
        add(f'S {sg} 0 0 0')
        for j in range(20):add(f'U {q(sg*.2)} {q(.1)} {q(-sg*.5)} 0','brake')
        add(f'U {q(sg*.8)} 0 {q(sg*2)} 0','release')
        for j in range(60):add('U 0 0 0 0','capture')
        assert dut.mode==2
        add('X')
        add(f'S {sg} 1 {q(.2)} {q(.1)}')
        for j in range(500):add('U '+' '.join(str(q(x)) for x in (r.uniform(-.1,.1),r.uniform(-1,1),r.uniform(-2,2),r.uniform(-2,2))),'manual')
        add('U 0 0 2147483647 0','out_of_domain');assert dut.fault
        add(f'S {sg} 0 0 0');assert not dut.fault
        for j in range(1000):add('U '+' '.join(str(q(x)) for x in (r.uniform(-math.pi,math.pi),r.uniform(-1,1),r.uniform(-20,20),r.uniform(-10,10))),'random')
    # Independent transition checks, not just equality to generated arithmetic.
    add('S 1 0 0 0')
    for j in range(29):add('U 0 0 0 0','29-not-yet');assert dut.mode!=2
    add('U 0 0 0 0','30-capture');assert dut.mode==2
    add('S 1 0 0 0');add(f'U {q(.35)} 0 {q(-.5)} 0','under-energy');assert dut.mode==0
    add(f'U {q(.35)} 0 {q(-5)} 0','enough-energy');assert dut.mode==1
    exe=EXE
    proc=subprocess.run([str(exe)],input='\n'.join(commands)+'\n',text=True,capture_output=True,check=True)
    rows=[tuple(map(int,row.split())) for row in proc.stdout.splitlines()]
    assert len(rows)==len(expected)
    for i,(actual,want) in enumerate(zip(rows,expected)):
        if actual[:4]!=want:raise AssertionError((i,commands[i],tags[i],actual,want))
    summary=dict(exact_transactions=len(rows),fixed_updates=sum(c.startswith('U') for c in commands),mismatches=0,max_clocks=max(r[4] for r in rows),fallback_samples=16000)
    return summary

def closed_loop(use_rtl=False):
    from plant import nominal,step,ADC_RAD,ENC_RAD
    import torque_crosscheck as tq
    wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi
    rows=[]
    for model in('effective','mass_matrix'):
      for sg in(-1,1):
        p=nominal();plant=step if model=='effective' else tq.step
        dut=IntegerSwing(sg);rtl=None
        if use_rtl:
            rtl=subprocess.Popen([str(EXE)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            rtl.stdin.write(f'S {sg} 0 0 0\n');rtl.stdin.flush();rtl.stdout.readline()
        x=np.array([sg*(math.pi-.02),0.,0.,0.]);abpos=wrap(round(wrap(x[0])/ADC_RAD)*ADC_RAD)
        omega=vel=0.;history=np.zeros(16);cap=None;why='';trace=[];maxarm=0.;maxpost=0.
        for j in range(18000):
            th=wrap(round(wrap(x[0])/ADC_RAD)*ADC_RAD);ph=round(x[2]/ENC_RAD)*ENC_RAD
            pred=wrap(abpos+omega*.001);err=wrap(th-pred);abpos=wrap(pred+15/64*err);omega=np.clip(omega+err/.064,-30,30)
            old=history[j%16];history[j%16]=ph;vel+=.25*((ph-old)/.016-vel)
            args=[q(th),q(ph),q(omega),q(vel)]
            u=dut.update(*args)/1000
            if rtl:
                rtl.stdin.write('U '+' '.join(map(str,args))+'\n');rtl.stdin.flush()
                out=list(map(int,rtl.stdout.readline().split()))
                assert out[:4]==[dut.e['prevu'],dut.mode,dut.reference,int(dut.fault)]
                u=out[0]/1000
            if dut.mode==2 and cap is None:cap=j*.001
            xn=plant(x,u,p,.001,4);maxarm=max(maxarm,abs(xn[2]))
            if cap is not None:maxpost=max(maxpost,abs(wrap(xn[0])))
            if dut.fault:why='arith_domain'
            elif abs(xn[2])>math.radians(60):why='arm60'
            elif cap is not None and abs(wrap(xn[0]))>math.radians(15):why='fall15'
            elif cap is None and j>=12000:why='timeout12'
            trace.append([j*.001,th,ph,omega,vel,u,dut.mode]);x=xn
            if why:break
            if cap is not None and j*.001-cap>=2:break
        if rtl:rtl.stdin.close();assert rtl.wait()==0
        result=dict(controller='rtl' if use_rtl else 'integer',model=model,sign=sg,capture_s=cap,hold_s=0 if cap is None else j*.001-cap,stop_reason=why or 'completed_2s_hold',passed=cap is not None and not why,max_arm_deg=math.degrees(maxarm),max_post_theta_deg=math.degrees(maxpost))
        np.savetxt(HERE/f'closed_{"rtl" if use_rtl else "integer"}_{model}_{sg:+d}.csv',trace,delimiter=',',header='time_s,theta,phi,omega,velocity,duty,mode',comments='')
        print(json.dumps(result),flush=True);rows.append(result)
    return rows
if __name__=='__main__':
    generate();BUILD.mkdir(exist_ok=True)
    with (HERE/'build.log').open('w') as log:subprocess.run(['verilator','--cc','--exe','--build','-j','4','--language','1364-2005','--top-module','r01_swing_core','--Mdir',str(BUILD),str(HERE/'r01_swing_core.v'),str(HERE/'rtl_runner.cpp')],stdout=log,stderr=subprocess.STDOUT,check=True)
    t=time.time();r={'equivalence':equivalence(),'closed_loop':closed_loop(),'rtl_closed_loop':closed_loop(True)};r['elapsed_s']=time.time()-t
    r['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*') if p.suffix in ('.py','.v','.cpp')}
    (HERE/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
