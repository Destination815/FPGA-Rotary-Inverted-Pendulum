"""Declared metrics over source-bound complete RTL traces, no acceptance filtering."""
from pathlib import Path
import argparse, hashlib, json
import numpy as np

def rms(a):
    return float(np.sqrt(np.mean(a*a))) if len(a) else None

def theta_deg(a):
    return np.degrees(np.arctan2(np.sin(a),np.cos(a)))

def velocity_band_rms(a,low=2,high=8):
    if len(a)<2000:return None
    # Whole contiguous declared window; remove DC, Hann to reduce edge leakage.
    h=np.hanning(len(a));z=np.fft.rfft((a-a.mean())*h);f=np.fft.rfftfreq(len(a),.001)
    return float(np.sqrt(2*np.sum(abs(z[(f>=low)&(f<=high)])**2)/(len(a)*np.sum(h*h))))

def longest_true_run(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int));starts=np.flatnonzero(edges==1);stops=np.flatnonzero(edges==-1)
    if not len(starts):return None
    k=np.argmax(stops-starts);return int(starts[k]),int(stops[k])

def extrema_events(a, start, stop):
    turns=[]; tops=[]; direction=0
    for k in range(start, stop):
        row=a[k]; nd=1 if row[6]>.2 else -1 if row[6]<-.2 else direction
        if direction and nd!=direction:
            turns.append(dict(ms=k,theta_deg=float(np.degrees(row[5])),omega=float(row[6]),pwm=float(row[9])))
        direction=nd
        if k>start and row[5]*a[k-1,5]<0 and abs(row[5]-a[k-1,5])<1:
            tops.append(dict(ms=k,omega=float(row[6]),arm_v_deg_s=float(np.degrees(row[8])),pwm=float(row[9]),state=int(row[10])))
    return dict(raw_hysteretic_velocity_reversals=len(turns),turns=turns,upright_crossings_before_confirm=tops,
                count_scope='Measured omega hysteresis +/-0.2 rad/s. Unfiltered physical turns, not a relabelled user-visible full-swing count; no guarantee of <=6 swings.')

def static_window(a, lo, hi, goal):
    w=a[lo:hi]; e=np.degrees(w[:,3])-goal
    return dict(start_ms=lo,end_ms=hi,rows=len(w),position_error_mean_deg=float(np.mean(e)),position_error_rms_deg=rms(e),position_error_peak_deg=float(np.max(abs(e))),position_error_percentile95_deg=float(np.percentile(abs(e),95)),within5_fraction=float(np.mean(abs(e)<=5)),theta_rms_deg=rms(theta_deg(w[:,1])),theta_peak_deg=float(np.max(abs(theta_deg(w[:,1])))),arm_velocity_rms_deg_s=rms(np.degrees(w[:,4])),pwm_rms_permille=rms(w[:,9]*1000))

def main():
    p=argparse.ArgumentParser();p.add_argument('result',type=Path);args=p.parse_args();rp=args.result.resolve();result=json.loads(rp.read_text());base=rp.parent
    tp=base/'trace.npz'
    if not tp.exists():tp=base/'case/trace.npz'
    z=np.load(tp);a=z['data'];commits=z['commits'];out=dict(source_result=str(rp),source_trace_sha256=hashlib.sha256(tp.read_bytes()).hexdigest(),frames=len(a),fault=int(a[-1,14]),max_compute_clocks=int(max(a[:,13])),frame_budget_clocks=50000,commit_events=len(commits),commit_min=int(min(commits[:,1])) if len(commits) else None,commit_max=int(max(commits[:,1])) if len(commits) else None,model_scope=result['scope'])
    if result.get('intervention_ms') is not None:
        ev=result['intervention_ms'];start=next((x for x in result['falls_ms'] if x>=ev),ev);stop=next((x for x in result['captures_ms'] if x>ev),len(a));out['recovery']=dict(intervention_ms=ev,exit_ms=start,confirmed_ms=stop if stop<len(a) else None,recapture_after_exit_s=(stop-start)/1000 if stop<len(a) else None,**extrema_events(a,start,stop))
    positions=[]
    events=result.get('target_events',[])
    if events:
        for i,ev in enumerate(events):
            start=ev['actual_ms'];stop=events[i+1]['actual_ms'] if i+1<len(events) else len(a);goal=ev['physical_deg'];w=a[start:stop]
            # Require both zero reference speed and final commanded reference;
            # an intermediate rescue hold cannot be called the target endpoint.
            match=(abs(np.degrees(w[:,11])-goal)<.05)&(abs(w[:,12])<1e-8)
            ends=np.flatnonzero(match)
            first=next((int(k) for k in ends if k+100<=len(match) and match[k:k+100].all()),None)
            item=dict(command_ms=start,next_command_ms=stop,goal_deg=goal,reference_endpoint_ms=None if first is None else start+first,reference_reached=first is not None)
            speed=result.get('numeric_speed_deg_s')
            if speed:
                cruise=longest_true_run((abs(np.degrees(w[:,12]))>=.8*speed)&(abs(np.degrees(w[:,11])-goal)>5))
                if cruise:
                    c=w[cruise[0]:cruise[1]];vel=np.degrees(c[:,4]);rv=np.degrees(c[:,12])
                    item['cruise']=dict(window_ms=[start+cruise[0],start+cruise[1]],rows=len(c),selection='Longest contiguous reference speed >=80% requested and reference >5deg from endpoint',velocity_tracking_rms_deg_s=rms(vel-rv),velocity_2_to_8Hz_rms_deg_s=velocity_band_rms(vel),velocity_2_to_30Hz_rms_deg_s=velocity_band_rms(vel,2,30),low_velocity_fraction=float(np.mean(abs(vel)<.25*speed)),reverse_velocity_fraction=float(np.mean(vel*rv<0)),theta_rms_deg=rms(theta_deg(c[:,1])),pwm_step_rms_permille=rms(np.diff(c[:,9])*1000))
            if first is not None:
                end=start+first;lo=end+3000;hi=min(stop,end+4000)
                item['three_to_four_seconds']=static_window(a,lo,hi,goal) if hi>lo else None
                # First 1s continuously inside +/-5 after reference endpoint.
                inside=abs(np.degrees(a[end:stop,3])-goal)<=5
                bad=np.convolve((~inside).astype(int),np.ones(1000,dtype=int),mode='valid') if len(inside)>=1000 else np.array([])
                runs=np.flatnonzero(bad==0)
                item['first_continuous_1s_within5_after_endpoint_s']=int(runs[0])/1000 if len(runs) else None
                item['tail_1s']=static_window(a,max(end,stop-1000),stop,goal)
            positions.append(item)
        out['position']=positions
    if 'ext2_seen' in result:
        active=a[:,16].astype(bool);lo=np.flatnonzero(active);out['sine']=dict(active_rows=int(active.sum()),first_active_ms=int(lo[0]) if len(lo) else None,last_active_ms=int(lo[-1]) if len(lo) else None)
        if len(lo):
            start=max(int(lo[0]),12000);stop=min([int(lo[-1])+1]+[e['ms']for e in result['events']if e['command']=='ext2' and not e['value']]);w=a[start:stop];phi=np.degrees(w[:,3]);ref=np.degrees(w[:,11]);vel=np.degrees(w[:,4]);rv=np.degrees(w[:,12]);center=(max(ref)+min(ref))/2
            out['sine'].update(window_ms=[start,stop],position_rms_error_deg=rms(phi-ref),actual_positive_amplitude_deg=float(max(phi)-center),actual_negative_amplitude_deg=float(center-min(phi)),amplitude_asymmetry_deg=float(max(phi)+min(phi)-2*center),velocity_tracking_rms_deg_s=rms(vel-rv),velocity_2_to_8Hz_rms_deg_s=velocity_band_rms(vel),velocity_2_to_30Hz_rms_deg_s=velocity_band_rms(vel,2,30),theta_rms_deg=rms(np.degrees(w[:,1])))
            startup=a[int(lo[0]):start]
            if len(startup):
                # Keep initial fade/first peak visible; steady-state scoring must not hide it.
                p0=np.degrees(startup[:,3]);v0=np.degrees(startup[:,4]);t0=np.degrees(startup[:,1])
                out['sine']['startup']=dict(window_ms=[int(lo[0]),start],positive_amplitude_deg=float(max(p0)-center),negative_amplitude_deg=float(center-min(p0)),absolute_amplitude_peak_deg=float(max(abs(p0-center))),requested_amplitude_deg=result['settings']['amplitude_physical_deg'],velocity_peak_deg_s=float(max(abs(v0))),theta_peak_deg=float(max(abs(t0))),position_error_rms_deg=rms(p0-np.degrees(startup[:,11])))
    dest=base/'METRICS.json';dest.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
