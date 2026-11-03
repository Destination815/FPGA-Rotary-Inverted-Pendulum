"""Portable M05 offline model API. No serial, FPGA access, or future measurements."""
from pathlib import Path
import ctypes as C,hashlib,json,subprocess
import numpy as np
P=Path(__file__).resolve().parent;PTR=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
def bank():return json.loads((P/'model_bank.json').read_bytes())
def build(name,horizon=200,refine=1):
 src=(P/'src'/f'{name}.cpp').read_text(encoding='utf-8')
 if horizon!=200:
  n=horizon//5;src=src.replace('i*142',f'i*{2*(31+n)}').replace('local[142]',f'local[{2*(31+n)}]').replace('j<71',f'j<{31+n}').replace('U+i*350',f'U+i*{150+horizon}').replace('i*82',f'i*{2*(n+1)}').replace('k<82',f'k<{2*(n+1)}').replace('k<=40',f'k<={n}').replace('k<40',f'k<{n}').replace('j<=40',f'j<={n}')
 if refine!=1:src=src.replace('p,.001,2)',f'p,.001,{2*refine})')
 dependency_bytes=b''.join(f.name.encode()+f.read_bytes()for f in sorted((P/'src').glob('*.cpp')))
 tag=hashlib.sha256(src.encode()+dependency_bytes).hexdigest()[:14];b=P/'build';b.mkdir(exist_ok=True);cc=b/f'{name}_{tag}.cpp';lib=b/f'{name}_{tag}.so'
 if not lib.exists():
  cc.write_text(src);subprocess.run(['c++','-O3','-std=c++17','-shared','-fPIC','-I',str(P/'src'),str(cc),'-o',str(lib)],check=True)
 obj=C.CDLL(str(lib));return obj

def predict(name,history,past_pwm,future_pwm,*,refine=1):
 """history: batch×31×2 angles every5ms, past_pwm:batch×150, future_pwm:batch×H true1ms fractions. Returns every5ms. Recorded future PWM => input replay, not closed-loop validation."""
 h=np.asarray(history,float);u0=np.asarray(past_pwm,float);u1=np.asarray(future_pwm,float)
 if h.ndim!=3 or u1.ndim!=2:raise ValueError('Expected batch×31×2 history and batch×H future PWM')
 n=len(h);H=u1.shape[1]
 if n<1 or h.shape!=(n,31,2)or u0.shape!=(n,150)or u1.shape!=(n,H)or H not in [200,500]:raise ValueError('Expected nonempty matching batches, 150ms history and 200/500ms input')
 if refine not in (1,2,4):raise ValueError('refine must be 1, 2 or 4')
 if not all(np.isfinite(x).all()for x in [h,u0,u1])or max(np.max(abs(u0)),np.max(abs(u1)))>1:raise ValueError('Nonfinite input or PWM fraction outside [-1,1]')
 if name=='local_m05':
  from local_update.api import predict as local_predict
  return local_predict(h,u0,u1,refine=refine)
 if name=='local_per_ms':
  if H!=200:raise ValueError('Local horizon validated through200ms only')
  from local_one_ms import forecast
  return forecast(np.array(bank()['models'][name]['q']),h,np.concatenate([u0,u1],axis=1))
 if name=='local_conditioned':
  if H!=200:raise ValueError('Local horizon currently validated through200ms only')
  from local_core import forecast
  Y=np.zeros((n,71,2));Y[:,:31]=h;U=np.concatenate([u0,u1],axis=1).reshape(n,70,5).mean(2);return forecast(np.array(bank()['models'][name]['q']),Y,U,0)
 if name=='M01_mass':
  model=dict(library='physical_1ms',p=json.loads((P/'baseline/physical_frozen.json').read_bytes())['selected']['p'])
 else:model=bank()['models'][name]
 lib=build(model['library'],H,refine);lib.forecast.argtypes=[PTR,PTR,PTR,PTR,C.c_int,C.c_int,PTR]
 Y=np.zeros((n,31+H//5,2));Y[:,:31]=h;U=np.concatenate([u0,u1],axis=1);V=np.array([np.polynomial.polynomial.polyfit(np.arange(9)*.005,y[:9],2)[1]for y in h]);V=np.clip(V,[-35,-20],[35,20]);out=np.empty((n,H//5+1,2));lib.forecast(np.ascontiguousarray(Y),np.ascontiguousarray(U),np.ascontiguousarray(V),np.ascontiguousarray(model['p'],dtype=float),n,1,out)
 if np.any(np.all(out==100.,axis=2)) or not np.isfinite(out).all():raise FloatingPointError('Nonlinear forecast diverged; do not score placeholder values')
 if model['library']=='transmission_sensor' or model.get('observation')=='sensor_scale_chart':
  # Frozen fitting used a single valid sensor chart. For long rollouts, apply
  # the periodic observation rather than scaling an unbounded revolution count.
  scale=model['p'][16];wrapped=np.angle(np.exp(1j*h[:,0,0]));wrapped=np.where(wrapped>1.7,wrapped-2*np.pi,wrapped)
  shift=h[:,0,0]-wrapped;true=(out[:,:,0]-shift[:,None])/scale
  alpha=(true+np.pi)%(2*np.pi)-np.pi;alpha=np.where(alpha>1.7/scale,alpha-2*np.pi,alpha)
  measured=alpha*scale;out[:,:,0]=measured+2*np.pi*np.round((out[:,:,0]-measured)/(2*np.pi))
 return out

class Plant:
 """Closed-loop-simulation stepper. State=[true rod theta,omega,encoder phi,v,load psi,vload,normalized torque]. No measured-state resets after initialization."""
 def __init__(self,state,variant=None,*,sensor_zero_q4=None):
  if variant not in ('transmission_sensor','transmission_offset','transmission_zero_gap'):raise ValueError('Choose an explicit full-angle stress model: transmission_sensor, transmission_offset or transmission_zero_gap; short-window domain fits are not physical plants')
  if sensor_zero_q4 is None:raise ValueError('Provide the actual logged sensor_zero_q4; hard-coded calibration is not valid for new sessions')
  if not np.isfinite(sensor_zero_q4) or not 0<=sensor_zero_q4<=16368:raise ValueError('sensor_zero_q4 out of ADC range')
  self.sensor_zero_q4=float(sensor_zero_q4)
  m=bank()['models'][variant];self.p=np.ascontiguousarray(m['p'],dtype=float);self.variant=variant;self.lib=build(m['library']);self.lib.tr_step.argtypes=[PTR,C.c_double,PTR,C.c_double,C.c_int];self.state=np.ascontiguousarray(state,dtype=float)
  if self.state.shape!=(7,) or not np.isfinite(self.state).all():raise ValueError('7 finite states required')
 def step(self,pwm,dt=.001):
  if not np.isfinite(pwm)or abs(pwm)>1 or not 0<dt<=.02:raise ValueError('PWM fraction or dt out of range')
  self.lib.tr_step(self.state,float(pwm),self.p,dt,max(1,int(np.ceil(dt/.0005))))
  if not np.isfinite(self.state).all():raise FloatingPointError('Plant diverged')
  return self.state.copy()
 def observation(self):
  theta=self.state[0];scale=self.p[16]if len(self.p)>16 else 1.
  # Sensor chart, not unbounded scaling of turn count. Report blind-zone validity separately.
  alpha=(theta+np.pi)%(2*np.pi)-np.pi
  if alpha>1.7/scale:alpha-=2*np.pi
  measured=scale*alpha;adc=self.sensor_zero_q4/16+measured*65536/386
  return dict(theta=float(np.angle(np.exp(1j*measured))),encoder_phi=float(self.state[2]),load_phi=float(self.state[4]),measurement_valid=7<adc<1016,adc_nominal=adc)


def free_decay(theta,omega,samples=901,dt=.02,variant='free_offset'):
 """Fixed-arm unpowered prediction; theta/omega are measured-coordinate radians.
 No state correction after start. offset and scale are competing hypotheses.
 """
 if variant not in ('free_offset','free_scale') or samples<1 or not 0<dt<=.02:raise ValueError('Invalid free-decay request')
 p=np.array(bank()['models'][variant]['p']);lib=build('free');lib.free_run.argtypes=[C.c_double,C.c_double,PTR,C.c_int,C.c_double,PTR]
 out=np.empty(samples);scale=p[4] if variant=='free_scale' else 1.
 if variant=='free_scale':p[4]=0.
 # Work on the downward sensor chart and restore the input revolution index.
 wrapped=np.angle(np.exp(1j*theta));wrapped=wrapped-2*np.pi if wrapped>1.7 else wrapped;shift=theta-wrapped
 lib.free_run(float(wrapped/scale),float(omega/scale),p,samples,dt,out)
 return out*scale+shift
