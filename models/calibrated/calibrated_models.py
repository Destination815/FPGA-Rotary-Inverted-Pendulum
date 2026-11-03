"""Measured horizontal-encoder coordinates over the frozen M05 model backend.

This fixes the angle-unit contract, not the known high-speed model residual.
No serial access, FPGA writes, controller retuning or future state inputs.
"""
from pathlib import Path
from contextlib import contextmanager
import importlib.util,json,math,sys
import numpy as np
P=Path(__file__).resolve().parent


class Calibration:
    def __init__(self,path=None,*,basis='firmware_q24'):
        self.path=Path(path)if path else P.parents[1]/'calibration/encoder_calibration.json'
        self.profile=json.loads(self.path.read_text(encoding='utf-8'))
        if self.profile.get('schema')!='j280.coordinate-contract.c101'or self.profile.get('status')!='measured':
            raise ValueError('An explicit measured encoder profile is required')
        self.cpr=float(self.profile['counts_per_output_turn'])
        self.legacy_q16=float(self.profile['legacy_q16_per_count'])
        if not math.isfinite(self.cpr)or self.cpr<=0 or self.legacy_q16!=396:
            raise ValueError('Invalid CPR or unsupported legacy coordinate')
        self.k=self.cpr*self.legacy_q16/65536/(2*np.pi)
        if abs(self.k-float(self.profile['legacy_per_physical']))>1e-12:raise ValueError('Inconsistent encoder profile')
        self.measured_k=self.k;self.basis=basis
        if basis=='firmware_q24':self.k=self.legacy_q16*256/self.profile['physical_rpc_q24']
        elif basis!='measured':raise ValueError('Choose firmware_q24 or measured explicitly')
        self.rpc=self.legacy_q16/65536/self.k

    @staticmethod
    def _finite(value):
        x=np.asarray(value,dtype=float)
        if not np.isfinite(x).all():raise ValueError('Finite coordinates required')
        return x

    def to_physical(self,value):return self._finite(value)/self.k
    def to_legacy(self,value):return self._finite(value)*self.k

    def counts_to_physical(self,counts,*,encoder_zero):
        """Raw signed32 counts with an explicitly supplied original zero.

        Valid displacement range is signed32, matching the existing subtraction.
        No GUI saturation is applied and no unknown old zero is silently guessed.
        """
        x=self._finite(counts);z=float(encoder_zero)
        if not math.isfinite(z)or not z.is_integer()or np.any(x!=np.rint(x))or np.any(x< -2147483648)or np.any(x>2147483647)or not -2147483648<=z<=2147483647:
            raise ValueError('Signed32 integer counts and an explicit integer zero required')
        delta=(x.astype(np.int64)-int(z)+(1<<31))%(1<<32)-(1<<31)
        return delta.astype(float)*self.rpc

    def history_to_legacy(self,history):
        h=np.array(history,dtype=float,copy=True)
        if h.ndim!=3 or h.shape[-1]!=2 or not np.isfinite(h).all():raise ValueError('Finite batch x time x [theta,phi_physical] required')
        h[:,:,1]*=self.k;return h

    def prediction_to_physical(self,angles):
        y=np.array(angles,dtype=float,copy=True)
        if y.ndim!=3 or y.shape[-1]!=2 or not np.isfinite(y).all():raise ValueError('Finite batch x time x [theta,phi_legacy] required')
        y[:,:,1]/=self.k;return y


class MeasuredModels:
    """Calibrated API; unchanged past initialization, friction masks and PWM units.

    The backend keeps legacy units deliberately. Conversion happens at every
    public angle boundary; this avoids an unvalidated change in the old fit.
    """
    def __init__(self,backend_root=None,calibration=None):
        self.root=Path(backend_root).resolve()if backend_root else P/'backend'
        if not (self.root/'models.py').is_file():raise FileNotFoundError('Provide the frozen M05 backend directory')
        self.cal=calibration or Calibration()
        spec=importlib.util.spec_from_file_location('_j280_r31c1_m05_backend',self.root/'models.py')
        self.backend=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.backend)

    @contextmanager
    def _imports(self):
        # The frozen backend has absolute local imports. Reject another model
        # package instead of silently using it after a notebook import collision.
        for name in ['local_update','local_one_ms','local_core']:
            m=sys.modules.get(name)
            origins=([m.__file__] if m and getattr(m,'__file__',None) else [])+list(getattr(m,'__path__',[]) if m else [])
            if any(not Path(origin).resolve().is_relative_to(self.root) for origin in origins):
                raise RuntimeError('Conflicting backend module '+name+'; use a fresh process')
        old=sys.path[:];sys.path.insert(0,str(self.root))
        try:yield
        finally:sys.path[:]=old

    def predict(self,name,history,past_pwm,future_pwm,*,refine=1):
        h=self.cal.history_to_legacy(history)
        with self._imports():y=self.backend.predict(name,h,past_pwm,future_pwm,refine=refine)
        return self.cal.prediction_to_physical(y)

    def forecast(self,domain,history,past_pwm,future_pwm,*,refine=1):
        if domain=='local':
            angles=self.predict('local_m05',history,past_pwm,future_pwm,refine=refine);horizon=200
        elif domain=='brake':
            u=np.asarray(future_pwm,dtype=float)
            if u.ndim!=2 or u.shape[1]!=100:raise ValueError('BRAKE requires exactly 100ms of future PWM')
            angles=self.predict('handoff_domain_constrained',history,past_pwm,np.pad(u,((0,0),(0,100))),refine=refine)[:,:21];horizon=100
        elif domain=='swing':
            angles={n:self.predict(n,history,past_pwm,future_pwm,refine=refine)for n in
                    ['transmission_sensor','swing_domain_constrained','swing_domain','transmission_offset','transmission_zero_gap']};horizon=None
        else:raise ValueError('Choose local, brake or swing explicitly')
        return dict(angles=angles,domain=domain,allowed_horizon_ms=horizon,encoder_coordinate='physical_rad_M07',
                    counts_per_output_turn=self.cal.cpr,coordinate_basis=self.cal.basis,wire_rpc_q24=self.cal.profile['physical_rpc_q24'],rod_coordinate='unchanged_M05_measured_theta',
                    interpretation='Calibrated units, unchanged legacy dynamics; high-speed residuals remain')

    def plant(self,state,*,variant,sensor_zero_q4):
        return PhysicalPlant(self,state,variant=variant,sensor_zero_q4=sensor_zero_q4)


class PhysicalPlant:
    """Exact coordinate view of an existing stress plant.

    State: [theta,omega,encoder_phi,v,load_model_angle,load_model_v,torque].
    Encoder and latent load angles are expressed in the new angular unit. The
    load states remain model hypotheses; calibration did not measure load slip.
    """
    def __init__(self,models,state,*,variant,sensor_zero_q4):
        self.models=models;self.scale=np.array([1,1,1/models.cal.k,1/models.cal.k,1/models.cal.k,1/models.cal.k,1])
        x=np.asarray(state,dtype=float)
        if x.shape!=(7,)or not np.isfinite(x).all():raise ValueError('Seven finite physical-view states required')
        with models._imports():self.legacy=models.backend.Plant(x/self.scale,variant=variant,sensor_zero_q4=sensor_zero_q4)

    @property
    def state(self):return self.legacy.state.copy()*self.scale

    def step(self,pwm,dt=.001):return self.legacy.step(pwm,dt)*self.scale

    def observation(self):
        o=self.legacy.observation();o['encoder_phi']/=self.models.cal.k
        o['load_model_phi']=o.pop('load_phi')/self.models.cal.k
        o.update(encoder_coordinate='physical_rad_M07',load_state_is_independently_measured=False)
        return o
