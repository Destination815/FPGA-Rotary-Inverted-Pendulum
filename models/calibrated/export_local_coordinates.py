"""Export calibrated local state matrices; no fitting or controller retuning."""
from pathlib import Path
import argparse,importlib.util,json
import numpy as np
from calibrated_models import Calibration
P=Path(__file__).resolve().parent


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--backend',type=Path,default=P/'backend');ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    cal=Calibration();a=1/cal.k;S=np.diag([1,1,a,a,a,a,a]);Si=np.linalg.inv(S);T=np.diag([1,a])
    path=args.backend/'local_update/grey_base.py';sp=importlib.util.spec_from_file_location('_m07_local_grey',path);g=importlib.util.module_from_spec(sp);sp.loader.exec_module(g);g.DT=.001
    bank=json.loads((args.backend/'local_update/frozen.json').read_text());q=next(c['q']for c in bank['candidates']if c['name']=='m05_past_velocity')
    plateau=json.loads((args.backend/'local_update/plateau_frozen.json').read_text())['q'];models={}
    for name,qv in [('moving',q),('plateau',plateau)]:
        A,B,C,F=g.matrices(np.asarray(qv));result=dict(A=S@A@Si,B=S@B,C=T@C@Si,F_legacy_friction_input=S@F)
        x=np.arange(7)*.07;u=.16;friction=.12
        left=S@(A@x+B*u+F*friction);right=result['A']@(S@x)+result['B']*u+result['F_legacy_friction_input']*friction
        assert np.allclose(left,right,atol=1e-13)and np.allclose(T@(C@x),result['C']@(S@x),atol=1e-13)
        models[name]={k:v.tolist()for k,v in result.items()}
    result=dict(status='PASS',dt_s=.001,state_names=['theta','omega','encoder_phi','encoder_velocity','drive_acceleration','filtered_acceleration_1','filtered_acceleration_2'],
                state_scale=S.diagonal().tolist(),measurement_scale=T.diagonal().tolist(),models=models,
                policy='Use calibrated_models API for complete initialization, friction and plateau behavior. Exported matrices alone are not a replacement predictor.',
                q_parameters_refitted=False,coordinate_basis=cal.basis)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({'status':'PASS','models':list(models),'dt_s':.001}))


if __name__=='__main__':main()
