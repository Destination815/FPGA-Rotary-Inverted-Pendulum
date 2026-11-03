from pathlib import Path
import argparse,hashlib,json,subprocess
ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--dependencies',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();P=a.project.resolve();D=a.dependencies.resolve();O=a.out.resolve();O.mkdir(parents=True,exist_ok=False)
cpp=r'''#include "Vcontroller.h"
#include "Vcontroller___024root.h"
#include "verilated.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#define C(n) t.rootp->controller__DOT__##n
#define R(n) t.rootp->controller__DOT__references__DOT__##n
static Vcontroller t;static int checks=0,transactions=0,maxcycles=0;
static int q(double x){return lround(x*M_PI/180/(97763./101376)*65536);}
static void ck(){t.clk=0;t.eval();t.clk=1;t.eval();}
static void need(bool x,const char*s){checks++;if(!x)throw std::runtime_error(s);}
static void init(double reference){
 t.rst_n=0;t.armed=0;t.step=t.frame_tick=0;t.request_stop=t.request_clear=t.request_start=t.request_capture=t.request_jog=t.request_target=t.request_shuttle=t.shuttle=0;
 t.measurement_ok=1;t.sensor_gap_active=0;t.sensor_gap_ms=0;t.theta=t.dtheta=t.phi=t.dphi=t.filtered_theta_q18=0;t.workspace_limit=q(360);t.motion_speed=3;t.motion_speed_limit=q(30);t.shuttle_period=35;t.shuttle_amplitude=t.ext2_amplitude=q(90);t.ext2_phase_step=227850;t.ext2_enable=0;t.full_turn_recovery=1;t.pwm_limit=150;ck();t.rst_n=1;t.armed=1;ck();
 C(state)=3;C(fault)=0;C(command)=0;C(hold_ticks)=2000;C(swing_manual)=0;C(phase)=0;C(observer_clear_q)=C(reference_clear_q)=C(swing_clear_q)=0;C(trip_code)=0;C(xt)=C(xw)=C(xp)=C(xv)=C(xf)=0;C(swing_measurement_current)=1;C(automatic_goal)=0;C(pending_target)=C(pending_shuttle)=C(saved_shuttle)=0;C(tracking_initialized)=1;C(fixed_reference)=q(reference)*4;
 R(mode)=0;R(reference)=q(reference)*4;R(ref_velocity)=R(ref_acceleration)=0;R(busy)=R(done)=0;R(age)=0;R(duration)=2000;C(track_tick)=C(swing_tick)=C(ref_tick)=0;ck();
}
static void reference_tx(){C(ref_tick)=1;ck();C(ref_tick)=0;int n=0;while(!C(ref_done)&&n++<20000)ck();need(n<20000&&!C(ref_rejected),"reference rejected/timeout");maxcycles=std::max(maxcycles,n);transactions++;ck();}
int main(){try{
 for(double actual_velocity:{-60.,-15.,0.,15.,60.})for(double target:{90.,-90.,0.}){
  init(101.830);t.phi=q(99.493);t.dphi=q(actual_velocity);C(xp)=t.phi*4;C(xv)=t.dphi*4;
  t.request_target=1;t.target=q(target);ck();t.request_target=0;need(C(r43_center_valid)&&int32_t(C(r43_center))==q(target)*4,"target not saved as session center");need(int32_t(t.ext2_entry_center_q18)==q(target)*4,"entry output lost target center");
  C(ref_reseed)=0;C(ref_target_valid)=1;reference_tx();C(ref_target_valid)=0;
  need(int32_t(R(reference))==q(99.493)*4,"target did not use actual measured position");need(R(ref_velocity)==0&&R(ref_acceleration)==0,"stationary derivative reset wrong");need(int32_t(C(xv))==q(actual_velocity)*4,"measured arm velocity was overwritten");
 }
 // Explicit target survives a fall and catch at a different location.
 for(double desired:{-90.,0.,90.}){
  init(6.677);t.target=q(desired);t.request_target=1;ck();t.request_target=0;
  C(state)=1;C(tracking_initialized)=0;C(fixed_reference)=q(6.677)*4;for(int k=0;k<12;k++)ck();need(C(r43_center_valid)&&int32_t(C(r43_center))==q(desired)*4,"fall overwrote user center");need(int32_t(t.ext2_entry_center_q18)==q(desired)*4,"entry output changed at recatch");
  C(state)=3;C(tracking_initialized)=1;R(mode)=0;R(reference)=q(6.677)*4;t.ext2_enable=1;C(ref_target_valid)=0;C(ref_shuttle_valid)=1;C(saved_shuttle)=1;C(ref_reseed)=0;reference_tx();C(ref_shuttle_valid)=0;
  need(int32_t(R(sine_center))==q(desired)*4,"sine inherited catch center");need(int32_t(R(reference))==q(6.677)*4,"sine entry jumped instead of ramping");
  t.ext2_amplitude=q(120);t.ext2_phase_step=341684;ck();need(int32_t(C(r43_center))==q(desired)*4,"parameter edit changed center");
  t.request_stop=1;ck();need(!C(r43_center_valid)&&C(r43_center)==0,"STOP retained session center");
 }
 // No prior explicit target: first centered request stores current reference.
 for(double first:{-30.,0.,31.}){
  init(first);need(!C(r43_center_valid),"unexpected initial center");need(int32_t(t.ext2_entry_center_q18)==q(first)*4,"fallback entry output not current reference");t.ext2_enable=1;C(ref_target_valid)=0;C(ref_shuttle_valid)=1;C(saved_shuttle)=1;C(ref_reseed)=0;reference_tx();C(ref_shuttle_valid)=0;
  need(C(r43_center_valid)&&int32_t(C(r43_center))==q(first)*4,"first centered start did not remember current reference");need(int32_t(R(sine_center))==q(first)*4,"first centered start wrong origin");
  C(state)=6;C(tracking_initialized)=0;C(fixed_reference)=q(8)*4;for(int k=0;k<8;k++)ck();need(int32_t(C(r43_center))==q(first)*4,"recovery changed implicit session center");
 }
 std::cout<<"{\"status\":\"PASS\",\"assertions\":"<<checks<<",\"reference_transactions\":"<<transactions<<",\"max_reference_cycles\":"<<maxcycles<<"}\n";
}catch(const std::exception&e){std::cerr<<e.what();return 1;}}
'''
(O/'main.cpp').write_text(cpp);sources=[P/'fpga/src'/f.name if f.name in ['controller.v','r01_reference.v']else f for f in sorted((D/'fpga/src').glob('*.v'))]+[D/'sim/dpb_sim_model.v'];cmd=['verilator','--cc','--exe','--build','-j','2','--public-flat-rw','--Mdir',str(O/'obj'),'--top-module','controller','-Wno-fatal','-I'+str(D/'fpga/src'),*[str(s)for s in sources],str(O/'main.cpp'),'-o','check']
with(O/'build.log').open('w')as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
r=subprocess.run([str(O/'obj/check')],capture_output=True,text=True);(O/'run.log').write_text(r.stdout+r.stderr);assert r.returncode==0,r.stderr
j=json.loads(r.stdout);j['scope']='Actual controller and reference RTL with declared internal initial state; checks measurement-velocity passthrough, center lifecycle and reference boundary. Not physical closed-loop performance.';j['sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in sources};(O/'RESULT.json').write_text(json.dumps(j,indent=2)+'\n');print(r.stdout)
