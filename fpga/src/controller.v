`include "j280_config.vh"
`default_nettype none
// R01 supervisor: 1 kHz snapshots, bounded sequential component evaluation.
// All physical angles at this boundary retain legacy Q16 protocol units.
module controller(input wire clk,rst_n,step,frame_tick,measurement_ok,armed,sensor_gap_active,
 input wire[7:0] sensor_gap_ms,
 input wire request_start,request_capture,request_clear,request_stop,request_jog,
 input wire request_target,request_shuttle,
 input wire signed [31:0] jog_value,theta,dtheta,phi,dphi,target,filtered_theta_q18,
 input wire shuttle,
 input wire full_turn_recovery,
 input wire signed[31:0] ext2_amplitude, input wire[31:0] ext2_phase_step,
 input wire ext2_enable, // R40 runtime profile; no session reset or recapture
 input wire signed[31:0] workspace_limit,shuttle_amplitude,
 input wire[1:0] motion_speed, input wire signed[31:0] motion_speed_limit,input wire[7:0] shuttle_period,
 input wire signed [31:0] ktheta,kdtheta,kphi,kdphi,pwm_limit,swing_gain,
 output reg [2:0] state,output reg [7:0] fault,
 output reg signed [31:0] command,
 output wire signed [31:0] reference,ref_velocity,
 output wire signed[31:0] ext2_entry_center_q18,
 output reg result_valid,output wire priority_active,reference_ready,trajectory_running,
 output wire signed [31:0] active_limit,
 output wire recovery_boosting,recovery_used,settled,settle_timeout,
 output wire handoff_recovery,
 output wire[511:0] observer_ext2,
 output wire[223:0] observer_state,output wire[7:0] observer_samples,output wire observer_ready_out,output wire signed[31:0] observer_integral,observer_lp,observer_origin,output wire[9:0] observer_age);
 localparam [2:0] IDLE=0,SWING=1,CAPTURE=2,HOLD=3,TRACK=4,FAULT=5,BRAKE=6,COAST=7;
 // Timing-only: parallel byte comparisons retain every signed32-bit result.
 function signed_gt32;
  input signed[31:0] a,b;
  begin
   signed_gt32=(a[31]!=b[31]) ? !a[31] :
    ((a[30:24]>b[30:24]) || ((a[30:24]==b[30:24]) &&
     ((a[23:16]>b[23:16]) || ((a[23:16]==b[23:16]) &&
      ((a[15:8]>b[15:8]) || ((a[15:8]==b[15:8]) && a[7:0]>b[7:0]))))));
  end
 endfunction
 function swing_range_ok;
  input signed[31:0] value;
  input[9:0] limit_value;
  reg[9:0] bounded_limit;
  reg signed[10:0] small_value;
  begin
   bounded_limit=limit_value>10'd350 ? 10'd350:limit_value;
   small_value={value[9],value[9:0]};
   swing_range_ok=(value[31:9]=={23{value[9]}}) &&
    small_value<=$signed({1'b0,bounded_limit}) &&
    small_value>=-$signed({1'b0,bounded_limit});
  end
 endfunction

 wire active=(state!=IDLE && state!=FAULT);
 reg signed [31:0] reported_limit;
 // Automatic commands are published at the next raw 1kHz frame boundary.
 // Computation completion never changes automatic actuator output.
 // The observation at a raw frame boundary describes the preceding interval.
 // Snapshot the OLD committed command before the same edge publishes pending PWM.
 reg signed[31:0] interval_command;
 // Preserve the calibrated R27 prediction timing during automatic capture.
 // A raw-frame snapshot selects completed-interval prediction only in HOLD/TRACK.
 reg prediction_use_interval;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin interval_command<=0;prediction_use_interval<=0;end
  else if(components_clear)begin interval_command<=0;prediction_use_interval<=0;end
  else if(frame_tick)begin interval_command<=command;prediction_use_interval<=(state==HOLD || state==TRACK);end
 end
 reg signed [31:0] pending_command;
 reg [9:0] pending_command_limit;
 reg pending_command_valid;
 wire automatic_frame_overrun=frame_tick && active && !swing_manual && phase!=0;
 assign active_limit=reported_limit;
 reg [3:0] phase;
 reg [16:0] watchdog;
 reg [16:0] session_ticks;
 reg [13:0] pre_capture_ticks;
 reg [11:0] hold_ticks;
 // Internal recovery can resume after actual quiet and a mature observer;
 // keep the original two-second admission for external motion commands.
 reg recovery_return_ready;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n) recovery_return_ready<=0;
  else if(components_clear || start_accepted || pre_capture || swing_manual) recovery_return_ready<=0;
  else if(step && internal_return && pending_target && hold_ticks>=500 &&
          automatic_quiet && observer_age>=400 && observer_ready) recovery_return_ready<=1;
 end
 reg [9:0] coast_age,resume_elapsed;
 reg [5:0] coast_good;
 wire coast_wait=state==COAST;
 // R24: any rejected measurement during active automatic pre-capture goes
 // through zero-drive requalification. First rail entry and a return outlier
 // need not have an active gap predictor. Persistent rejection still times out;
 // manual/captured holding keep fault2. Invalid estimates never drive the motor.
 wire coast_request=active && !swing_manual && (state==SWING || state==BRAKE) &&
                    !measurement_ok;
 wire coast_good_now=measurement_ok && !sensor_gap_active &&
       dtheta<1966080 && dtheta> -1966080 && dphi<786432 && dphi> -786432;
 reg tracking_initialized;
 reg pending_target,pending_shuttle;
 reg signed [31:0] saved_target;
 reg saved_shuttle;
 reg automatic_home_valid;reg signed[31:0] automatic_home;
 // R27: configuration is writable only while idle/unarmed. Cache both
 // bounds independently; no subtract -> negate -> compare -> mux -> equality chain.
 reg signed[31:0] home_bound,home_lower_bound;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin home_bound<=32'sd823688;home_lower_bound<=-32'sd823688;end
  else begin home_bound<=(workspace_limit-32'sd11438)<<<2;
             home_lower_bound<=(32'sd11438-workspace_limit)<<<2;end
 end
 wire catch_above=swing_ref>home_bound,catch_below=swing_ref<home_lower_bound;
 wire signed[31:0] bounded_catch_home=catch_above?home_bound:(catch_below?home_lower_bound:swing_ref);
 reg[1:0] automatic_goal; // 0 none/off, 1 user target, 2 shuttle, 3 persistent recovery home
 // Internal recovery moves wait for measured quiet; explicit user commands
 // retain their existing acceptance/trajectory rules. The live hold loop never
 // waits on this counter. Three hundred successive valid 1ms observations.
 reg[8:0] automatic_quiet_ticks;
 wire automatic_quiet=automatic_quiet_ticks>=9'd300;
 wire internal_return= !swing_manual && (automatic_goal==2 || automatic_goal==3);
 reg r37_deferred_target;
 wire pending_target_allowed=r37_deferred_target ? (!guard_active && ref_stationary) :
  (!internal_return || automatic_quiet);
 wire explicit_target=request_target && (state==HOLD || state==TRACK);
 wire explicit_shuttle=request_shuttle && (state==HOLD || state==TRACK);
 wire[1:0] recovery_goal=explicit_shuttle?(shuttle?2'd2:2'd0):(explicit_target?2'd1:automatic_goal);
 reg signed [31:0] xt,xw,xp,xv,xf;
 reg swing_measurement_current;
 reg swing_session,swing_tick,swing_manual,track_tick,track_init,ref_tick,ref_reseed;
 reg ref_target_valid,ref_shuttle_valid;
 wire[9:0]swing_limit;wire[7:0]swing_fault_code;wire[31:0]swing_cycles;
 wire swing_fault,reference_trajectory_running,swing_busy,swing_done,track_busy,track_done,track_fault,ref_busy,ref_done,ref_stationary,ref_rejected;
 wire [1:0] swing_mode;
 wire signed [31:0] swing_u,swing_ref,track_u,rp,rv,ra,rj;
 wire observer_busy,observer_done,observer_fault,observer_ready;wire signed[31:0] observer_u;
 assign observer_ready_out=observer_ready;
 // Guard acts on the completed automatic observer result. Manual timing stays exact.
 wire signed[15:0] guarded_u;
 wire guard_done,guard_reblend,guard_active;
 wire guard_start=!swing_manual && !r40_ext2_transaction && phase==4 && observer_done && !observer_result_fault && !track_fault;
 // A new timing trigger only; all old target/reference and PWM paths remain.
 // Read OLD command/reported_limit at raw frame_tick, before the same edge
 // commits pending PWM. HOLD follows the original continuous 2000ms capture.
 wire early_mature=((state==HOLD && hold_ticks>=2000) ||
  (state==CAPTURE && hold_ticks==1999)) && swing_mode==2 &&
  !swing_manual && ref_stationary && !reference_trajectory_running;
 // The 1999 raw snapshot will increment the unchanged capture counter to
 // 2000 in this frame before its policy result. This aligns the native
 // post-run mature decision without altering capture/target admission.
 // Static-reference busy is a compute transient, not actual reference motion.
 wire early_hard_cancel=components_clear || start_accepted || coast_request || coast_wait || r40_ext2_transaction ||
  trip_code!=0 || automatic_frame_overrun || swing_manual ||
  (state!=HOLD && state!=CAPTURE) || swing_mode!=2 ||
  reference_trajectory_running || (!ref_stationary && !ref_busy) ||
  request_target || request_shuttle;
 wire early_trigger;
 `include "r29_returning_gate.vh"
 wire early_force_trigger=early_trigger&&r29_returning_gate(xt,xw);
 // Attempt always retires via guard_done, even if the returning gate skips
 // the actual force. Neither event changes the old guard_active meaning.
 wire early_attempt_accept=guard_start&&early_trigger;
 wire early_force_accept=guard_start&&early_force_trigger;
 r29_early_saturation early_saturation(.clk(clk),.rst_n(rst_n),
  .clear(early_hard_cancel),.stop(request_stop),
  .frame_tick(frame_tick),.mature(early_mature),
  .previous_command(command),.reported_limit(reported_limit),
  .guard_start(guard_start),.guard_done(guard_done),
  .discard_pending(!swing_manual && phase==2 && swing_done && !auto_observer_warm),
  .early_trigger(early_trigger),
  .saturated_count(),.fired(),.pending(),.inflight(),.prior_mature());
 // R35 default-home rescue: remember the last COMPLETED guard transaction,
 // not a 50MHz edge. A CAPTURE/explicit-command activation is consumed too,
 // so becoming eligible later cannot relabel an already-active rescue as new.
 reg r35_guard_completed_active;
 // R35 v2: yield at most once after each real catch. Reuse the original
 // capture-inside |arm velocity|<1rad/s domain; fast arms keep old-position
 // braking rather than adopting a moving center. No new tuning threshold.
 reg r35_local_recenter_used;
 // Quiet-platform yielding is never a first-catch policy. This records an
 // actual subsequent confirmed default-home capture in the existing phase2
 // transaction, independently of external disturbance labels or timers.
 reg r35_recovered_catch;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)r35_recovered_catch<=0;
  else if(components_clear || start_accepted || swing_manual)r35_recovered_catch<=0;
  else if(!step && trip_code==0 && phase==2 && swing_done && !swing_fault &&
   swing_mode==2 && pre_capture && automatic_home_valid && automatic_goal==2'd3 &&
   swing_range_ok(swing_u,swing_limit))
   r35_recovered_catch<=1;
 end
 // The default recovery-home goal has no explicit positioning request.
 // If it is already truly quiet but remains > the existing5degree local
 // domain from the old point, hold here once instead of charging static I.
 // Keep out-of-home-bound recovery and every explicit motion goal intact.
 wire signed[32:0] r35_local_error_wide={xp[31],xp}-{rp[31],rp};
 wire r35_quiet_platform=r35_recovered_catch && automatic_quiet && xp>home_lower_bound && xp<home_bound &&
  (r35_local_error_wide>33'sd22876 || r35_local_error_wide< -33'sd22876);
 wire r35_new_guard=guard_active && !r35_guard_completed_active;
 wire r35_local_recenter=phase==4 && guard_done &&
  (r35_new_guard || r35_quiet_platform) && !r35_local_recenter_used &&
  xv<32'sd262144 && xv> -32'sd262144 && !swing_manual && state==HOLD &&
  hold_ticks>=2000 && swing_mode==2 && automatic_goal==2'd3 &&
  tracking_initialized && ref_stationary && !reference_trajectory_running &&
  !pending_target && !pending_shuttle && !request_target && !request_shuttle &&
  measurement_ok && swing_measurement_current && !sensor_gap_active &&
  !components_clear && !start_accepted && !coast_request && !coast_wait &&
  trip_code==0 && !automatic_frame_overrun && !observer_result_fault &&
  !track_fault && !signed_gt32(selected_u,transaction_limit) && !signed_gt32(-transaction_limit,selected_u);
 // R41: during explicit positioning OR ordinary shuttle, rescue stabilizes at current
 // bounded position. Retain the user's final target and smoothly replay it
 // only after the original guard release and stationary local reference.
 wire r37_motion_recenter=phase==4 && guard_done && r35_new_guard &&
  !swing_manual && state==TRACK && hold_ticks>=2000 && swing_mode==2 &&
  (automatic_goal==2'd1 || (automatic_goal==2'd2 && !ext2_enable)) && tracking_initialized && !ref_stationary &&
  measurement_ok && swing_measurement_current && !sensor_gap_active &&
  !pending_target && !pending_shuttle && !request_target && !request_shuttle &&
  !components_clear && !start_accepted && !coast_request && !coast_wait &&
  trip_code==0 && !automatic_frame_overrun && !observer_result_fault &&
  !signed_gt32(selected_u,transaction_limit) && !signed_gt32(-transaction_limit,selected_u);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)r37_deferred_target<=0;
  else if(components_clear || start_accepted || coast_request || coast_wait ||
          pre_capture || swing_manual || request_target || request_shuttle)
   r37_deferred_target<=0;
  else if(r37_motion_recenter)r37_deferred_target<=1;
  else if(phase==1 && tracking_initialized && pending_target && pending_target_allowed)
   r37_deferred_target<=0;
 end
 wire signed[31:0] r35_bounded_local_home=(xp>home_bound)?home_bound:
  ((xp<home_lower_bound)?home_lower_bound:xp);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)r35_guard_completed_active<=0;
  else if(components_clear || start_accepted || coast_request || coast_wait || swing_manual)
   r35_guard_completed_active<=0;
  else if(phase==4 && guard_done)r35_guard_completed_active<=guard_active;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)r35_local_recenter_used<=0;
  else if(components_clear || start_accepted || pre_capture || swing_manual)
   r35_local_recenter_used<=0;
  else if(r35_local_recenter)r35_local_recenter_used<=1;
 end
 wire observer_result_done=(swing_manual || r40_ext2_transaction)?observer_done:guard_done;
 // Reject the original full-width result before narrowing into the selector.
 wire observer_result_fault=observer_fault || (!swing_manual && observer_done &&
  (signed_gt32(observer_u,transaction_limit) || signed_gt32(-transaction_limit,observer_u) || signed_gt32(swing_u,transaction_limit) || signed_gt32(-transaction_limit,swing_u)));
 wire signed[31:0] selected_u=(swing_manual || r40_ext2_transaction)?observer_u:{{16{guarded_u[15]}},guarded_u};
 assign handoff_recovery=guard_active;
 r29_handoff_guard handoff_guard(.clk(clk),.rst_n(rst_n),
  .clear(components_clear || start_accepted || coast_request || coast_wait || swing_manual || r40_ext2_transaction),.stop(request_stop),
  .start(guard_start),.early_trigger(early_force_trigger),
  .fast_release_permission(r34_mature_static_hold && observer_ready),
  .active(auto_observer_warm && swing_mode==2 && !swing_manual),.valid(swing_measurement_current),
  .theta(xt),.omega(xw),.velocity(xv),.observer_duty(observer_u[15:0]),.fallback_duty(swing_u[15:0]),
  .duty(guarded_u),.done(guard_done),.reblend(guard_reblend),.guard_active(guard_active),.opposed_count(),.release_count());
 wire auto_observer_domain=(xt<32'sd73400 && xt> -32'sd73400);
 // R26: qualified observer also participates during in-domain BRAKE.
 // Preserve R25 target semantics: BRAKE uses current position, confirmed catch
 // freezes the capture position; observer anchor is only a coordinate origin.
 wire auto_observer_warm=(swing_mode==1 || swing_mode==2) && auto_observer_domain;
 reg observer_clear_q;
 reg automatic_observing;reg signed[31:0] automatic_anchor;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)observer_clear_q<=1;
  else observer_clear_q<=components_clear || (!swing_manual && (coast_wait || coast_request || !auto_observer_warm));
 wire signed [31:0] dbg_lean,dbg_accel,dbg_vreq,dbg_integral;
 wire components_clear=!armed || request_stop || state==FAULT || request_clear;
 reg swing_clear_q,reference_clear_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin swing_clear_q<=1;reference_clear_q<=1;end
  else begin swing_clear_q<=components_clear;reference_clear_q<=components_clear || coast_wait || coast_request;end
 end
 wire manual_ok=measurement_ok && theta<1144 && theta> -1144 && dtheta<13108 && dtheta> -13108 && dphi<65536 && dphi> -65536;
 wire start_ok=measurement_ok && (theta>163840 || theta< -163840) && dtheta<131072 && dtheta> -131072 && dphi<32768 && dphi> -32768;
 wire pre_capture=(state==SWING || state==BRAKE || state==COAST);
 wire [7:0] physical_fault=(!measurement_ok && !coast_request && !coast_wait)?8'd2:
   (((!swing_manual && full_turn_recovery && pre_capture)?(signed_gt32(phi,411775) || signed_gt32(-411775,phi)):(signed_gt32(phi,workspace_limit) || signed_gt32(-workspace_limit,phi) || (pre_capture && (signed_gt32(phi,205887) || signed_gt32(-205887,phi)))))?8'd1:
   ((signed_gt32(dphi,1310720) || signed_gt32(-1310720,dphi))?8'd6:
   ((swing_manual && !pre_capture && (signed_gt32(theta,17157) || signed_gt32(-17157,theta)))?8'd4:8'd0)));
 assign reference=coast_wait?phi:(!swing_manual && automatic_observing && !pre_capture)?(tracking_initialized?(rp>>>2):(fixed_reference>>>2)):((state==CAPTURE || pre_capture)?(swing_ref>>>2):(tracking_initialized?(rp>>>2):phi));
 assign ref_velocity=tracking_initialized?(rv>>>2):32'sd0;
 assign trajectory_running=reference_trajectory_running && tracking_initialized && (state==HOLD || state==TRACK);
 // A quiet-gated internal home/center return has not started moving yet and
 // may be replaced by an explicit command through the board/host ready gate.
 // User targets, queued shuttle requests and moving references stay blocked.
 assign reference_ready=(state==HOLD || state==TRACK) && tracking_initialized && ref_stationary &&
                        (!pending_target || (internal_return && !automatic_quiet)) && !pending_shuttle;
 assign priority_active=(state==HOLD || state==TRACK) && (xf>13107 || xf< -13107);
 // Fixed observer origin; only its feedback target moves. No reference subtraction in sensor data.
 reg signed[31:0] fixed_reference;
 assign observer_origin=(swing_manual?fixed_reference:automatic_anchor)>>>2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin automatic_observing<=0;automatic_anchor<=0;end
  else if(components_clear || start_accepted || coast_request || coast_wait)begin automatic_observing<=0;automatic_anchor<=accepted_phi;end
  else if(phase==2 && swing_done)begin
   if(!auto_observer_warm)automatic_observing<=0;
   else if(!automatic_observing)begin automatic_observing<=1;automatic_anchor<=xp;end
  end
 end
 r01_swing_core swing_core(.clk(clk),.rst_n(rst_n),.session_start(swing_session),.kick_negative(theta<0),.manual_capture(swing_manual),.resume_elapsed_ms(resume_elapsed),.tick_start(swing_tick),.stop(swing_clear_q),
 .theta_q18(xt),.phi_q18(xp),.omega_q18(xw),.velocity_q18(xv),.measurement_current(swing_measurement_current),.observer_ready(observer_ready),.previous_duty(command),.target_enable(tracking_initialized && !pre_capture),.target_position_q18(rp),.target_velocity_q18(rv),.limit_permille(swing_limit),.fault_code(swing_fault_code),.compute_cycles(swing_cycles),.busy(swing_busy),.done(swing_done),.fault(swing_fault),.duty_permille(swing_u),.reference_q18(swing_ref),.mode_out(swing_mode));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)fixed_reference<=0;
  else if(start_accepted)fixed_reference<=accepted_phi;
  else if(r37_motion_recenter || r35_local_recenter)fixed_reference<=r35_bounded_local_home;
  else if(phase==2 && swing_done && swing_mode==2 && pre_capture)fixed_reference<=swing_ref;
 end
 // R43: a user center belongs to the session, not to a later recovery catch.
 reg r43_center_valid;
 reg signed[31:0] r43_center;
 wire signed[31:0] r43_desired_center=r43_center_valid?r43_center:(tracking_initialized?rp:fixed_reference);
 assign ext2_entry_center_q18=r43_desired_center;
 // Ordinary stationary targets start at this frame's measured arm position.
 // The original stationary reference has zero velocity/acceleration; retain
 // those boundary derivatives and let measured arm-rate feedback brake motion.
 wire signed[31:0] r43_seed_phi=ref_reseed?fixed_reference:(ref_target_valid?xp:(ext2_enable?r43_desired_center:xp));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin r43_center_valid<=0;r43_center<=0;end
  else if(components_clear || start_accepted)begin r43_center_valid<=0;r43_center<=0;end
  else if(explicit_target)begin r43_center_valid<=1;r43_center<=target<<<2;end
  else if(ref_shuttle_valid && saved_shuttle && ext2_enable && !r43_center_valid)begin
   r43_center_valid<=1;r43_center<=tracking_initialized?rp:fixed_reference;
  end
 end
 r01_reference references(.clk(clk),.rst_n(rst_n),.clear(reference_clear_q),.start(ref_tick),.reseed(ref_reseed),.seed_phi(r43_seed_phi),.anchor_from_seed(1'b1),
  .amplitude(ext2_enable?(ext2_amplitude<<<2):(shuttle_amplitude<<<2)),.speed_level(ext2_enable?2'd3:motion_speed),.speed_limit(ext2_enable?32'sd0:(motion_speed_limit<<<2)),.period_seconds(ext2_enable?8'd35:shuttle_period),.target_bound(home_bound),.target_valid(ref_target_valid),.target(saved_target),.shuttle_valid(ref_shuttle_valid),.shuttle_on(saved_shuttle),
  .busy(ref_busy),.done(ref_done),.reference(rp),.ref_velocity(rv),.ref_acceleration(ra),.ref_jerk(rj),
  .stationary(ref_stationary),.trajectory_running(reference_trajectory_running),.command_rejected(ref_rejected),
  .ext2_centered(ext2_enable),.ext2_phase_step(ext2_phase_step),.ext2_center(r40_ext2_center),.ext2_ff_reference(r41t2_ext2_ff_reference));
 // R13 observer controls the first capture transaction; no parallel legacy feedback datapath.
 assign track_fault=0;assign track_busy=0;assign track_done=0;assign track_u=0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)automatic_quiet_ticks<=0;
  else if(components_clear || start_accepted || pre_capture || swing_manual)automatic_quiet_ticks<=0;
  else if(step)begin
   if(measurement_ok && !sensor_gap_active && theta<32'sd3432 && theta> -32'sd3432 &&
      dtheta<32'sd32768 && dtheta> -32'sd32768 && dphi<32'sd32768 && dphi> -32'sd32768)begin
    if(automatic_quiet_ticks<9'd300)automatic_quiet_ticks<=automatic_quiet_ticks+1'b1;
   end else automatic_quiet_ticks<=0;
  end
 end
 // Only an admitted explicit user positioning/shuttle phase can qualify
 // enhanced motion feedback. Capture/internal home return never qualify it.
 // Snapshot at the raw frame boundary, independently of compute completion.
 reg motion_assist_permission;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)motion_assist_permission<=0;
  else if(components_clear)motion_assist_permission<=0;
  else if(frame_tick)motion_assist_permission<=!swing_manual && tracking_initialized &&
   (state==HOLD || state==TRACK) &&
   (automatic_goal==2'd1 || (automatic_goal==2'd2 && reference_trajectory_running));
 end
 // R30: follow position through warm/blend and the first1s provisional catch, then
 // freeze the last sampled reference without a step to the earlier warm anchor.
 // Clearing/recovery/STOP reseeds it; confirmed capture still freezes its own goal.
 reg signed[31:0] automatic_brake_reference;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)automatic_brake_reference<=0;
  else if(components_clear || !auto_observer_warm)automatic_brake_reference<=xp;
  else if(track_tick && observer_age<10'd1000)automatic_brake_reference<=xp;
 end
 wire r34_mature_static_hold=!swing_manual && state==HOLD && hold_ticks>=16'd2000 && tracking_initialized && ref_stationary && !reference_trajectory_running && !pending_target && !pending_shuttle;
 wire r38_static_rate_hold=!swing_manual && state==HOLD && tracking_initialized && !pending_target && !pending_shuttle;
 wire r36_current_rate=swing_measurement_current&&measurement_ok&&!sensor_gap_active;
 // Keep the established observer engaged along an admitted positioning,
 // shuttle or default-home reference. Reference motion alone must not fade
 // balancing to fallback before the explicit motion-gain latch qualifies.
 // The observer domain, gain permission and independent guard remain active.
 wire r36_admitted_reference_hold=tracking_initialized && !pre_capture && automatic_goal!=0;
 // R39: the A39 sine controller is permitted only by explicit Ext2 preparation
 // and the exact original INTERNAL configuration. C101 conversion remains at
 // the public protocol/host boundary; these values must not be scaled again.
 wire signed[31:0] r40_ext2_center,r41t2_ext2_ff_reference;
 wire r39_ext2_warm=(ext2_enable===1'b1) && !pre_capture && tracking_initialized;
 // The completed result is routed by its accepted profile, not an asynchronous command.
 reg r40_ext2_transaction;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)r40_ext2_transaction<=0;
  else if(components_clear)r40_ext2_transaction<=0;
  else if(track_tick)r40_ext2_transaction<=r39_ext2_warm;
 end
 wire r39_ext2_active=r39_ext2_warm && reference_trajectory_running;
 wire r39_ext2_motion=r39_ext2_warm && !ref_stationary;
 r10_observer_core observer(.ext2_warm(r39_ext2_warm),.ext2_enable(r39_ext2_active),.ext2_motion(r39_ext2_motion),.ext2_reference(r41t2_ext2_ff_reference>>>2),.rate_hold(r38_static_rate_hold),.mature_static_hold(r34_mature_static_hold),.clk(clk),.rst_n(rst_n),.clear(observer_clear_q),.start(track_tick),.automatic_policy(!swing_manual),.motion_assist_permission(motion_assist_permission),.automatic_brake(pre_capture),.automatic_hold(auto_observer_warm && (!tracking_initialized || r36_admitted_reference_hold || (ref_stationary && (!pending_target || (internal_return && !automatic_quiet)) && !pending_shuttle))),.handoff_reblend(guard_reblend),.fallback_duty(swing_u),
 .measurement_current(r36_current_rate),.measured_omega(xw>>>2),.theta(xt>>>2),.phi((xp-(swing_manual?fixed_reference:automatic_anchor))>>>2),.reference_offset(swing_manual?(tracking_initialized?((rp-fixed_reference)>>>2):32'sd0):((tracking_initialized?rp:(pre_capture?(observer_age>=10'd1000 ? automatic_brake_reference:xp):fixed_reference))-automatic_anchor)>>>2),.reference_velocity(tracking_initialized?(rv>>>2):32'sd0),.prediction_duty(prediction_use_interval?interval_command:command),.previous_duty(command),.limit_permille(swing_manual?{22'd0,budget_limit}:{22'd0,swing_limit}),
 .busy(observer_busy),.done(observer_done),.fault(observer_fault),.ready(observer_ready),.duty(observer_u),.debug_ext2(observer_ext2),.debug_state(observer_state),.debug_samples(observer_samples),.debug_integral(observer_integral),.debug_lp(observer_lp),.debug_age(observer_age));
 // R02: register protection decisions before the supervisor priority mux.
 // Adds one 50 MHz clock (20 ns) to automatic trips. STOP/disarm stays immediate;
 // the independent output guard still checks actual duty every clock.
 // R41: a recovery controller must demonstrably recover, not drive indefinitely.
 // Real run4 kept handoff selected for6s and escaped; other recorded recoveries
 // selected it for <=202ms. One second is a failure bound, not normal damping.
 // Only captured HOLD/TRACK is covered; SWING/BRAKE/COAST retain swing recovery.
 wire r41_monitor=(state==HOLD || state==TRACK) && tracking_initialized;
 reg[9:0]r41_rescue_ticks;
 reg[7:0]r41_escape_ticks;
 reg signed[32:0]r41_error_q;
 reg signed[31:0]r41_velocity_q;
 reg r41_sample_valid,r41_compare_pending,r41_escape_q;
 // Register subtraction before comparison. No new wide combinational path
 // into the control datapath, no multiplier, no RAM, no output schedule change.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   r41_error_q<=0;r41_velocity_q<=0;r41_sample_valid<=0;
   r41_compare_pending<=0;r41_escape_q<=0;
   r41_rescue_ticks<=0;r41_escape_ticks<=0;
  end else if(components_clear || start_accepted)begin
   r41_error_q<=0;r41_velocity_q<=0;r41_sample_valid<=0;
   r41_compare_pending<=0;r41_escape_q<=0;
   r41_rescue_ticks<=0;r41_escape_ticks<=0;
  end else begin
   r41_compare_pending<=step;
   if(step)begin
    r41_error_q<=$signed({phi[31],phi})-$signed({reference[31],reference});
    r41_velocity_q<=dphi;
    r41_sample_valid<=r41_monitor && measurement_ok && !sensor_gap_active;
    if(r41_monitor && measurement_ok && guard_active)begin
     if(r41_rescue_ticks<1000)r41_rescue_ticks<=r41_rescue_ticks+1'b1;
    end else r41_rescue_ticks<=0;
    if(r41_monitor && r41_escape_q)begin
     if(r41_escape_ticks<200)r41_escape_ticks<=r41_escape_ticks+1'b1;
    end else r41_escape_ticks<=0;
   end
   if(r41_compare_pending)r41_escape_q<=r41_sample_valid &&
    ((r41_error_q>33'sd71165 && r41_velocity_q>32'sd142331) ||
     (r41_error_q< -33'sd71165 && r41_velocity_q< -32'sd142331));
  end
 end
 wire r41_motion_failed=r41_monitor &&
  (r41_rescue_ticks>=1000 || r41_escape_ticks>=200);
 reg [7:0] trip_code;
 wire command_bad=pre_capture?(command>350 || command< -350):(command>350 || command< -350);
 wire start_decision=armed && !request_stop && trip_code==0 && state==IDLE &&
  ((request_capture && manual_ok)||(request_start && start_ok));
reg start_accepted,accepted_manual;reg signed[31:0] accepted_phi;
// Snapshot request and its SAME-cycle sensor admission separately from the
// state/permission mux. Final permission is checked again after one clock.
reg admission_pending,admission_manual,admission_start,manual_ok_q,start_ok_q;
reg signed[31:0] admission_phi;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin start_accepted<=0;accepted_manual<=0;accepted_phi<=0;
  admission_pending<=0;admission_manual<=0;admission_start<=0;manual_ok_q<=0;start_ok_q<=0;admission_phi<=0;end
 else begin
  admission_pending<=armed && !request_stop && trip_code==0 && state==IDLE && (request_capture || request_start);
  admission_manual<=request_capture;admission_start<=request_start;manual_ok_q<=manual_ok;start_ok_q<=start_ok;admission_phi<=phi<<<2;
  start_accepted<=admission_pending && ((admission_manual && manual_ok_q) || (admission_start && start_ok_q)) && armed && !request_stop && trip_code==0 && state==IDLE;
  accepted_manual<=admission_manual;accepted_phi<=admission_phi;
 end
end
// Readiness is diagnostic only. Register session cancellation to break the
// input-validity/velocity -> acceptance -> wide readiness-reset path.
reg readiness_start_q;
always @(posedge clk or negedge rst_n)
 if(!rst_n) readiness_start_q<=1'b0;
 else readiness_start_q<=start_accepted;

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) trip_code<=0;
  else if(!armed || request_stop) trip_code<=0;
  else if(active && (watchdog>=99999 || (step && phase!=0) || automatic_frame_overrun)) trip_code<=3;
  else if(active && command_bad) trip_code<=7;
  else if(active && r41_motion_failed) trip_code<=10;
  else if(active && step && physical_fault!=0) trip_code<=physical_fault;
  else if(coast_wait && step && coast_age>=599) trip_code<=2;
  else if(active && step && pre_capture && pre_capture_ticks>=12000)trip_code<=8'd5;
  else trip_code<=0;
 end
 // R10: policy consumes the current 1kHz estimate and previous committed command.
 // Its cap is snapshotted with the later sequential tracking transaction.
 wire [9:0] budget_limit;
 wire [7:0] budget_elapsed,settle_reason;
 wire [3:0] settle_blocks;
 reg signed [31:0] transaction_limit;
 wire signed [31:0] goal_q16=rp>>>2;
 wire signed [32:0] position_error_wide={phi[31],phi}-{goal_q16[31],goal_q16};
 wire signed [31:0] position_error=(position_error_wide>33'sd2147483647)?32'sh7fffffff:
  ((position_error_wide< -33'sd2147483648)?32'sh80000000:position_error_wide[31:0]);
 r01_recovery_budget recovery(.clk(clk),.rst(!rst_n),.start_session(start_accepted),.tick(step),
  .active(active && !pre_capture && !components_clear),.valid(measurement_ok),.fault(fault!=0 || trip_code!=0),
  .theta(filtered_theta_q18>>>2),.omega(dtheta),.arm_error(position_error),.duty(command[15:0]),
  .limit_permille(budget_limit),.used(recovery_used),.boosting(recovery_boosting),.elapsed(budget_elapsed));
 r01_motion_readiness readiness(.clk(clk),.reset(!rst_n),.tick(step),.start_check(readiness_start_q),
  .active(active && !pre_capture && !components_clear),.valid(measurement_ok),.reference_moving(trajectory_running || !ref_stationary),
  .fault(fault),.theta(filtered_theta_q18>>>2),.phi_error(position_error),.omega(dtheta),.arm_velocity(dphi),
  .settled(settled),.timed_out(settle_timeout),.reason(settle_reason),.good_blocks(settle_blocks));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin reported_limit<=150;transaction_limit<=150;end
  else if(components_clear || start_accepted || trip_code!=0) begin reported_limit<=(start_accepted && !accepted_manual)?350:150;transaction_limit<=150;end
  else begin
   if(!swing_manual && frame_tick && pending_command_valid && phase==0)reported_limit<={22'd0,pending_command_limit};
   if(swing_manual && phase==2 && swing_done && !swing_fault)reported_limit<={22'd0,swing_limit};
   if(track_tick)transaction_limit<=swing_manual?{22'd0,budget_limit}:{22'd0,swing_limit};
   if(swing_manual && phase==4 && observer_done && !observer_fault && !track_fault)reported_limit<=transaction_limit;
  end
 end
 // Sensor snapshots do not traverse the fault/timeout/command priority chain.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin xt<=0;xw<=0;xp<=0;xv<=0;xf<=0;swing_measurement_current<=0;end
  else if(step || state==IDLE) begin
   xt<=theta<<<2;xw<=dtheta<<<2;xp<=phi<<<2;xv<=dphi<<<2;xf<=filtered_theta_q18;
   // Progress decisions use real samples from this same 1 kHz snapshot.
   // A bounded gap prediction remains usable by the existing control, but
   // cannot certify a new turning point or teach the swing-progress logic.
   swing_measurement_current<=measurement_ok && !sensor_gap_active;
  end
 end
 // One-slot, session-cleared automatic command queue. The previous_duty input
 // remains the command actually committed at frame_tick, never pending_command.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pending_command<=0;pending_command_limit<=350;pending_command_valid<=0;end
  else if(components_clear || trip_code!=0 || state==IDLE || start_accepted || automatic_frame_overrun || coast_request || coast_wait)begin
   pending_command<=0;pending_command_limit<=350;pending_command_valid<=0;
  end else begin
   if(frame_tick && !swing_manual)pending_command_valid<=0;
   if(!swing_manual && phase==2 && swing_done && !swing_fault && swing_mode<=2 &&
      !auto_observer_warm && swing_range_ok(swing_u,swing_limit))begin
    pending_command<=swing_u;pending_command_limit<=swing_limit;pending_command_valid<=1;
   end
   if(!swing_manual && phase==4 && observer_result_done && !observer_result_fault && !track_fault && !signed_gt32(selected_u,transaction_limit) && !signed_gt32(-transaction_limit,selected_u))begin
    pending_command<=selected_u;pending_command_limit<=transaction_limit[9:0];pending_command_valid<=1;
   end
  end
 end
 // Stop/fault/disarm bypass the frame queue and clear output immediately.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) command<=0;
  else if(request_stop || !armed || trip_code!=0 || state==IDLE || state==FAULT || automatic_frame_overrun || coast_request || coast_wait) command<=0;
  else if(!swing_manual && frame_tick)begin
   if(pending_command_valid && phase==0)command<=pending_command;else command<=0;
  end
  else if(step) begin if(!active) command<=0;end
  else if(phase==2 && (swing_done || swing_fault)) begin
   if(swing_fault || !swing_range_ok(swing_u,swing_limit) || swing_mode>2) command<=0;
   else if(swing_manual)command<=swing_u;
  end else if(phase==3 && ref_done && ref_rejected) command<=0;
  else if(phase==4 && (observer_result_done || observer_result_fault || track_fault)) begin
   if(track_fault || observer_result_fault || signed_gt32(selected_u,transaction_limit) || signed_gt32(-transaction_limit,selected_u)) command<=0;
   else if(swing_manual)command<=selected_u;
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;fault<=0;phase<=0;result_valid<=0;watchdog<=0;session_ticks<=0;coast_age<=0;coast_good<=0;resume_elapsed<=0;
   pre_capture_ticks<=0;hold_ticks<=0;tracking_initialized<=0;pending_target<=0;pending_shuttle<=0;saved_target<=0;saved_shuttle<=0;automatic_goal<=0;automatic_home_valid<=0;automatic_home<=0;
   swing_session<=0;swing_tick<=0;swing_manual<=0;track_tick<=0;track_init<=0;
   ref_tick<=0;ref_reseed<=0;ref_target_valid<=0;ref_shuttle_valid<=0;
  end else begin
   result_valid<=0;swing_session<=0;swing_tick<=0;track_tick<=0;ref_tick<=0;
   if(!active || step) watchdog<=0;else if(watchdog<100000) watchdog<=watchdog+1'b1;
   if(explicit_target) begin pending_target<=1;saved_target<=target<<<2;if(!swing_manual)automatic_goal<=1;end
   if(explicit_shuttle) begin
    pending_shuttle<=1;saved_shuttle<=shuttle;if(!swing_manual)automatic_goal<=shuttle?2:0;
    // A user shuttle command replaces a still-queued internal home/center move.
    // Otherwise the stale target could be sent after the shuttle starts.
    if(internal_return || r37_deferred_target)pending_target<=0;
   end
   if(request_clear && !armed) begin
    automatic_goal<=0;automatic_home_valid<=0;automatic_home<=0;coast_age<=0;coast_good<=0;resume_elapsed<=0;
    state<=IDLE;fault<=0;phase<=0;tracking_initialized<=0;pending_target<=0;pending_shuttle<=0;
   end else if(request_stop || !armed) begin
    automatic_goal<=0;automatic_home_valid<=0;automatic_home<=0;coast_age<=0;coast_good<=0;resume_elapsed<=0;
    result_valid<=step;
    if(state!=FAULT) state<=IDLE;
    phase<=0;tracking_initialized<=0;pending_target<=0;pending_shuttle<=0;
   end else if(trip_code!=0) begin
    automatic_goal<=0;automatic_home_valid<=0;automatic_home<=0;coast_age<=0;coast_good<=0;resume_elapsed<=0;
    state<=FAULT;fault<=trip_code;phase<=0;result_valid<=1;
   end else if(state==IDLE && start_accepted) begin
    state<=accepted_manual?CAPTURE:SWING;swing_manual<=accepted_manual;swing_session<=1;coast_age<=0;coast_good<=0;resume_elapsed<=0;
    fault<=0;phase<=0;session_ticks<=0;pre_capture_ticks<=0;hold_ticks<=0;tracking_initialized<=0;
    pending_target<=0;pending_shuttle<=0;saved_shuttle<=0;automatic_goal<=0;automatic_home_valid<=0;automatic_home<=0;
   end else if(step) begin
    if(active) begin
     // A recovered shuttle first returns to zero through the normal smooth
     // reference. Only after that move ends and the measurements are quiet is
     // the retained shuttle request issued. The generator only admits shuttle
     // near zero; directly replaying it at a remote catch caused fault9.
     if(!swing_manual && !ext2_enable && automatic_goal==2 && saved_shuttle && tracking_initialized &&
        ref_stationary && !pending_target && !pending_shuttle && automatic_quiet &&
        !explicit_target && !explicit_shuttle)pending_shuttle<=1;
     if(session_ticks<17'h1ffff)session_ticks<=session_ticks+1'b1;
     if(pre_capture) pre_capture_ticks<=pre_capture_ticks+1'b1;
     if(state==CAPTURE && hold_ticks<2000) hold_ticks<=hold_ticks+1'b1;
     if(coast_request)begin
      state<=COAST;coast_age<=0;coast_good<=0;resume_elapsed<=0;
      phase<=0;result_valid<=1;tracking_initialized<=0;hold_ticks<=0;
      pending_target<=(recovery_goal!=0);pending_shuttle<=0;
      if(recovery_goal==2)saved_target<=0;
     end else if(coast_wait)begin
      phase<=0;result_valid<=1;coast_age<=coast_age+1'b1;
      if(coast_good_now)begin
       if(coast_good<32)coast_good<=coast_good+1'b1;
       if(coast_good>=31 && coast_age<599)begin
        state<=SWING;resume_elapsed<=coast_age+1'b1;coast_good<=0;
       end
      end else coast_good<=0;
     end else phase<=1;
    end else begin result_valid<=1;end
   end else case(phase)
    1:begin
     // R10: manual capture goes directly to the selected feedback core.
     // R26: BRAKE can use the observer before capture confirmation;
     // after 200 in-domain samples, blend with the same 200ms ramp.
     // The manual flag is latched at session start, never inferred from angle.
     if(!swing_manual && (pre_capture || (hold_ticks<2000 && !recovery_return_ready))) begin swing_tick<=1;phase<=2;end
     else begin
      ref_reseed<=!tracking_initialized;ref_target_valid<=pending_target && (swing_manual || tracking_initialized) && pending_target_allowed && !(r37_deferred_target && (explicit_target || explicit_shuttle));ref_shuttle_valid<=pending_shuttle && (swing_manual || tracking_initialized);
      if(swing_manual || tracking_initialized)begin if(!request_target && pending_target_allowed)pending_target<=0;if(!request_shuttle)pending_shuttle<=0;end ref_tick<=1;phase<=3;
     end
    end
    2:if(swing_done || swing_fault) begin
     resume_elapsed<=0;
     if(swing_fault || !swing_range_ok(swing_u,swing_limit)) begin state<=FAULT;fault<=swing_fault?swing_fault_code:7;end
     else begin
      
      if(swing_mode==2)begin
       if(pre_capture)begin
        state<=CAPTURE;hold_ticks<=0;pre_capture_ticks<=0;tracking_initialized<=0;
        // First catch establishes session provenance. A later local catch
        // does not request motion back to an old center by itself.
        if(!automatic_home_valid)begin
         automatic_home_valid<=1;automatic_home<=bounded_catch_home;
         if(automatic_goal==0)begin automatic_goal<=3;saved_target<=bounded_catch_home;
          pending_target<=catch_above || catch_below;end
        end
        else if(automatic_goal==2'd3)begin
         // No explicit target: hold around the new bounded catch center.
         // Preserve goal1 targets and goal2 retained shuttle requests.
         saved_target<=bounded_catch_home;
         pending_target<=catch_above || catch_below;
        end
       end
       else state<=hold_ticks<2000?CAPTURE:(tracking_initialized && !ref_stationary?TRACK:HOLD);
      end
      else if(swing_mode==1)begin state<=BRAKE;tracking_initialized<=0;hold_ticks<=0;pending_target<=(recovery_goal!=0);pending_shuttle<=0;
      if(recovery_goal==2)saved_target<=0;end
      else if(swing_mode==0)begin state<=SWING;tracking_initialized<=0;hold_ticks<=0;pending_target<=(recovery_goal!=0);pending_shuttle<=0;
      if(recovery_goal==2)saved_target<=0;end
      else begin state<=FAULT;fault<=7;end
     end
     if(!swing_manual && !swing_fault && auto_observer_warm)begin track_tick<=1;phase<=4;end
     else begin result_valid<=1;phase<=0;end
    end
    3:if(ref_done) begin
     ref_reseed<=0;ref_target_valid<=0;ref_shuttle_valid<=0;
     if(ref_rejected) begin state<=FAULT;fault<=9;result_valid<=1;phase<=0;end
     else if(!swing_manual)begin tracking_initialized<=1;swing_tick<=1;phase<=2;end
     else begin track_init<=!tracking_initialized;track_tick<=1;phase<=4;end
    end
    4:if(observer_result_done || observer_result_fault || track_fault) begin
     track_init<=0;
     if(track_fault || observer_result_fault || signed_gt32(selected_u,transaction_limit) || signed_gt32(-transaction_limit,selected_u)) begin state<=FAULT;fault<=7;end
     else if(swing_manual)begin tracking_initialized<=1;
      // Preserve the two-second capture qualification before position commands.
      state<=(hold_ticks<2000)?CAPTURE:(ref_stationary?HOLD:TRACK);end
     // Yield the default capture-home position once per rescue activation.
     // Keep observer origin/state and all angular/velocity feedback intact.
     // Publish fixed_reference now; the next phase1 truly reseeds r01_reference
     // before either controller is allowed to use tracking_initialized again.
     // Explicit target/shuttle requests are excluded above and retain priority.
     if(r37_motion_recenter)begin
      tracking_initialized<=0;pending_target<=1;pending_shuttle<=0;
      // An ordinary shuttle rescue first returns to its zero center after
      // guard release; the existing quiet/restart path restores saved_shuttle.
      // Explicit positioning retains the exact user target.
      if(automatic_goal==2'd2)saved_target<=0;
     end
     if(r35_local_recenter)begin
      tracking_initialized<=0;saved_target<=r35_bounded_local_home;
      automatic_home<=r35_bounded_local_home;automatic_home_valid<=1;
      automatic_goal<=2'd3;pending_target<=0;pending_shuttle<=0;
     end
     result_valid<=1;phase<=0;
    end
    default:begin end
   endcase
  end
 end
endmodule
`default_nettype wire
