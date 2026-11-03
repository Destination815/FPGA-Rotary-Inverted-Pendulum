`default_nettype none
module r01_control_harness(input wire clk,rst_n,step,armed,measurement_ok,
 input wire raw_mode,fractional_mode,fractional_valid,adc_ready,calibrate,input wire[13:0] adc_q4,zero_q4,input wire [9:0] adc,zero,input wire signed [31:0] encoder,
 input wire begin_swing,begin_capture,stop,clear,target_valid,shuttle_valid,shuttle,full_turn_recovery,
 input wire signed[31:0] workspace_limit,shuttle_amplitude,input wire[1:0] motion_speed,input wire[7:0] shuttle_period,
 input wire signed [31:0] theta,omega,phi,velocity,filtered,target,
 output wire signed [31:0] debug_home,
 output wire signed [31:0] duty,reference,ref_velocity,active_limit,
 output wire [2:0] state,output wire [7:0] fault,
 output wire handoff_recovery,
 output wire recovery_boosting,recovery_used,settled,settle_timeout,
 output wire done,reference_ready,trajectory_running,measurement_valid,gap_active,
 output wire [7:0] gap_ms,output wire signed [31:0] measured_theta,measured_omega,measured_phi,measured_velocity,measured_filtered);
 wire est_step;reg previous_coast;reg estimator_restart_q;
 always @(posedge clk or negedge rst_n)if(!rst_n)previous_coast<=0;else previous_coast<=state==7;
 always @(posedge clk or negedge rst_n)if(!rst_n)estimator_restart_q<=0;else estimator_restart_q<=state==7 && !previous_coast;
 state_estimator estimator(.clk(clk),.rst_n(rst_n),.tick(step),.adc_ready(adc_ready),.calibrate(calibrate),.restart_observation(estimator_restart_q),
 .adc(adc),.zero_raw(zero),.adc_q4(fractional_mode?adc_q4:{adc,4'd0}),.zero_q4(fractional_mode?zero_q4:{zero,4'd0}),.adc_q4_valid(!fractional_mode || fractional_valid),.theta_raw_q16(),.encoder(encoder),.gap_allow(state==1 || state==6),
 .step_valid(est_step),.measurement_ok(measurement_valid),.theta(measured_theta),.dtheta(measured_omega),
 .phi(measured_phi),.dphi(measured_velocity),.filtered_theta_q18(measured_filtered),.gap_active(gap_active),.gap_ms(gap_ms));
 wire priority_unused;
 controller control(.clk(clk),.rst_n(rst_n),.step(raw_mode?est_step:step),.frame_tick(step),.armed(armed),.measurement_ok(raw_mode?measurement_valid:measurement_ok),.sensor_gap_active(raw_mode?gap_active:1'b0),.sensor_gap_ms(raw_mode?gap_ms:8'd0),
 .request_start(begin_swing),.request_capture(begin_capture),.request_stop(stop),.request_clear(clear),.request_jog(1'b0),
 .workspace_limit(workspace_limit),.full_turn_recovery(full_turn_recovery),.shuttle_amplitude(shuttle_amplitude),.motion_speed(motion_speed),.shuttle_period(shuttle_period),.request_target(target_valid),.request_shuttle(shuttle_valid),.shuttle(shuttle),.target(target),
 .theta(raw_mode?measured_theta:theta),.dtheta(raw_mode?measured_omega:omega),.phi(raw_mode?measured_phi:phi),.dphi(raw_mode?measured_velocity:velocity),.filtered_theta_q18(raw_mode?measured_filtered:filtered),
 .jog_value(32'sd0),.ktheta(32'sd0),.kdtheta(32'sd0),.kphi(32'sd0),.kdphi(32'sd0),.pwm_limit(32'sd150),.swing_gain(32'sd0),
 .state(state),.fault(fault),.command(duty),.reference(reference),.ref_velocity(ref_velocity),
 .result_valid(done),.reference_ready(reference_ready),.trajectory_running(trajectory_running),.priority_active(priority_unused),.active_limit(active_limit),.recovery_boosting(recovery_boosting),.recovery_used(recovery_used),.settled(settled),.settle_timeout(settle_timeout),.handoff_recovery(handoff_recovery),.observer_state(),.observer_samples(),.observer_ready_out(),.observer_integral(),.observer_lp(),.observer_origin(),.observer_age());
 assign debug_home=control.automatic_home;
endmodule
`default_nettype wire
