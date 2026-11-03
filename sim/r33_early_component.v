`default_nettype none
module component_top(
 input wire clk,rst_n,clear,stop,frame_tick,
 input wire [2:0] controller_state,
 input wire [11:0] hold_ticks,
 input wire [1:0] swing_mode,
 input wire manual,ref_stationary,ref_busy,new_request,trajectory_running,observing,valid,start,use_returning_gate,
 input wire publish,discard_pending,
 input wire signed [31:0] publish_command,publish_cap,
 input wire signed [31:0] theta,omega,velocity,
 input wire signed [15:0] observer_duty,fallback_duty,
 output wire mature,early_trigger,budget_trigger_accept,returning_allowed,force_trigger,
 output wire [6:0] saturated_count,
 output wire fired,pending,inflight,prior_mature,
 output wire signed [31:0] actual_command,actual_cap,
 output wire signed [15:0] duty,
 output wire done,reblend,guard_active,
 output wire [4:0] opposed_count,compatible_count,
 output wire [6:0] release_count,
 output wire [8:0] rescue_age,
 output wire signed [15:0] legacy_duty,
 output wire legacy_done,legacy_reblend,legacy_active
);
reg signed [31:0] committed_command,committed_cap;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin committed_command<=0;committed_cap<=150;end
 else if(publish)begin committed_command<=publish_command;committed_cap<=publish_cap;end
end
assign actual_command=committed_command;
assign actual_cap=committed_cap;
assign mature=((controller_state==3'd3&&hold_ticks>=12'd2000)||
               (controller_state==3'd2&&hold_ticks==12'd1999))&&swing_mode==2'd2&&
              !manual&&ref_stationary&&!trajectory_running;
wire hard_cancel=clear||manual||(controller_state!=3'd3&&controller_state!=3'd2)||
                 swing_mode!=2'd2||trajectory_running||(!ref_stationary&&!ref_busy)||new_request;
assign budget_trigger_accept=start&&early_trigger;
`include "r29_returning_gate.vh"
assign returning_allowed=r29_returning_gate(theta,omega);
assign force_trigger=early_trigger&&(!use_returning_gate||returning_allowed);
r29_early_saturation counter(.clk(clk),.rst_n(rst_n),.clear(hard_cancel),.stop(stop),
 .frame_tick(frame_tick),.mature(mature),.previous_command(committed_command),
 .reported_limit(committed_cap),.guard_start(start),.guard_done(done),.discard_pending(discard_pending),
 .early_trigger(early_trigger),.saturated_count(saturated_count),.fired(fired),
 .pending(pending),.inflight(inflight),.prior_mature(prior_mature));
wire active=observing&&swing_mode==2'd2&&!manual;
r29_handoff_guard guard(.clk(clk),.rst_n(rst_n),.clear(clear),.stop(stop),
 .start(start),.active(active),.valid(valid),.early_trigger(force_trigger),
 .theta(theta),.omega(omega),.velocity(velocity),.observer_duty(observer_duty),
 .fallback_duty(fallback_duty),.duty(duty),.done(done),.reblend(reblend),
 .guard_active(guard_active),.opposed_count(opposed_count),.release_count(release_count));
assign compatible_count=guard.compatible_count;
assign rescue_age=guard.rescue_age;
r29_handoff_guard_legacy legacy(.clk(clk),.rst_n(rst_n),.clear(clear),.stop(stop),
 .start(start),.active(active),.valid(valid),
 .theta(theta),.omega(omega),.velocity(velocity),.observer_duty(observer_duty),
 .fallback_duty(fallback_duty),.duty(legacy_duty),.done(legacy_done),.reblend(legacy_reblend),
 .guard_active(legacy_active),.opposed_count(),.release_count());
endmodule
`default_nettype wire
