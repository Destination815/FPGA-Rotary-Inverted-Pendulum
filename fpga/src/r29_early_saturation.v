`default_nettype none
// Count completed actuator intervals only at the raw 1 kHz boundary. The
// caller supplies the OLD command and its OLD reported cap from that edge;
// this module never publishes PWM, starts a controller, or adjusts a target.
// Pending survives accept and is retired only by its corresponding done.
module r29_early_saturation(
 input wire clk,rst_n,clear,stop,frame_tick,mature,
 input wire signed [31:0] previous_command,reported_limit,
 input wire guard_start,guard_done,discard_pending,
 output wire early_trigger,
 output reg [6:0] saturated_count,
 output reg fired,pending,inflight,prior_mature
);
wire signed [32:0] command_wide={previous_command[31],previous_command};
wire signed [32:0] cap_wide={reported_limit[31],reported_limit};
wire saturated=prior_mature&&(reported_limit>0)&&
               ((command_wide>=cap_wide)||(command_wide<=-cap_wide));
assign early_trigger=pending&&!inflight&&mature&&!clear&&!stop&&!discard_pending;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  saturated_count<=0;fired<=0;pending<=0;inflight<=0;prior_mature<=0;
 end else if(clear||stop||(frame_tick&&!mature))begin
  saturated_count<=0;fired<=0;pending<=0;inflight<=0;prior_mature<=0;
 end else begin
  // Accept cannot retire pending: the matching completion does that. The
  // in-flight bit suppresses duplicate force on consecutive start pulses.
  if(guard_done&&inflight)begin pending<=0;inflight<=0;end
  else if(guard_start&&early_trigger)inflight<=1;
  if(frame_tick)begin
   prior_mature<=mature;
   if(!fired)begin
    if(saturated)begin
     if(saturated_count==7'd99)begin
      saturated_count<=7'd100;fired<=1;pending<=1;
     end else saturated_count<=saturated_count+1'b1;
    end else saturated_count<=0;
   end
  end
  // A completed policy frame outside the original observer domain consumes
  // this one attempt. Do not postpone it until a later in-domain frame.
  // Keep fired: only leaving mature HOLD can arm a fresh 100ms attempt.
  if(discard_pending)begin pending<=0;inflight<=0;end
 end
end
endmodule
`default_nettype wire
