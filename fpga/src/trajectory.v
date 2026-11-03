`include "j280_config.vh"
`default_nettype none
module trajectory(input wire clk,rst_n,step,enable,reseed,pause,
 input wire signed [31:0] position,target,
 input wire shuttle,
 output reg signed [31:0] reference,velocity);
 localparam signed [31:0] VMAX=`J280_REF_VMAX_Q16;
 localparam signed [31:0] ACCEL=`J280_REF_ACCEL_Q16;
 localparam signed [31:0] DV=ACCEL/1000;
 localparam signed [31:0] AMP=`J280_REF_AMPLITUDE_Q16;
 reg signed [31:0] remainder,next_rem,move_sum;
 reg positive;reg [9:0] dwell;
 reg signed [31:0] goal,error,desired,next_v,next_p,abs_error,abs_v,stop_distance;
 reg signed [63:0] speed_square,stop_wide;
 always @* begin
  goal=shuttle?(positive?AMP:-AMP):target;
  error=goal-reference;abs_error=error<0?-error:error;
  abs_v=velocity<0?-velocity:velocity;
  speed_square=$signed(velocity)*$signed(velocity);
  stop_wide=speed_square/(2*ACCEL);stop_distance=stop_wide[31:0];
  desired=(error>=0)?VMAX:-VMAX;
  if(pause || abs_error<=stop_distance+abs_v/1000+32'sd1) desired=0;
  next_v=velocity;
  if(velocity<desired) begin next_v=velocity+DV;if(next_v>desired) next_v=desired;end
  else if(velocity>desired) begin next_v=velocity-DV;if(next_v<desired) next_v=desired;end
  move_sum=next_v+remainder;
  next_p=reference+move_sum/1000;next_rem=move_sum%1000;
  if(!pause && abs_error<=1 && abs_v<DV*2) begin next_p=goal;next_v=0;next_rem=0;end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin reference<=0;velocity<=0;remainder<=0;positive<=1;dwell<=0;end
  else if(reseed || !enable) begin reference<=position;velocity<=0;remainder<=0;positive<=1;dwell<=0;end
  else if(step) begin
   reference<=next_p;velocity<=next_v;remainder<=next_rem;
   if(shuttle && abs_error<=1 && abs_v<DV*2 && !pause) begin
    if(dwell==499) begin positive<=!positive;dwell<=0;end else dwell<=dwell+1'b1;
   end else dwell<=0;
  end
 end
endmodule
`default_nettype wire
