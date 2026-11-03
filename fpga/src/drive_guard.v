`default_nettype none
// R01 independent mode-specific invariant; STA required for this implementation.
module drive_guard(input wire clk,rst_n,enable,clear,
 input wire signed [31:0] requested,limit,
 output wire allowed,output wire signed [31:0] safe_command,output reg tripped);
 wire limit_valid=(limit[31:9]==23'd0) &&
                  (limit[8:0]>=9'd150) && (limit[8:0]<=9'd350);
 // For any accepted cap <=350, an in-range command must fit signed10.
 // Check the discarded bits before narrowing so arbitrary signed32 inputs
 // keep the exact old rejection behavior; no added register or clock delay.
 wire requested_fits_10=(requested[31:10]=={22{requested[9]}});
 wire signed [9:0] requested_10=requested[9:0];
 wire signed [9:0] cap_10={1'b0,limit[8:0]};
 wire in_range=limit_valid && requested_fits_10 &&
               requested_10<=cap_10 && requested_10>=-cap_10;
 assign allowed=enable && !tripped && in_range;
 assign safe_command=allowed?requested:32'sd0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) tripped<=0;
  else if(clear && !enable) tripped<=0;
  else if(enable && !in_range) tripped<=1;
 end
endmodule
`default_nettype wire
