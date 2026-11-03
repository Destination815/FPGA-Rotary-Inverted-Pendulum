`include "j280_coordinates.vh"
`default_nettype none
// Separate measurement export. No controller input or enable depends on this unit.
// Serial constant multiply: no new DSP or RAM. Completion < 25 clocks at 50MHz.
module calibrated_position(input wire clk,rst_n,tick,calibrate,input wire signed[31:0] encoder,
 output reg signed[31:0] encoder_zero,phi_physical,output reg saturated);
 reg[4:0] bit_index;reg busy,finish;reg signed[49:0] term,acc;
 wire signed[31:0] delta=encoder-encoder_zero;
 // Half-away-from-zero rounding: one constant add, no double negation path.
 wire signed[49:0] rounded=(acc+(acc<0?50'sd127:50'sd128))>>>8;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin encoder_zero<=0;phi_physical<=0;saturated<=0;busy<=0;finish<=0;bit_index<=0;term<=0;acc<=0;end
  else if(calibrate)begin encoder_zero<=encoder;phi_physical<=0;saturated<=0;busy<=0;finish<=0;end
  else if(finish)begin
   finish<=0;
   if(rounded>50'sd2147483647)begin phi_physical<=32'sh7fffffff;saturated<=1;end
   else if(rounded< -50'sd2147483648)begin phi_physical<=32'sh80000000;saturated<=1;end
   else begin phi_physical<=rounded[31:0];saturated<=0;end
  end else if(busy)begin
   if(((`J280_C_PHYSICAL_RPC_Q24>>bit_index)&32'd1)!=0)acc<=acc+term;
   term<=term<<<1;
   if(bit_index==16)begin busy<=0;finish<=1;end else bit_index<=bit_index+1'b1;
  end else if(tick)begin term<={{18{delta[31]}},delta};acc<=0;bit_index<=0;busy<=1;end
 end
endmodule
`default_nettype wire
