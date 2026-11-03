`include "j280_coordinates.vh"
`default_nettype none
// Physical target/speed conversion. Every output is registered; ordinary
// commands never pass a raw opcode mux to the next stage. CRC-valid STOP is
// handled independently in j280_top on its original acceptance edge.
module calibrated_command(input wire clk,rst_n,stop_level,in_valid,
 input wire[7:0] in_seq,in_opcode,input wire signed[31:0] in_value,
 output reg out_valid,output reg[7:0] out_seq,out_opcode,
 output reg signed[31:0] out_value,output reg overflow);
 reg busy,finish;reg[4:0] bit_index;reg[7:0] saved_seq,saved_opcode;
 reg signed[57:0] term,acc;
 wire signed[57:0] rounded=(acc+(acc<0?58'sd8388607:58'sd8388608))>>>24;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;finish<=0;bit_index<=0;term<=0;acc<=0;saved_seq<=0;saved_opcode<=0;out_valid<=0;out_seq<=0;out_opcode<=0;out_value<=0;overflow<=0;end
  else begin
   out_valid<=0;
   if(stop_level)begin busy<=0;finish<=0;overflow<=0;end
   else if(in_valid)begin
    busy<=0;finish<=0;overflow<=0;
    if(in_opcode==8'h26 || in_opcode==8'h37 || in_opcode==8'h39)begin
     busy<=1;bit_index<=0;term<={{26{in_value[31]}},in_value};acc<=0;saved_seq<=in_seq;saved_opcode<=in_opcode;
    end else if(in_opcode!=8'h01)begin
     out_valid<=1;out_seq<=in_seq;out_opcode<=in_opcode;out_value<=in_value;
    end
   end else if(finish)begin
    finish<=0;out_valid<=1;out_seq<=saved_seq;out_opcode<=saved_opcode;
    overflow<=rounded>58'sd2147483647 || rounded< -58'sd2147483648;
    out_value<=rounded[31:0];
   end else if(busy)begin
    if(((`J280_C_TARGET_K_Q24>>bit_index)&32'd1)!=0)acc<=acc+term;
    term<=term<<<1;
    if(bit_index==24)begin busy<=0;finish<=1;end else bit_index<=bit_index+1'b1;
   end
  end
 end
endmodule
`default_nettype wire
