`default_nettype none
module command_rx #(parameter integer TIMEOUT=100000)(input wire clk,rst_n,byte_valid,byte_error,
 input wire [7:0] byte_data, output reg valid, output reg [7:0] sequence_id,opcode,
 output reg signed [31:0] value,output reg [15:0] errors);
 reg [3:0] index;reg [31:0] timer;reg [15:0] crc;reg [7:0] crc_low,seq_buf,op_buf;reg [31:0] val_buf;
 function [15:0] crc_byte;
 input [15:0] old;input [7:0] data;integer j;reg [15:0] c;
 begin c=old^{data,8'd0};for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin index<=0;timer<=0;crc<=16'hffff;crc_low<=0;seq_buf<=0;op_buf<=0;val_buf<=0;valid<=0;sequence_id<=0;opcode<=0;value<=0;errors<=0;end
  else begin
   valid<=0;
   if(index!=0) timer<=timer+1'b1;else timer<=0;
   if(byte_error || (index!=0 && timer>=TIMEOUT)) begin index<=0;timer<=0;if(errors!=65535) errors<=errors+1'b1;end
   else if(byte_valid) begin
    timer<=0;
    case(index)
     0:if(byte_data==8'haa) index<=1;
     1:if(byte_data==8'h55) begin index<=2;crc<=16'hffff;end else if(byte_data!=8'haa) index<=0;
     2:begin seq_buf<=byte_data;crc<=crc_byte(crc,byte_data);index<=3;end
     3:begin op_buf<=byte_data;crc<=crc_byte(crc,byte_data);index<=4;end
     4,5,6,7:begin val_buf[(index-4)*8 +:8]<=byte_data;crc<=crc_byte(crc,byte_data);index<=index+1'b1;end
     8:begin crc_low<=byte_data;index<=9;end
     9:begin index<=0;
       if({byte_data,crc_low}==crc) begin sequence_id<=seq_buf;opcode<=op_buf;value<=val_buf;valid<=1;end
       else if(errors!=65535) errors<=errors+1'b1;
      end
     default:index<=0;
    endcase
   end
  end
 end
endmodule
`default_nettype wire
