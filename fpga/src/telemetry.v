`default_nettype none
module telemetry #(parameter PRECAPTURE=0)(input wire clk,rst_n,send,tx_ready,capture,
 input wire [511:0] snapshot,
 output wire tx_valid,output wire busy,output wire [7:0] tx_data,
 output reg [15:0] dropped);
 reg [511:0] frame;reg [1:0] state;reg [5:0] index;reg [15:0] crc;
 function [15:0] crc_byte;
 input [15:0] old;input [7:0] data;integer j;reg [15:0] c;
 begin c=old^{data,8'd0};for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1;crc_byte=c;end
 endfunction
 assign tx_valid=(state==2);
 assign busy=(state!=0);
 assign tx_data=frame[index*8 +:8];
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin frame<=0;state<=0;index<=0;crc<=16'hffff;dropped<=0;end
  else begin
   // The top previously stored exactly this snapshot in a separate512-bit
   // staging bank. Capture its same edge here; send still starts CRC one
   // clock later. Only idle frames may be preloaded; no serial overlap.
   if(PRECAPTURE && capture && state==0)frame<=snapshot;
   if(send && state!=0 && dropped!=65535) dropped<=dropped+1'b1;
   case(state)
    0:if(send) begin if(!PRECAPTURE)frame<=snapshot;state<=1;index<=2;crc<=16'hffff;end
    1:begin
       crc<=crc_byte(crc,frame[index*8 +:8]);
       if(index==61) begin frame[511:496]<=crc_byte(crc,frame[index*8 +:8]);index<=0;state<=2;end
       else index<=index+1'b1;
      end
    2:if(tx_ready) begin if(index==63) state<=3;else index<=index+1'b1;end
    3:if(tx_ready) state<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
