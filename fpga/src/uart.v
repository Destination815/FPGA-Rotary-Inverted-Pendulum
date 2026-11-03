`default_nettype none
module uart_tx #(parameter integer DIV=434)(input wire clk,rst_n,valid,
 input wire [7:0] data, output wire ready,output wire tx);
 localparam [15:0] RELOAD=DIV[15:0]-16'd1;
 reg [9:0] shift;reg [15:0] timer;reg [3:0] bits;
 assign ready=(bits==0);assign tx=(bits==0)?1'b1:shift[0];
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin shift<=10'h3ff;timer<=0;bits<=0;end
  else if(bits==0) begin if(valid) begin shift<={1'b1,data,1'b0};bits<=10;timer<=RELOAD;end end
  else if(timer==0) begin shift<={1'b1,shift[9:1]};bits<=bits-1'b1;timer<=RELOAD;end
  else timer<=timer-1'b1;
 end
endmodule
module uart_rx #(parameter integer DIV=434)(input wire clk,rst_n,rx,
 output reg valid,error,output reg [7:0] data);
 localparam [15:0] RELOAD=DIV[15:0]-16'd1, HALF=DIV[16:1]-16'd1;
 reg [1:0] sync_rx;reg [1:0] state;reg [15:0] timer;reg [2:0] bit_index;reg [7:0] shift;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync_rx<=3;state<=0;timer<=0;bit_index<=0;shift<=0;data<=0;valid<=0;error<=0;end
  else begin
   sync_rx<={sync_rx[0],rx};valid<=0;error<=0;
   case(state)
    0:if(!sync_rx[1]) begin state<=1;timer<=HALF;end
    1:if(timer!=0) timer<=timer-1'b1;
      else if(sync_rx[1]) state<=0;else begin state<=2;timer<=RELOAD;bit_index<=0;end
    2:if(timer!=0) timer<=timer-1'b1;else begin
       shift[bit_index]<=sync_rx[1];timer<=RELOAD;
       if(bit_index==7) state<=3;else bit_index<=bit_index+1'b1;
      end
    3:if(timer!=0) timer<=timer-1'b1;else begin
       if(sync_rx[1]) begin data<=shift;valid<=1;end else error<=1;
       state<=0;
      end
   endcase
  end
 end
endmodule
`default_nettype wire
