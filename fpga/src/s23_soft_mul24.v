module s23_soft_mul24(input wire clk,rst_n,cancel,start,input wire[23:0]a,b,output reg[47:0]product,output reg done);
reg active;reg[2:0]count;reg[23:0]left,right;reg[47:0]acc;
wire[27:0]p0={4'd0,left & {24{right[0]}}};
wire[27:0]p1={3'd0,left & {24{right[1]}},1'b0};
wire[27:0]p2={2'd0,left & {24{right[2]}},2'b0};
wire[27:0]p3={1'd0,left & {24{right[3]}},3'b0};
wire[27:0]x=p0^p1^p2,y=((p0&p1)|(p0&p2)|(p1&p2))<<1;
wire[27:0]z=x^y^p3,c=((x&y)|(x&p3)|(y&p3))<<1;
wire[47:0]next=(acc>>4)+{(z+c),20'd0};
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin active<=0;done<=0;product<=0;count<=0;left<=0;right<=0;acc<=0;end
 else if(cancel)begin active<=0;done<=0;product<=0;end
 else begin done<=0;
 if(start&&!active)begin left<=a;right<=b;acc<=0;count<=0;active<=1;end
 else if(active)begin acc<=next;right<=right>>4;count<=count+1'b1;if(count==5)begin product<=next;done<=1;active<=0;end end
 end end
endmodule
