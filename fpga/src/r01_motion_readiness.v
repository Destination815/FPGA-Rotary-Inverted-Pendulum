// R10 r3: same per-sample status arithmetic, registered 23-clock evaluation.
// Four parallel shift/add squares avoid DSP use and multiplication+sum+decision chains.
// tick snapshots inputs; reset/start_check abort every in-flight stage. Status only.
module r01_motion_readiness(
 input wire clk,reset,tick,start_check,active,valid,reference_moving,
 input wire [7:0] fault,
 input wire signed [31:0] theta,phi_error,omega,arm_velocity,
 output reg settled,output reg timed_out,
 output reg [7:0] reason,output reg [3:0] good_blocks);
localparam [47:0] ANGLE_SUM=48'd294122500;
localparam [47:0] SPEED_SUM=48'd38651560000;
reg ever_settled;
reg [6:0] count;reg [13:0] elapsed;
reg [47:0] st,sp,sw,sv,nt,np,nw,nv;
reg [2:0] phase;
reg signed[31:0] xt,xp,xw,xv;
reg xa,xvalid,xmoving;reg[7:0] xfault;
reg hard_bad,block_ok;
reg [16:0] mt,mp,mw,mv;
reg [33:0] at,ap,aw,av,qt,qp,qw,qv;
reg [4:0] bit_count;
function [16:0] magnitude;
 input signed[31:0] a;
 begin
 if(a>65536 || a< -65536)magnitude=17'd65536;
 else if(a<0)magnitude=-a[16:0];
 else magnitude=a[16:0];
 end
endfunction
wire next_settled=xa&&xvalid&&(xfault==0)&&!xmoving&&!hard_bad&&((count==99)?(block_ok&&good_blocks>=4):settled);
// Snapshot data independently of start_check. The FSM discards the sample on
// reset/start; including their long supervisor predicate in data enables was
// an unnecessary estimator -> start acceptance -> snapshot timing path.
always @(posedge clk)begin
 if(tick && phase==0)begin
  xt<=theta;xp<=phi_error;xw<=omega;xv<=arm_velocity;
  xa<=active;xvalid<=valid;xmoving<=reference_moving;xfault<=fault;
 end
end
// Intermediate arithmetic is disposable on reset/start. Only the scheduler
// and committed status need the priority cancellation predicate. Phase 2
// initializes every square operand/accumulator before any valid commit.
always @(posedge clk)begin
 case(phase)
 1:begin
  mt<=magnitude(xt);mp<=magnitude(xp);mw<=magnitude(xw);mv<=magnitude(xv);
  hard_bad<=xt>5719||xt< -5719||xp>5719||xp< -5719||xw>65536||xw< -65536||xv>65536||xv< -65536;
 end
 2:begin
  at<={17'd0,mt};ap<={17'd0,mp};aw<={17'd0,mw};av<={17'd0,mv};
  qt<=0;qp<=0;qw<=0;qv<=0;bit_count<=0;
 end
 3:begin
  qt<=qt+(mt[0]?at:34'd0);qp<=qp+(mp[0]?ap:34'd0);
  qw<=qw+(mw[0]?aw:34'd0);qv<=qv+(mv[0]?av:34'd0);
  mt<=mt>>1;mp<=mp>>1;mw<=mw>>1;mv<=mv>>1;
  at<=at<<1;ap<=ap<<1;aw<=aw<<1;av<=av<<1;
  if(bit_count!=16)bit_count<=bit_count+1'b1;
 end
 4:begin nt<=st+{14'd0,qt};np<=sp+{14'd0,qp};nw<=sw+{14'd0,qw};nv<=sv+{14'd0,qv};end
 5:begin block_ok<=(nt<=ANGLE_SUM)&&(np<=ANGLE_SUM)&&(nw<=SPEED_SUM)&&(nv<=SPEED_SUM);end
 default:begin end
 endcase
end
always @(posedge clk)begin
 if(reset||start_check)begin
  phase<=0;ever_settled<=0;count<=0;elapsed<=0;st<=0;sp<=0;sw<=0;sv<=0;
  settled<=0;timed_out<=0;good_blocks<=0;reason<=8'd1;
 end else if(phase==0)begin
  if(tick)begin
   phase<=1;
  end
 end else case(phase)
 1:phase<=2;
 2:phase<=3;
 3:if(bit_count==16)phase<=4;
 4:phase<=5;
 5:phase<=6;
 6:begin
  phase<=0;
  if(!xa||!xvalid||xfault!=0||xmoving||hard_bad)begin
   count<=0;st<=0;sp<=0;sw<=0;sv<=0;settled<=0;good_blocks<=0;
   if(!xa)reason<=8'd2;else if(!xvalid)reason<=8'd3;else if(xfault!=0)reason<=8'd4;else if(xmoving)reason<=8'd5;else reason<=8'd6;
  end else if(count==99)begin
   count<=0;st<=0;sp<=0;sw<=0;sv<=0;
   if(block_ok)begin
    if(good_blocks<5)good_blocks<=good_blocks+1;
    if(good_blocks>=4)begin settled<=1;reason<=0;end else begin settled<=0;reason<=1;end
   end else begin good_blocks<=0;settled<=0;reason<=7;end
  end else begin count<=count+1;st<=nt;sp<=np;sw<=nw;sv<=nv;end
  if(next_settled)ever_settled<=1;
  if(xa&&!ever_settled)begin if(elapsed<8000)elapsed<=elapsed+1;if(elapsed>=7999&&!next_settled)timed_out<=1;end
 end
 default:phase<=0;
 endcase
end
endmodule
