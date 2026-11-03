// R20: repeatable authority, exact sampled policy with registered arithmetic.
module r01_recovery_budget(input wire clk,rst,start_session,tick,active,valid,fault,
 input wire signed[31:0] theta,omega,arm_error,input wire signed[15:0] duty,
 output reg[9:0] limit_permille,output reg used,boosting,output reg[7:0] elapsed);
`include "r10_observer_parameters.vh"
reg[7:0] warm;reg[8:0] stable;reg[2:0] phase;
reg signed[31:0] theta_s,omega_s;reg signed[63:0] lead;
reg signed[32:0] projected;reg outward_s,recovered_s,risk_s;
wire unused_inputs=^arm_error ^ ^duty;
// Scratch values are only consumed by a non-cancelled transaction.
always @(posedge clk)begin
 if(tick)begin theta_s<=theta;omega_s<=omega;end
 if(phase==1)lead<=$signed(omega_s)*RECOVERY_LEAD;
 if(phase==2)begin
  projected<=$signed({theta_s[31],theta_s})+$signed(lead[48:16]);
  outward_s<=(theta_s[31]==omega_s[31])&&(omega_s>9830||omega_s< -9830);
  recovered_s<=(theta_s<2621&&theta_s> -2621&&omega_s<19661&&omega_s> -19661);
 end
 if(phase==3)risk_s<=outward_s&&(projected>$signed(RECOVERY_TRIGGER[32:0])||projected< -$signed(RECOVERY_TRIGGER[32:0]));
end
always @(posedge clk or posedge rst)begin
 if(rst)begin phase<=0;warm<=0;stable<=0;limit_permille<=150;used<=0;boosting<=0;elapsed<=0;end
 else if(start_session)begin phase<=0;warm<=0;stable<=0;limit_permille<=150;used<=0;boosting<=0;elapsed<=0;end
 else if(tick)begin
  phase<=0;
  if(!active||!valid||fault)begin limit_permille<=150;boosting<=0;stable<=0;elapsed<=0;end
  else if(warm<200)begin warm<=warm+1;limit_permille<=150;end
  else phase<=1;
 end else if(phase!=0)begin
  if(!active||!valid||fault)begin phase<=0;limit_permille<=150;boosting<=0;stable<=0;elapsed<=0;end
  else if(phase<4)phase<=phase+1'b1;
  else begin
   phase<=0;
   if(boosting)begin
    if(recovered_s)begin if(stable<50)stable<=stable+1;if(stable>=49)begin boosting<=0;stable<=0;end end else stable<=0;
    if(elapsed<255)elapsed<=elapsed+1;
   end else if(risk_s)begin boosting<=1;used<=1;elapsed<=0;stable<=0;end
   if(boosting || (!boosting&&risk_s))begin if(limit_permille<340)limit_permille<=limit_permille+10;else limit_permille<=350;end
   else if(limit_permille>152)limit_permille<=limit_permille-2;else limit_permille<=150;
  end
 end
end
endmodule
