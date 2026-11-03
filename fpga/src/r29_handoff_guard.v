`default_nettype none
// Independent candidate; no controller/observer connection in this module.
// Q18 observations, signed permille duties. No new gain, cap, slew or multiply.
// Edge N accepts start and computes pending_duty. Edge N+1 commits duty/done
// and the matching reblend pulse. Back-to-back starts are supported.
// rst_n/clear/stop cancel pending work. Inactive starts still complete, bypassing
// observer_duty; inactive clocks clear the guard even without a transaction.
module r29_handoff_guard(
 input wire clk,rst_n,clear,stop,start,active,valid,early_trigger,fast_release_permission,
 input wire signed [31:0] theta,omega,velocity,
 input wire signed [15:0] observer_duty,fallback_duty,
 output reg signed [15:0] duty,
 output reg done,reblend,guard_active,
 output reg [4:0] opposed_count,
 output reg [6:0] release_count
);
reg pending,pending_reblend;
reg [4:0] compatible_count,fast_compatible_count;
reg [8:0] rescue_age;
reg signed [15:0] pending_duty;
// Signed comparisons avoid abs(INT_MIN) and replace both sign-only products.
wire angle_large=(theta>32'sd13726)||(theta< -32'sd13726);
wire moving_out=((theta>0)&&(omega>0))||((theta<0)&&(omega<0));
wire output_opposed=((theta>0)&&(observer_duty<0))||
                    ((theta<0)&&(observer_duty>0));
wire opposed=angle_large&&moving_out&&output_opposed;
wire quiet=(theta> -32'sd6863)&&(theta<32'sd6863)&&
           (omega> -32'sd52429)&&(omega<32'sd52429)&&
           (velocity> -32'sd78643)&&(velocity<32'sd78643);
// A mature, already blended observer may recover while the rescue loop is
// still moving. Return only in a bounded inward state and with compatible
// commands; the caller restarts its unchanged 200 ms blend after this pulse.
wire inward=((theta>=0)&&(omega<=0))||((theta<=0)&&(omega>=0));
wire same_direction=((observer_duty>0)&&(fallback_duty>0))||
                    ((observer_duty<0)&&(fallback_duty<0));
// R39T6: diff = 256*high + low. Each subtraction is only nine bits.
// Exact signed16 full-domain +/-20 and +/-125 tests, without serial wide compares.
wire signed[8:0] r39t6_diff_low=$signed({1'b0,observer_duty[7:0]})-$signed({1'b0,fallback_duty[7:0]});
wire signed[8:0] r39t6_diff_high=$signed({observer_duty[15],observer_duty[15:8]})-$signed({fallback_duty[15],fallback_duty[15:8]});
wire r39t6_near20=((r39t6_diff_high==0)&&(r39t6_diff_low>= -9'sd20)&&(r39t6_diff_low<=9'sd20)) ||
 ((r39t6_diff_high==9'sd1)&&(r39t6_diff_low<= -9'sd236)) ||
 ((r39t6_diff_high== -9'sd1)&&(r39t6_diff_low>=9'sd236));
wire r39t6_near125=((r39t6_diff_high==0)&&(r39t6_diff_low>= -9'sd125)&&(r39t6_diff_low<=9'sd125)) ||
 ((r39t6_diff_high==9'sd1)&&(r39t6_diff_low<= -9'sd131)) ||
 ((r39t6_diff_high== -9'sd1)&&(r39t6_diff_low>=9'sd131));
wire compatible=(rescue_age>=9'd500)&&(theta> -32'sd11438)&&(theta<32'sd11438)&&
                (omega> -32'sd262144)&&(omega<32'sd262144)&&
                (velocity> -32'sd393216)&&(velocity<32'sd393216)&&
                inward&&same_direction&&
                r39t6_near20;
wire fast_compatible=(fast_release_permission===1'b1)&&(rescue_age>=9'd100)&&
 (theta> -32'sd39322)&&(theta<32'sd39322)&&
 (omega> -32'sd655360)&&(omega<32'sd655360)&&
 (velocity> -32'sd786432)&&(velocity<32'sd786432)&&
 inward&&same_direction&&r39t6_near125;
// R41T1: parallel next-value masks for the rescue counter only.
// Early trigger remains current-cycle, and every original cancel wins.
wire r41t1_release=(quiet && release_count==7'd99) ||
 (compatible && compatible_count==5'd19) || (fast_compatible && fast_compatible_count==5'd19);
wire[8:0] r41t1_age_inc=rescue_age<9'd500 ? rescue_age+1'b1:rescue_age;
wire[8:0] r41t1_age_next=({9{early_trigger&&valid}}&9'd1) |
 ({9{!early_trigger&&valid&&guard_active&&!r41t1_release}}&r41t1_age_inc);
always @(posedge clk)begin
 if(!rst_n||clear||stop||!active)rescue_age<=0;
 else if(start)rescue_age<=r41t1_age_next;
end
always @(posedge clk)begin
 if(!rst_n||clear||stop)begin
  pending<=0;pending_reblend<=0;pending_duty<=0;
  duty<=0;done<=0;reblend<=0;guard_active<=0;
  opposed_count<=0;release_count<=0;compatible_count<=0;fast_compatible_count<=0;
 end else begin
  done<=pending;reblend<=pending&&pending_reblend;
  if(pending)duty<=pending_duty;
  pending<=start;pending_reblend<=0;
  if(!active)begin
   guard_active<=0;opposed_count<=0;release_count<=0;compatible_count<=0;fast_compatible_count<=0;
  end
  if(start)begin
   pending_duty<=observer_duty;
   if(!active)begin
    guard_active<=0;opposed_count<=0;release_count<=0;compatible_count<=0;fast_compatible_count<=0;
   end else if(early_trigger)begin
    // Match forcing latched=true and clearing all four counters immediately
    // before the native contract tick. This is already-latched processing,
    // not the ordinary opposition activation: first valid rescue_age is 1.
    // A prediction still selects fallback but establishes no release dwell.
    guard_active<=1;opposed_count<=0;compatible_count<=0;fast_compatible_count<=0;
    release_count<=valid&&quiet?7'd1:7'd0;
    
    pending_duty<=fallback_duty;
   end else if(!valid)begin
    // Predictions neither accumulate opposition nor establish quiet recovery.
    // A previously latched rescue remains selected until a real release.
    opposed_count<=0;release_count<=0;compatible_count<=0;fast_compatible_count<=0;
    if(guard_active)pending_duty<=fallback_duty;
   end else if(guard_active)begin
    pending_duty<=fallback_duty;

    // opposed_count retains the triggering 20 while latched; invalid, release
    // or inactive clears it. It is a trigger diagnostic, not a new learning term.
    if(quiet)release_count<=release_count+1'b1;
    else release_count<=0;
    if(compatible)compatible_count<=compatible_count+1'b1;
    else compatible_count<=0;fast_compatible_count<=0;
    if(fast_compatible)fast_compatible_count<=fast_compatible_count+1'b1;
    else fast_compatible_count<=0;
    if((quiet && release_count==7'd99) || (compatible && compatible_count==5'd19) ||
       (fast_compatible && fast_compatible_count==5'd19))begin
     guard_active<=0;opposed_count<=0;release_count<=0;compatible_count<=0;fast_compatible_count<=0;
     pending_reblend<=1;
    end
   end else begin
    release_count<=0;compatible_count<=0;fast_compatible_count<=0;
    if(opposed)begin
     if(opposed_count==5'd19)begin
      opposed_count<=5'd20;guard_active<=1;
      pending_duty<=fallback_duty;
     end else opposed_count<=opposed_count+1'b1;
    end else opposed_count<=0;
   end
  end
 end
end
endmodule
`default_nettype wire
