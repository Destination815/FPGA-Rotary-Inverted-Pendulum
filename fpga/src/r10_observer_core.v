// Generated Q24 normalized seven-state observer + LQR. One shared registered multiplier.
// Every transaction snapshots measurements, previous applied logical command and cap.
module r10_observer_core(input wire clk,rst_n,clear,start,automatic_policy,automatic_hold,automatic_brake,motion_assist_permission,
 input wire handoff_reblend,
 input wire ext2_warm,ext2_enable,ext2_motion,input wire signed[31:0] ext2_reference,
 input wire mature_static_hold,rate_hold,
 input wire measurement_current,input wire signed[31:0] measured_omega,
 input wire signed [31:0] fallback_duty,theta,phi,reference_offset,reference_velocity,previous_duty,prediction_duty,limit_permille,
 output reg busy,done,fault,ready,output reg signed [31:0] duty,
 output wire[511:0] debug_ext2,
 output wire[223:0] debug_state,output wire[7:0] debug_samples,output wire signed[31:0] debug_integral,debug_lp,output wire[9:0] debug_age);

reg [8:0] state;
reg signed[31:0] theta_s;

// R39: only the explicitly admitted manual ext2 transaction uses A391.
// Shared Q24 scratch, seven observer state registers and multiplier retain
// their existing 242 identities; the extra bank is the A manual 8-state matrix.
reg[7:0] r40_transfer_ticks;
reg r39_mode;reg r39_matrix_added;reg[8:0]r39_matrix_return;reg motion_s;reg signed[31:0] z7,n7;reg bias_clipped;
wire signed[31:0] velocity_clip=r39t5_data_cmp0 ? 32'sd4194304 : r39t5_data_cmp1 ? -32'sd4194304 : velocity_s;
localparam integer MANUAL_SMOOTH_CONTINUITY=1,MANUAL_SMOOTH_VELOCITY=1,MANUAL_SINE_DAMPING=1,MANUAL_SINE_FEEDFORWARD=1,MANUAL_DAMPED_FEEDBACK=1,MANUAL_OUTPUT_SHAPE=1,MANUAL_FILTERED_MOTION=1,MANUAL_ENCODER_FEEDBACK=1,MANUAL_LOW_GAIN=1,MOTION_INTEGRAL=1,MOTION_I_HOLD=1,MOTION_VELOCITY_TRIM=1,MOTION_POSITION_BOOST=1,DISTURBANCE_OBSERVER=1,DISTURBANCE_REGULATOR=1;
function signed[31:0] position_boost_operand;
 input signed[31:0] e;reg signed[31:0] clipped;reg signed[63:0] wide;
 begin
 clipped=e>32'sd5856353 ? 32'sd5856353 : e< -32'sd5856353 ? -32'sd5856353 : e;
 wide=$signed({{32{e[31]}},e})+($signed({{32{clipped[31]}},clipped})>>>3);
 position_boost_operand=sat(wide);
 end
endfunction

// R27 bounded encoder residual correction of the manual velocity feedback only.
// Encoder/observer positions are Q24 radians; n3 is Q24 normalized by10.
// Gain1 therefore adds10/s * position residual, capped at0.2rad/s.
// Three transaction scratch registers pipeline the arithmetic; no new filter memory.
// Live intervals are disjoint in the unchanged A schedule:
// a:316->317,317->318,326->327,410->411,366->367;
// b:400/404->407,366->367,368->369. These are scratch, never checkpoint state.
reg signed[32:0] r39t1_scratch33_a,r39t1_scratch33_b;
wire signed[32:0] encoder_residual=r39t1_scratch33_a;
// R29 motion-only: gain20/s, same0.2rad/s bound, alpha1/4 correction filter.
reg signed[31:0] feedback_lp,feedback_delta;

reg signed[31:0] encoder_velocity_correction,feedback_velocity;
// Bit-exact original position boost, pipelined across existing idle arithmetic slots.
reg signed[31:0] boost_clipped,boosted_position;wire signed[32:0] boost_wide=r39t1_scratch33_a;


// A30D1: bounded partial notch of dynamic feedback only; no added multiplier.
// All shaper quantities are native Q24 duty requests, not arm-angle states.
reg signed[31:0] shape_source,shape_fast,shape_slow,shape_fast_next,shape_slow_next,shape_correction;
wire signed[32:0] shape_fast_delta=r39t1_scratch33_b,shape_slow_delta=r39t1_scratch33_a,shape_band=r39t1_scratch33_b;
reg shape_valid;


wire shape_enabled=1'b1;

// A36D1 two-degree-of-freedom reference drive, exact A311 feedback unchanged.
// reference_sine is the original zero-centered sine position in INTERNAL Q16.
// It is distinct from the observer's capture-relative reference_offset.
// One entry-bias state removes the first-frame correction at a nonzero origin.
// All three debug values are committed with the observer output transaction.
reg sine_s,sine_valid;
reg signed[31:0] sine_reference_s,sine_bias,sine_bias_next,sine_position_request;
wire signed[32:0] sine_delta=r39t1_scratch33_a;
reg signed[31:0] sine_position_committed,sine_reference_committed;
wire sine_ff_enabled=motion_s && sine_s;
// A37: strengthen only the existing Kx band correction during the exact A36 sine.
// F/L state updates and the +/-671088 request bound remain unchanged. This adds
// no hidden state, reference modification, or actuator limit relaxation.
wire sine_damping_enabled=MANUAL_SINE_DAMPING && shape_enabled && sine_ff_enabled;





// A39M1: quantization-interval alpha/beta tracking. Internal position Q24;
// velocity_step is internal rad/ms Q24, so normalized n3 velocity is 100*step.
// The tracker warms in manual low-gain modes; only the fixed active sine can
// apply its bounded half-blend. A39 continuity keeps the same fixed bound at turns;
// the legacy path retains its reference-speed-dependent bound.
reg signed[31:0]smooth_position,smooth_velocity_step,smooth_position_next,smooth_velocity_step_next;
reg smooth_valid,smooth_sine_active_committed;
wire signed[32:0]smooth_prediction=r39t1_scratch33_b;
reg signed[31:0]smooth_normalized_next,smooth_previous_next,smooth_correction_next,smooth_bound_next;
reg signed[33:0]smooth_residual,smooth_error;
wire signed[32:0]smooth_velocity_delta=r39t1_scratch33_a;
wire signed[31:0]smooth_position_delta=boost_clipped;wire signed[27:0]smooth_step_delta=encoder_velocity_correction[27:0];reg signed[38:0]smooth_normalized_wide;
reg signed[31:0]smooth_normalized_committed,smooth_previous_committed,smooth_correction_committed,smooth_bound_committed,smooth_innovation_committed;
reg [31:0]smooth_dividend,smooth_quotient,smooth_seed_rounded;
reg [6:0]smooth_remainder;reg [5:0]smooth_divide_bits;reg smooth_seed_negative;
wire [7:0]smooth_trial_remainder={smooth_remainder,smooth_dividend[31]};
wire [31:0]smooth_reference_abs=velocity_s[31] ? (~velocity_s)+32'd1 : velocity_s;
wire signed[37:0]smooth_error_wide={{4{smooth_error[33]}},smooth_error};
wire signed[32:0]smooth_velocity_delta_wide=smooth_velocity_delta;
wire smooth_enabled=1'b1;




// Exact finite-range A arithmetic: |e| <= 3*2^31, so15e needs38bits,
// round(15e/64) fits signed32; round(e/64) fits signed28. No policy change.
function signed[31:0]smooth_round6;
 input signed[37:0]v;begin
 // Signed floor plus a tie-even carry equals magnitude-round-restore-sign,
 // including the minimum signed input. Only the retained 32 result bits add.
 smooth_round6=$signed(v[37:6])+((v[5:0]>6'd32)||((v[5:0]==6'd32)&&v[6]));end
endfunction
function signed[32:0]smooth_half;
 input signed[32:0]v;begin smooth_half=(v>>>1)+$signed({32'd0,(v[0]&&v[1])});end
endfunction
wire signed[33:0]smooth_position_sum=$signed({smooth_prediction[32],smooth_prediction})+$signed({{2{smooth_position_delta[31]}},smooth_position_delta});
wire signed[32:0]smooth_step_sum=$signed({smooth_velocity_step[31],smooth_velocity_step})+$signed({{5{smooth_step_delta[27]}},smooth_step_delta});



assign debug_ext2={{23'd0,r40_unwind_active,r40_transfer_ticks},smooth_innovation_committed,smooth_bound_committed,smooth_previous_committed,smooth_normalized_committed,smooth_correction_committed,smooth_velocity_step,smooth_position,sine_reference_committed,sine_position_committed,sine_bias,{26'd0,r39_mode,bias_clipped,smooth_sine_active_committed,smooth_valid,sine_valid,shape_valid},shape_slow,shape_fast,feedback_lp,z7};

wire[3:0] damping_weight;
wire r38_postmove_mode=(mature_static_hold===1'b1)&&automatic_policy&&!automatic_brake&&reference_velocity==0&&motion_assist_permission&&motion_latched&&motion_phase==3;
// Timing-only: sample start-domain predicates before any motion-latch change.
// Raw phi/ref are latched by the original input stage. Dead acc holds their
// widened difference at323;324 registers admission;state1 clocks the detector.
// All consumers are afterstate263, so output/transaction clocks are unchanged.
reg r38_damping_mode_s,r38_damping_default_s,r38_damping_theta_ok_s,r38_damping_rate_ok_s,r38_damping_eligible_q;

// OFFLINE risk candidate: exact32bit guard, no sampling/state/output change.
wire risk_bad_theta_s=(theta_s[31] ? ((!(&theta_s[31:15])) || theta_s[14:0] < 15'd12768) : ((|theta_s[31:15]) || theta_s[14:0] > 15'd20000));
wire risk_bad_phi_s=(phi_s[31] ? ((!(&phi_s[31:20])) || phi_s[19:0] < 20'd225026) : ((|phi_s[31:20]) || phi_s[19:0] > 20'd823550));
wire risk_bad_previous=(previous[31] ? ((!(&previous[31:9])) || previous[8:0] < 9'd162) : ((|previous[31:9]) || previous[8:0] > 9'd350));
wire risk_bad_reference_s=(reference_s[31] ? ((!(&reference_s[31:20])) || reference_s[19:0] < 20'd225026) : ((|reference_s[31:20]) || reference_s[19:0] > 20'd823550));
// Feedforward is frequency-weighted, not a measured/target angle. Check the
// exact signed Q16 range that can be shifted to Q24 without overflow.
wire risk_bad_sine_reference_s=sine_reference_s[31] ? !(&sine_reference_s[31:23]) : (|sine_reference_s[31:23]);
wire risk_bad_cap=(|cap[31:9]) || cap[8:0]<9'd150 || cap[8:0]>9'd350;
wire risk_bad_base=risk_bad_theta_s||risk_bad_phi_s||risk_bad_previous||risk_bad_cap||risk_bad_reference_s;
wire risk_bad_A=risk_bad_base||(sine_ff_enabled&&risk_bad_sine_reference_s);

always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin r38_damping_mode_s<=0;r38_damping_default_s<=0;r38_damping_theta_ok_s<=0;r38_damping_rate_ok_s<=0;r38_damping_eligible_q<=0;end
 else if(clear)begin r38_damping_mode_s<=0;r38_damping_default_s<=0;r38_damping_theta_ok_s<=0;r38_damping_rate_ok_s<=0;r38_damping_eligible_q<=0;end
 else begin
  if(start&&!busy&&!fault)begin
   r38_damping_mode_s<=r38_postmove_mode;
   r38_damping_default_s<=(mature_static_hold===1'b1)&&automatic_policy&&!automatic_brake&&reference_velocity==0&&!motion_latched;
   r38_damping_theta_ok_s<=theta<=9151&&theta>=-9151;
   r38_damping_rate_ok_s<=(measurement_current===1'b1)&&measured_omega<163840&&measured_omega> -163840;
  end
  if(busy&&state==9'd324)r38_damping_eligible_q<=r38_damping_mode_s ? (r38_damping_theta_ok_s&&r38_damping_rate_ok_s):(r38_damping_default_s&&r38_damping_theta_ok_s&&r39t5_data_cmp2&&r39t5_data_cmp3);
 end
end
r34_hold_damping_gate damping_gate(.clk(clk),.rst_n(rst_n),.clear(clear),.tick(busy&&state==9'd1&&!fault),.eligible(r38_damping_eligible_q),.postmove(r38_damping_mode_s),.theta(theta_s>>>8),.weight(damping_weight));
function signed[31:0] damping_gain_1;input[3:0]w;begin case(w)
4'd0:damping_gain_1=32'sd38753879;
4'd1:damping_gain_1=32'sd40691572;
4'd2:damping_gain_1=32'sd42629266;
4'd3:damping_gain_1=32'sd44566960;
default:damping_gain_1=32'sd38753879;endcase end endfunction

localparam signed[31:0] DEAD=298096;
reg signed[31:0] prediction_previous;
// Mutually exclusive original/A conversions share one unchanged 32-bit sum.
wire signed[31:0] r39t1_prior_source=prediction_previous;
wire signed[31:0] r39t1_prior_q=(r39t1_prior_source<<<14)+(r39t1_prior_source<<<8)+(r39t1_prior_source<<<7)+(r39t1_prior_source<<<3)+r39t1_prior_source;
// Seed is unsigned magnitude before sign restoration; retain all 33 bits.
// Both original x100 operations feed the same 39-bit destination on distinct states.
wire signed[32:0] r39t1_scale_input=(state==9'd403) ?
 (smooth_seed_negative ? -$signed({1'b0,smooth_seed_rounded}) : $signed({1'b0,smooth_seed_rounded})) :
 $signed({smooth_velocity_step_next[31],smooth_velocity_step_next});
wire signed[38:0] r39t1_scale_wide={{6{r39t1_scale_input[32]}},r39t1_scale_input};
wire signed[38:0] r39t1_scale100=(r39t1_scale_wide<<<6)+(r39t1_scale_wide<<<5)+(r39t1_scale_wide<<<2);
reg signed[31:0] output_slew;
reg signed[31:0] slew_low,slew_high;
reg manual_moved;
reg motion_watch;reg[4:0] motion_bad_ticks;
reg motion_latched;reg[2:0] motion_phase;reg[5:0] motion_ticks;
function signed[31:0] motion_gain_0; input[2:0] ix; begin case(ix)
3'd0:motion_gain_0=32'sd67937104;
3'd1:motion_gain_0=32'sd76298406;
3'd2:motion_gain_0=32'sd84659708;
3'd3:motion_gain_0=32'sd93021010;
3'd4:motion_gain_0=32'sd101382311;
3'd5:motion_gain_0=32'sd109743613;
3'd6:motion_gain_0=32'sd118104915;
3'd7:motion_gain_0=32'sd126466217;
default:motion_gain_0=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_1; input[2:0] ix; begin case(ix)
3'd0:motion_gain_1=32'sd38753879;
3'd1:motion_gain_1=32'sd49966655;
3'd2:motion_gain_1=32'sd61179432;
3'd3:motion_gain_1=32'sd72392208;
3'd4:motion_gain_1=32'sd83604984;
3'd5:motion_gain_1=32'sd94817760;
3'd6:motion_gain_1=32'sd106030537;
3'd7:motion_gain_1=32'sd117243313;
default:motion_gain_1=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_2; input[2:0] ix; begin case(ix)
3'd0:motion_gain_2=32'sd3568142;
3'd1:motion_gain_2=32'sd4703531;
3'd2:motion_gain_2=32'sd5838919;
3'd3:motion_gain_2=32'sd6974308;
3'd4:motion_gain_2=32'sd8109696;
3'd5:motion_gain_2=32'sd9245085;
3'd6:motion_gain_2=32'sd10380473;
3'd7:motion_gain_2=32'sd11515862;
default:motion_gain_2=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_3; input[2:0] ix; begin case(ix)
3'd0:motion_gain_3=32'sd30194271;
3'd1:motion_gain_3=32'sd38985989;
3'd2:motion_gain_3=32'sd47777706;
3'd3:motion_gain_3=32'sd56569424;
3'd4:motion_gain_3=32'sd65361142;
3'd5:motion_gain_3=32'sd74152860;
3'd6:motion_gain_3=32'sd82944577;
3'd7:motion_gain_3=32'sd91736295;
default:motion_gain_3=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_4; input[2:0] ix; begin case(ix)
3'd0:motion_gain_4=-32'sd268429;
3'd1:motion_gain_4=-32'sd336814;
3'd2:motion_gain_4=-32'sd405199;
3'd3:motion_gain_4=-32'sd473584;
3'd4:motion_gain_4=-32'sd541970;
3'd5:motion_gain_4=-32'sd610355;
3'd6:motion_gain_4=-32'sd678740;
3'd7:motion_gain_4=-32'sd747125;
default:motion_gain_4=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_5; input[2:0] ix; begin case(ix)
3'd0:motion_gain_5=-32'sd2104864;
3'd1:motion_gain_5=-32'sd2648833;
3'd2:motion_gain_5=-32'sd3192802;
3'd3:motion_gain_5=-32'sd3736771;
3'd4:motion_gain_5=-32'sd4280741;
3'd5:motion_gain_5=-32'sd4824710;
3'd6:motion_gain_5=-32'sd5368679;
3'd7:motion_gain_5=-32'sd5912648;
default:motion_gain_5=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_6; input[2:0] ix; begin case(ix)
3'd0:motion_gain_6=-32'sd2148228;
3'd1:motion_gain_6=-32'sd2842718;
3'd2:motion_gain_6=-32'sd3537209;
3'd3:motion_gain_6=-32'sd4231699;
3'd4:motion_gain_6=-32'sd4926190;
3'd5:motion_gain_6=-32'sd5620680;
3'd6:motion_gain_6=-32'sd6315171;
3'd7:motion_gain_6=-32'sd7009661;
default:motion_gain_6=32'sd0;endcase end endfunction
function signed[31:0] motion_gain_7; input[2:0] ix; begin case(ix)
3'd0:motion_gain_7=-32'sd3019427;
3'd1:motion_gain_7=-32'sd3786453;
3'd2:motion_gain_7=-32'sd4553479;
3'd3:motion_gain_7=-32'sd5320505;
3'd4:motion_gain_7=-32'sd6087530;
3'd5:motion_gain_7=-32'sd6854556;
3'd6:motion_gain_7=-32'sd7621582;
3'd7:motion_gain_7=-32'sd8388608;
default:motion_gain_7=32'sd0;endcase end endfunction
reg automatic_s,hold_s,brake_s,drift_s;reg mature_static_s;reg signed[31:0] blend,fallback_s;reg [7:0] samples;reg[9:0] age;
// R38: a mature explicit endpoint can accumulate stasis evidence while
// preserving the independent first-capture and moving-reference paths.
reg r38_explicit_endpoint_s,r38_current_s;
// R40: snapshot explicit positioning validity; no first-capture/Ext2 leakage.
reg r40_position_unwind_s;
always @(posedge clk or negedge rst_n) begin
 if(!rst_n)r40_position_unwind_s<=0;
 else if(clear)r40_position_unwind_s<=0;
 else if(start&&!busy&&!fault)r40_position_unwind_s<=automatic_policy&&!automatic_brake&&motion_assist_permission&&(measurement_current===1'b1)&&!(ext2_warm===1'b1);
end

always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin r38_explicit_endpoint_s<=0;r38_current_s<=0;end
 else if(clear)begin r38_explicit_endpoint_s<=0;r38_current_s<=0;end
 else if(start&&!busy&&!fault)begin
 r38_current_s<=(measurement_current===1'b1)&&measured_omega<=32'sd1966080&&measured_omega>=-32'sd1966080;
 r38_explicit_endpoint_s<=
  (mature_static_hold===1'b1)&&automatic_policy&&!automatic_brake&&
  motion_assist_permission&&reference_velocity==0&&(measurement_current===1'b1);
 end
end
reg signed[31:0] theta_gain,integral,integral_lp,integral_step,output_error,delivered,velocity_s;
assign debug_integral=integral;assign debug_lp=integral_lp;assign debug_age=age;
reg signed[31:0] phi_s,reference_s,position_error,previous,prior_q,cap,effective,tf_old,tf_new,ma,mb,request,wanted,bounded;
reg signed[63:0] product,conversion;
// At most12 signed40 Q24 product terms per accumulator epoch; the other
// epochs start from a signed33 difference or signed32 correction. Signed44
// covers the entire input domain (12*2^39 < 2^43), including every partial sum.
reg signed[43:0] acc;
reg [63:0] magnitude;reg [31:0] rounded;reg negative;
reg signed[31:0] z0,z1,z2,z3,z4,z5,z6,n0,n1,n2,n3,n4,n5,n6;
assign debug_state={z6,z5,z4,z3,z2,z1,z0};assign debug_samples=samples;
// R36 candidate: fast external-disturbance rate residual, mature static HOLD
// only. Require two current measurements; predicted/gap samples clear permission.
// z1 is normalized omega/10; all arithmetic below is signed, widened Q24.
reg r37_rate_was_good;
reg[7:0] r36_fast_good;reg r36_fast_enabled_s;reg signed[31:0] r36_omega_s;
wire r37_rate_eligible=((mature_static_hold===1'b1)||(rate_hold===1'b1))&&reference_velocity==0&&automatic_policy&&!automatic_brake&&
 (measurement_current===1'b1)&&
 measured_omega<=32'sd1966080&&measured_omega>=-32'sd1966080;
wire r36_fast_eligible=(mature_static_hold===1'b1)&&automatic_policy&&!automatic_brake&&
 reference_velocity==0&&!motion_latched&&!motion_assist_permission&&(measurement_current===1'b1)&&
 measured_omega<=32'sd1966080&&measured_omega>=-32'sd1966080;
wire signed[35:0] r36_observer_omega=($signed({{4{n1[31]}},n1})<<<3)+($signed({{4{n1[31]}},n1})<<<1);
// product is dead afterthelastmatrixmultiply untilstate266. Store the
// full-width innovation there at259, before any deadzone/shift/clamp logic.
// Read only after state259 stored measured32 - 10*n1 (signed37 exact).
wire signed[36:0] r36_rate_error=product[36:0];
// Admit a fast correction only for a substantial measured/model disagreement.
// R38:0.2rad/s dead zone,1/4 residual,125permille bound; mature static current only.
wire signed[36:0] r36_rate_shaped=r39t5_data_cmp4 ? r36_rate_error-37'sd13421773:
 (r39t5_data_cmp5 ? r36_rate_error+37'sd13421773:37'sd0);
wire r38_rate_boost=r36_omega_s<32'sd41943040&&r36_omega_s> -32'sd41943040&&r39t5_data_cmp6&&r39t5_data_cmp7&&(r39t5_data_cmp8||r39t5_data_cmp9);
// R43: start explicit-position correction from zero at the existing 0.8 rad/s gate.
wire signed[36:0] r38_strong_shaped=r36_rate_shaped;
wire signed[63:0] r38_rate_bound=coefficient_dynamic?64'sd2097152:64'sd1048576;
wire signed[63:0] r36_rate_correction=acc>r38_rate_bound?r38_rate_bound:(acc< -r38_rate_bound?-r38_rate_bound:acc);
// Explicit endpoint oscillation only: partly replace normalized omega/10,
// preserving the phase3 D coefficient. Maximum fraction30/64; the normalized
// delta bound243012 makes |D*delta|<1/16. Reuse tf_old after its lastmatrix
// read247: register delta at270, add at276, same shared multiply277.
wire signed[36:0] r38_rate_replace_base=r36_rate_error>>>6;
wire signed[36:0] r38_rate_replace_sum=damping_weight==3 ? (r38_rate_replace_base<<<1)+r38_rate_replace_base:
 (damping_weight==2 ? (r38_rate_replace_base<<<1):(damping_weight==1 ? r38_rate_replace_base:37'sd0));
wire signed[31:0] r38_rate_replace_delta=r39t5_data_cmp12 ? 32'sd243012:
 (r39t5_data_cmp13 ? -32'sd243012:r38_rate_replace_sum[31:0]);
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin r37_rate_was_good<=0;r36_fast_good<=0;r36_fast_enabled_s<=0;r36_omega_s<=0;end
 else if(clear)begin r37_rate_was_good<=0;r36_fast_good<=0;r36_fast_enabled_s<=0;r36_omega_s<=0;end
 else if(start&&!busy&&!fault)begin
  r36_fast_enabled_s<=r37_rate_eligible&&r37_rate_was_good;
  r37_rate_was_good<=r37_rate_eligible;
  r36_omega_s<=measured_omega<<<8;
  if(r36_fast_eligible)begin if(r36_fast_good<201)r36_fast_good<=r36_fast_good+1'b1;end
  else r36_fast_good<=0;
 end
end
function signed[31:0] sat;input signed[63:0] a;begin
 // Full 64-bit domain: representable iff every discarded bit equals bit31.
 sat=(a[63:31]=={33{a[31]}}) ? a[31:0] : a[63] ? 32'sh80000000 : 32'sh7fffffff;
end endfunction
// R41T1: retain ONE shared signed33 saturated adder. Predecode only
// its selectors on the existing predecessor clocks, before operand routing.
// Operands retain their original registers and can change on that same edge.
// 412 is reached from 409 (seed), 411 or 413 (continuity); shape is fixed on.
// These three unreset local FFs are consumed only by the listed valid states.
reg[2:0]r41t1_add_op;
always @(posedge clk)begin
 case(state)
 9'd319:r41t1_add_op<=1;
 9'd321:r41t1_add_op<=2;
 9'd369:r41t1_add_op<=3;
 9'd370:r41t1_add_op<=4;
 9'd409,9'd411,9'd413:r41t1_add_op<=5;
 default:r41t1_add_op<=0;
 endcase
end
reg signed[31:0] r39t1_add_a,r39t1_add_b;reg r39t1_sub;
always @*begin
 r39t1_add_a=n3;r39t1_add_b=encoder_velocity_correction;r39t1_sub=0;
 case(r41t1_add_op)
 3'd1:r39t1_add_b=feedback_lp;
 3'd2:begin r39t1_add_a=sine_position_request;r39t1_add_b=sine_position_request;end
 3'd3:begin r39t1_add_a=request;r39t1_add_b=shape_correction;r39t1_sub=1;end
 3'd4:begin r39t1_add_a=request;r39t1_add_b=sine_position_request;end
 3'd5:begin r39t1_add_a=smooth_previous_next;r39t1_add_b=smooth_correction_next;end
 default:begin end
 endcase
end
wire[32:0] r39t1_add_b_xor={r39t1_add_b[31],r39t1_add_b}^{33{r39t1_sub}};
wire signed[32:0] r39t1_add_sum={r39t1_add_a[31],r39t1_add_a}+r39t1_add_b_xor+r39t1_sub;
wire signed[31:0] r39t1_add_sat=(r39t1_add_sum[32]==r39t1_add_sum[31])?r39t1_add_sum[31:0]:{r39t1_add_sum[32],{31{!r39t1_add_sum[32]}}};
// MA input preparation uses an existing empty state; no new state or tick.
wire signed[32:0]r39t7_ma21_sum=$signed({n1[31],n1})+$signed({tf_old[31],tf_old});
reg signed[31:0]r39t7_ma21_prepared;
always @(posedge clk)begin
 if(rst_n&&!clear&&busy&&state==9'd275)
  r39t7_ma21_prepared<=(r39t7_ma21_sum[32]==r39t7_ma21_sum[31])?
    r39t7_ma21_sum[31:0]:{r39t7_ma21_sum[32],{31{!r39t7_ma21_sum[32]}}};
end
// R41T1: local unreset bound pipeline. No wide CE/reset/control fanout.
// Both bounds retain all 33 bits; no saturation/truncation before comparison.
reg signed[32:0] r41t1_smooth_high,r41t1_smooth_low;
always @(posedge clk)begin
 r41t1_smooth_high<=$signed({smooth_correction_committed[31],smooth_correction_committed})+33'sd8192;
 r41t1_smooth_low<=$signed({smooth_correction_committed[31],smooth_correction_committed})-33'sd8192;
end
// Shared banked table, seven registered steps, exact original integer lookup.
reg [31:0] lut_abs;reg [34:0] lut_scaled,lut_rounded;
reg [9:0] lut_address;reg lut_negative,lut_overflow;reg[24:0] lut_value;
reg [24:0] tanh_rom[0:1023] /* synthesis syn_romstyle = "block_rom" */;reg[24:0] tanh_word;
initial begin
tanh_rom[0]=25'd0;
tanh_rom[1]=25'd163835;
tanh_rom[2]=25'd327638;
tanh_rom[3]=25'd491379;
tanh_rom[4]=25'd655027;
tanh_rom[5]=25'd818550;
tanh_rom[6]=25'd981917;
tanh_rom[7]=25'd1145097;
tanh_rom[8]=25'd1308060;
tanh_rom[9]=25'd1470775;
tanh_rom[10]=25'd1633211;
tanh_rom[11]=25'd1795340;
tanh_rom[12]=25'd1957129;
tanh_rom[13]=25'd2118551;
tanh_rom[14]=25'd2279574;
tanh_rom[15]=25'd2440171;
tanh_rom[16]=25'd2600313;
tanh_rom[17]=25'd2759970;
tanh_rom[18]=25'd2919116;
tanh_rom[19]=25'd3077721;
tanh_rom[20]=25'd3235759;
tanh_rom[21]=25'd3393203;
tanh_rom[22]=25'd3550027;
tanh_rom[23]=25'd3706203;
tanh_rom[24]=25'd3861708;
tanh_rom[25]=25'd4016514;
tanh_rom[26]=25'd4170599;
tanh_rom[27]=25'd4323937;
tanh_rom[28]=25'd4476506;
tanh_rom[29]=25'd4628281;
tanh_rom[30]=25'd4779241;
tanh_rom[31]=25'd4929363;
tanh_rom[32]=25'd5078627;
tanh_rom[33]=25'd5227010;
tanh_rom[34]=25'd5374493;
tanh_rom[35]=25'd5521057;
tanh_rom[36]=25'd5666681;
tanh_rom[37]=25'd5811348;
tanh_rom[38]=25'd5955040;
tanh_rom[39]=25'd6097739;
tanh_rom[40]=25'd6239428;
tanh_rom[41]=25'd6380092;
tanh_rom[42]=25'd6519716;
tanh_rom[43]=25'd6658283;
tanh_rom[44]=25'd6795781;
tanh_rom[45]=25'd6932195;
tanh_rom[46]=25'd7067513;
tanh_rom[47]=25'd7201722;
tanh_rom[48]=25'd7334811;
tanh_rom[49]=25'd7466768;
tanh_rom[50]=25'd7597583;
tanh_rom[51]=25'd7727246;
tanh_rom[52]=25'd7855748;
tanh_rom[53]=25'd7983080;
tanh_rom[54]=25'd8109234;
tanh_rom[55]=25'd8234203;
tanh_rom[56]=25'd8357980;
tanh_rom[57]=25'd8480558;
tanh_rom[58]=25'd8601932;
tanh_rom[59]=25'd8722097;
tanh_rom[60]=25'd8841048;
tanh_rom[61]=25'd8958781;
tanh_rom[62]=25'd9075292;
tanh_rom[63]=25'd9190579;
tanh_rom[64]=25'd9304639;
tanh_rom[65]=25'd9417471;
tanh_rom[66]=25'd9529072;
tanh_rom[67]=25'd9639442;
tanh_rom[68]=25'd9748580;
tanh_rom[69]=25'd9856487;
tanh_rom[70]=25'd9963162;
tanh_rom[71]=25'd10068608;
tanh_rom[72]=25'd10172825;
tanh_rom[73]=25'd10275814;
tanh_rom[74]=25'd10377580;
tanh_rom[75]=25'd10478123;
tanh_rom[76]=25'd10577447;
tanh_rom[77]=25'd10675556;
tanh_rom[78]=25'd10772453;
tanh_rom[79]=25'd10868142;
tanh_rom[80]=25'd10962629;
tanh_rom[81]=25'd11055917;
tanh_rom[82]=25'd11148012;
tanh_rom[83]=25'd11238920;
tanh_rom[84]=25'd11328646;
tanh_rom[85]=25'd11417197;
tanh_rom[86]=25'd11504578;
tanh_rom[87]=25'd11590797;
tanh_rom[88]=25'd11675860;
tanh_rom[89]=25'd11759776;
tanh_rom[90]=25'd11842550;
tanh_rom[91]=25'd11924190;
tanh_rom[92]=25'd12004706;
tanh_rom[93]=25'd12084103;
tanh_rom[94]=25'd12162392;
tanh_rom[95]=25'd12239580;
tanh_rom[96]=25'd12315676;
tanh_rom[97]=25'd12390689;
tanh_rom[98]=25'd12464628;
tanh_rom[99]=25'd12537501;
tanh_rom[100]=25'd12609319;
tanh_rom[101]=25'd12680090;
tanh_rom[102]=25'd12749824;
tanh_rom[103]=25'd12818531;
tanh_rom[104]=25'd12886220;
tanh_rom[105]=25'd12952901;
tanh_rom[106]=25'd13018584;
tanh_rom[107]=25'd13083280;
tanh_rom[108]=25'd13146997;
tanh_rom[109]=25'd13209747;
tanh_rom[110]=25'd13271539;
tanh_rom[111]=25'd13332383;
tanh_rom[112]=25'd13392291;
tanh_rom[113]=25'd13451272;
tanh_rom[114]=25'd13509337;
tanh_rom[115]=25'd13566495;
tanh_rom[116]=25'd13622758;
tanh_rom[117]=25'd13678135;
tanh_rom[118]=25'd13732638;
tanh_rom[119]=25'd13786277;
tanh_rom[120]=25'd13839061;
tanh_rom[121]=25'd13891002;
tanh_rom[122]=25'd13942110;
tanh_rom[123]=25'd13992394;
tanh_rom[124]=25'd14041867;
tanh_rom[125]=25'd14090537;
tanh_rom[126]=25'd14138416;
tanh_rom[127]=25'd14185512;
tanh_rom[128]=25'd14231838;
tanh_rom[129]=25'd14277402;
tanh_rom[130]=25'd14322215;
tanh_rom[131]=25'd14366288;
tanh_rom[132]=25'd14409629;
tanh_rom[133]=25'd14452249;
tanh_rom[134]=25'd14494158;
tanh_rom[135]=25'd14535366;
tanh_rom[136]=25'd14575883;
tanh_rom[137]=25'd14615718;
tanh_rom[138]=25'd14654881;
tanh_rom[139]=25'd14693381;
tanh_rom[140]=25'd14731228;
tanh_rom[141]=25'd14768432;
tanh_rom[142]=25'd14805002;
tanh_rom[143]=25'd14840947;
tanh_rom[144]=25'd14876276;
tanh_rom[145]=25'd14910999;
tanh_rom[146]=25'd14945124;
tanh_rom[147]=25'd14978660;
tanh_rom[148]=25'd15011617;
tanh_rom[149]=25'd15044003;
tanh_rom[150]=25'd15075826;
tanh_rom[151]=25'd15107096;
tanh_rom[152]=25'd15137821;
tanh_rom[153]=25'd15168009;
tanh_rom[154]=25'd15197669;
tanh_rom[155]=25'd15226808;
tanh_rom[156]=25'd15255436;
tanh_rom[157]=25'd15283560;
tanh_rom[158]=25'd15311187;
tanh_rom[159]=25'd15338327;
tanh_rom[160]=25'd15364986;
tanh_rom[161]=25'd15391173;
tanh_rom[162]=25'd15416895;
tanh_rom[163]=25'd15442159;
tanh_rom[164]=25'd15466973;
tanh_rom[165]=25'd15491344;
tanh_rom[166]=25'd15515280;
tanh_rom[167]=25'd15538787;
tanh_rom[168]=25'd15561873;
tanh_rom[169]=25'd15584544;
tanh_rom[170]=25'd15606808;
tanh_rom[171]=25'd15628671;
tanh_rom[172]=25'd15650139;
tanh_rom[173]=25'd15671220;
tanh_rom[174]=25'd15691920;
tanh_rom[175]=25'd15712246;
tanh_rom[176]=25'd15732202;
tanh_rom[177]=25'd15751797;
tanh_rom[178]=25'd15771036;
tanh_rom[179]=25'd15789924;
tanh_rom[180]=25'd15808469;
tanh_rom[181]=25'd15826676;
tanh_rom[182]=25'd15844550;
tanh_rom[183]=25'd15862097;
tanh_rom[184]=25'd15879323;
tanh_rom[185]=25'd15896234;
tanh_rom[186]=25'd15912835;
tanh_rom[187]=25'd15929131;
tanh_rom[188]=25'd15945128;
tanh_rom[189]=25'd15960830;
tanh_rom[190]=25'd15976244;
tanh_rom[191]=25'd15991373;
tanh_rom[192]=25'd16006223;
tanh_rom[193]=25'd16020800;
tanh_rom[194]=25'd16035106;
tanh_rom[195]=25'd16049149;
tanh_rom[196]=25'd16062931;
tanh_rom[197]=25'd16076458;
tanh_rom[198]=25'd16089734;
tanh_rom[199]=25'd16102764;
tanh_rom[200]=25'd16115552;
tanh_rom[201]=25'd16128102;
tanh_rom[202]=25'd16140419;
tanh_rom[203]=25'd16152506;
tanh_rom[204]=25'd16164369;
tanh_rom[205]=25'd16176010;
tanh_rom[206]=25'd16187434;
tanh_rom[207]=25'd16198644;
tanh_rom[208]=25'd16209646;
tanh_rom[209]=25'd16220441;
tanh_rom[210]=25'd16231035;
tanh_rom[211]=25'd16241430;
tanh_rom[212]=25'd16251631;
tanh_rom[213]=25'd16261641;
tanh_rom[214]=25'd16271463;
tanh_rom[215]=25'd16281100;
tanh_rom[216]=25'd16290557;
tanh_rom[217]=25'd16299836;
tanh_rom[218]=25'd16308940;
tanh_rom[219]=25'd16317873;
tanh_rom[220]=25'd16326639;
tanh_rom[221]=25'd16335239;
tanh_rom[222]=25'd16343677;
tanh_rom[223]=25'd16351956;
tanh_rom[224]=25'd16360079;
tanh_rom[225]=25'd16368049;
tanh_rom[226]=25'd16375868;
tanh_rom[227]=25'd16383540;
tanh_rom[228]=25'd16391067;
tanh_rom[229]=25'd16398451;
tanh_rom[230]=25'd16405696;
tanh_rom[231]=25'd16412804;
tanh_rom[232]=25'd16419777;
tanh_rom[233]=25'd16426618;
tanh_rom[234]=25'd16433330;
tanh_rom[235]=25'd16439914;
tanh_rom[236]=25'd16446374;
tanh_rom[237]=25'd16452711;
tanh_rom[238]=25'd16458928;
tanh_rom[239]=25'd16465027;
tanh_rom[240]=25'd16471010;
tanh_rom[241]=25'd16476880;
tanh_rom[242]=25'd16482638;
tanh_rom[243]=25'd16488287;
tanh_rom[244]=25'd16493828;
tanh_rom[245]=25'd16499264;
tanh_rom[246]=25'd16504596;
tanh_rom[247]=25'd16509827;
tanh_rom[248]=25'd16514958;
tanh_rom[249]=25'd16519992;
tanh_rom[250]=25'd16524930;
tanh_rom[251]=25'd16529773;
tanh_rom[252]=25'd16534525;
tanh_rom[253]=25'd16539185;
tanh_rom[254]=25'd16543757;
tanh_rom[255]=25'd16548242;
tanh_rom[256]=25'd16552641;
tanh_rom[257]=25'd16556956;
tanh_rom[258]=25'd16561189;
tanh_rom[259]=25'd16565341;
tanh_rom[260]=25'd16569414;
tanh_rom[261]=25'd16573409;
tanh_rom[262]=25'd16577327;
tanh_rom[263]=25'd16581171;
tanh_rom[264]=25'd16584941;
tanh_rom[265]=25'd16588639;
tanh_rom[266]=25'd16592266;
tanh_rom[267]=25'd16595824;
tanh_rom[268]=25'd16599314;
tanh_rom[269]=25'd16602737;
tanh_rom[270]=25'd16606094;
tanh_rom[271]=25'd16609388;
tanh_rom[272]=25'd16612618;
tanh_rom[273]=25'd16615786;
tanh_rom[274]=25'd16618894;
tanh_rom[275]=25'd16621942;
tanh_rom[276]=25'd16624932;
tanh_rom[277]=25'd16627864;
tanh_rom[278]=25'd16630740;
tanh_rom[279]=25'd16633561;
tanh_rom[280]=25'd16636328;
tanh_rom[281]=25'd16639042;
tanh_rom[282]=25'd16641703;
tanh_rom[283]=25'd16644314;
tanh_rom[284]=25'd16646875;
tanh_rom[285]=25'd16649386;
tanh_rom[286]=25'd16651849;
tanh_rom[287]=25'd16654265;
tanh_rom[288]=25'd16656635;
tanh_rom[289]=25'd16658959;
tanh_rom[290]=25'd16661238;
tanh_rom[291]=25'd16663474;
tanh_rom[292]=25'd16665666;
tanh_rom[293]=25'd16667817;
tanh_rom[294]=25'd16669926;
tanh_rom[295]=25'd16671995;
tanh_rom[296]=25'd16674024;
tanh_rom[297]=25'd16676014;
tanh_rom[298]=25'd16677965;
tanh_rom[299]=25'd16679879;
tanh_rom[300]=25'd16681757;
tanh_rom[301]=25'd16683598;
tanh_rom[302]=25'd16685404;
tanh_rom[303]=25'd16687175;
tanh_rom[304]=25'd16688912;
tanh_rom[305]=25'd16690615;
tanh_rom[306]=25'd16692286;
tanh_rom[307]=25'd16693925;
tanh_rom[308]=25'd16695532;
tanh_rom[309]=25'd16697108;
tanh_rom[310]=25'd16698654;
tanh_rom[311]=25'd16700170;
tanh_rom[312]=25'd16701657;
tanh_rom[313]=25'd16703115;
tanh_rom[314]=25'd16704545;
tanh_rom[315]=25'd16705948;
tanh_rom[316]=25'd16707323;
tanh_rom[317]=25'd16708672;
tanh_rom[318]=25'd16709995;
tanh_rom[319]=25'd16711293;
tanh_rom[320]=25'd16712566;
tanh_rom[321]=25'd16713814;
tanh_rom[322]=25'd16715038;
tanh_rom[323]=25'd16716238;
tanh_rom[324]=25'd16717415;
tanh_rom[325]=25'd16718570;
tanh_rom[326]=25'd16719702;
tanh_rom[327]=25'd16720813;
tanh_rom[328]=25'd16721902;
tanh_rom[329]=25'd16722970;
tanh_rom[330]=25'd16724018;
tanh_rom[331]=25'd16725045;
tanh_rom[332]=25'd16726053;
tanh_rom[333]=25'd16727041;
tanh_rom[334]=25'd16728010;
tanh_rom[335]=25'd16728960;
tanh_rom[336]=25'd16729892;
tanh_rom[337]=25'd16730806;
tanh_rom[338]=25'd16731703;
tanh_rom[339]=25'd16732582;
tanh_rom[340]=25'd16733444;
tanh_rom[341]=25'd16734290;
tanh_rom[342]=25'd16735119;
tanh_rom[343]=25'd16735932;
tanh_rom[344]=25'd16736730;
tanh_rom[345]=25'd16737512;
tanh_rom[346]=25'd16738279;
tanh_rom[347]=25'd16739031;
tanh_rom[348]=25'd16739769;
tanh_rom[349]=25'd16740492;
tanh_rom[350]=25'd16741202;
tanh_rom[351]=25'd16741898;
tanh_rom[352]=25'd16742580;
tanh_rom[353]=25'd16743249;
tanh_rom[354]=25'd16743906;
tanh_rom[355]=25'd16744549;
tanh_rom[356]=25'd16745181;
tanh_rom[357]=25'd16745800;
tanh_rom[358]=25'd16746407;
tanh_rom[359]=25'd16747002;
tanh_rom[360]=25'd16747586;
tanh_rom[361]=25'd16748158;
tanh_rom[362]=25'd16748720;
tanh_rom[363]=25'd16749271;
tanh_rom[364]=25'd16749811;
tanh_rom[365]=25'd16750340;
tanh_rom[366]=25'd16750860;
tanh_rom[367]=25'd16751369;
tanh_rom[368]=25'd16751869;
tanh_rom[369]=25'd16752359;
tanh_rom[370]=25'd16752839;
tanh_rom[371]=25'd16753310;
tanh_rom[372]=25'd16753772;
tanh_rom[373]=25'd16754225;
tanh_rom[374]=25'd16754670;
tanh_rom[375]=25'd16755106;
tanh_rom[376]=25'd16755533;
tanh_rom[377]=25'd16755952;
tanh_rom[378]=25'd16756363;
tanh_rom[379]=25'd16756766;
tanh_rom[380]=25'd16757162;
tanh_rom[381]=25'd16757549;
tanh_rom[382]=25'd16757929;
tanh_rom[383]=25'd16758302;
tanh_rom[384]=25'd16758668;
tanh_rom[385]=25'd16759026;
tanh_rom[386]=25'd16759378;
tanh_rom[387]=25'd16759723;
tanh_rom[388]=25'd16760061;
tanh_rom[389]=25'd16760393;
tanh_rom[390]=25'd16760718;
tanh_rom[391]=25'd16761037;
tanh_rom[392]=25'd16761350;
tanh_rom[393]=25'd16761656;
tanh_rom[394]=25'd16761957;
tanh_rom[395]=25'd16762252;
tanh_rom[396]=25'd16762541;
tanh_rom[397]=25'd16762825;
tanh_rom[398]=25'd16763103;
tanh_rom[399]=25'd16763376;
tanh_rom[400]=25'd16763644;
tanh_rom[401]=25'd16763906;
tanh_rom[402]=25'd16764164;
tanh_rom[403]=25'd16764416;
tanh_rom[404]=25'd16764663;
tanh_rom[405]=25'd16764906;
tanh_rom[406]=25'd16765144;
tanh_rom[407]=25'd16765378;
tanh_rom[408]=25'd16765606;
tanh_rom[409]=25'd16765831;
tanh_rom[410]=25'd16766051;
tanh_rom[411]=25'd16766267;
tanh_rom[412]=25'd16766479;
tanh_rom[413]=25'd16766686;
tanh_rom[414]=25'd16766890;
tanh_rom[415]=25'd16767090;
tanh_rom[416]=25'd16767285;
tanh_rom[417]=25'd16767477;
tanh_rom[418]=25'd16767666;
tanh_rom[419]=25'd16767850;
tanh_rom[420]=25'd16768031;
tanh_rom[421]=25'd16768209;
tanh_rom[422]=25'd16768383;
tanh_rom[423]=25'd16768554;
tanh_rom[424]=25'd16768721;
tanh_rom[425]=25'd16768886;
tanh_rom[426]=25'd16769047;
tanh_rom[427]=25'd16769205;
tanh_rom[428]=25'd16769360;
tanh_rom[429]=25'd16769512;
tanh_rom[430]=25'd16769661;
tanh_rom[431]=25'd16769807;
tanh_rom[432]=25'd16769950;
tanh_rom[433]=25'd16770090;
tanh_rom[434]=25'd16770228;
tanh_rom[435]=25'd16770363;
tanh_rom[436]=25'd16770496;
tanh_rom[437]=25'd16770626;
tanh_rom[438]=25'd16770753;
tanh_rom[439]=25'd16770878;
tanh_rom[440]=25'd16771001;
tanh_rom[441]=25'd16771121;
tanh_rom[442]=25'd16771239;
tanh_rom[443]=25'd16771354;
tanh_rom[444]=25'd16771468;
tanh_rom[445]=25'd16771579;
tanh_rom[446]=25'd16771688;
tanh_rom[447]=25'd16771795;
tanh_rom[448]=25'd16771900;
tanh_rom[449]=25'd16772003;
tanh_rom[450]=25'd16772103;
tanh_rom[451]=25'd16772202;
tanh_rom[452]=25'd16772299;
tanh_rom[453]=25'd16772394;
tanh_rom[454]=25'd16772488;
tanh_rom[455]=25'd16772579;
tanh_rom[456]=25'd16772669;
tanh_rom[457]=25'd16772757;
tanh_rom[458]=25'd16772843;
tanh_rom[459]=25'd16772927;
tanh_rom[460]=25'd16773010;
tanh_rom[461]=25'd16773092;
tanh_rom[462]=25'd16773171;
tanh_rom[463]=25'd16773250;
tanh_rom[464]=25'd16773326;
tanh_rom[465]=25'd16773402;
tanh_rom[466]=25'd16773475;
tanh_rom[467]=25'd16773548;
tanh_rom[468]=25'd16773619;
tanh_rom[469]=25'd16773688;
tanh_rom[470]=25'd16773756;
tanh_rom[471]=25'd16773823;
tanh_rom[472]=25'd16773889;
tanh_rom[473]=25'd16773953;
tanh_rom[474]=25'd16774016;
tanh_rom[475]=25'd16774078;
tanh_rom[476]=25'd16774139;
tanh_rom[477]=25'd16774199;
tanh_rom[478]=25'd16774257;
tanh_rom[479]=25'd16774314;
tanh_rom[480]=25'd16774370;
tanh_rom[481]=25'd16774425;
tanh_rom[482]=25'd16774479;
tanh_rom[483]=25'd16774532;
tanh_rom[484]=25'd16774584;
tanh_rom[485]=25'd16774635;
tanh_rom[486]=25'd16774685;
tanh_rom[487]=25'd16774734;
tanh_rom[488]=25'd16774782;
tanh_rom[489]=25'd16774829;
tanh_rom[490]=25'd16774875;
tanh_rom[491]=25'd16774920;
tanh_rom[492]=25'd16774965;
tanh_rom[493]=25'd16775008;
tanh_rom[494]=25'd16775051;
tanh_rom[495]=25'd16775093;
tanh_rom[496]=25'd16775134;
tanh_rom[497]=25'd16775174;
tanh_rom[498]=25'd16775214;
tanh_rom[499]=25'd16775252;
tanh_rom[500]=25'd16775290;
tanh_rom[501]=25'd16775328;
tanh_rom[502]=25'd16775364;
tanh_rom[503]=25'd16775400;
tanh_rom[504]=25'd16775435;
tanh_rom[505]=25'd16775470;
tanh_rom[506]=25'd16775503;
tanh_rom[507]=25'd16775536;
tanh_rom[508]=25'd16775569;
tanh_rom[509]=25'd16775601;
tanh_rom[510]=25'd16775632;
tanh_rom[511]=25'd16775663;
tanh_rom[512]=25'd16775693;
tanh_rom[513]=25'd16775722;
tanh_rom[514]=25'd16775751;
tanh_rom[515]=25'd16775779;
tanh_rom[516]=25'd16775807;
tanh_rom[517]=25'd16775834;
tanh_rom[518]=25'd16775861;
tanh_rom[519]=25'd16775887;
tanh_rom[520]=25'd16775913;
tanh_rom[521]=25'd16775938;
tanh_rom[522]=25'd16775963;
tanh_rom[523]=25'd16775987;
tanh_rom[524]=25'd16776011;
tanh_rom[525]=25'd16776034;
tanh_rom[526]=25'd16776057;
tanh_rom[527]=25'd16776080;
tanh_rom[528]=25'd16776102;
tanh_rom[529]=25'd16776123;
tanh_rom[530]=25'd16776144;
tanh_rom[531]=25'd16776165;
tanh_rom[532]=25'd16776185;
tanh_rom[533]=25'd16776205;
tanh_rom[534]=25'd16776225;
tanh_rom[535]=25'd16776244;
tanh_rom[536]=25'd16776263;
tanh_rom[537]=25'd16776281;
tanh_rom[538]=25'd16776299;
tanh_rom[539]=25'd16776317;
tanh_rom[540]=25'd16776334;
tanh_rom[541]=25'd16776351;
tanh_rom[542]=25'd16776368;
tanh_rom[543]=25'd16776385;
tanh_rom[544]=25'd16776401;
tanh_rom[545]=25'd16776416;
tanh_rom[546]=25'd16776432;
tanh_rom[547]=25'd16776447;
tanh_rom[548]=25'd16776462;
tanh_rom[549]=25'd16776476;
tanh_rom[550]=25'd16776491;
tanh_rom[551]=25'd16776505;
tanh_rom[552]=25'd16776519;
tanh_rom[553]=25'd16776532;
tanh_rom[554]=25'd16776545;
tanh_rom[555]=25'd16776558;
tanh_rom[556]=25'd16776571;
tanh_rom[557]=25'd16776583;
tanh_rom[558]=25'd16776596;
tanh_rom[559]=25'd16776608;
tanh_rom[560]=25'd16776619;
tanh_rom[561]=25'd16776631;
tanh_rom[562]=25'd16776642;
tanh_rom[563]=25'd16776653;
tanh_rom[564]=25'd16776664;
tanh_rom[565]=25'd16776675;
tanh_rom[566]=25'd16776685;
tanh_rom[567]=25'd16776696;
tanh_rom[568]=25'd16776706;
tanh_rom[569]=25'd16776716;
tanh_rom[570]=25'd16776725;
tanh_rom[571]=25'd16776735;
tanh_rom[572]=25'd16776744;
tanh_rom[573]=25'd16776753;
tanh_rom[574]=25'd16776762;
tanh_rom[575]=25'd16776771;
tanh_rom[576]=25'd16776780;
tanh_rom[577]=25'd16776788;
tanh_rom[578]=25'd16776796;
tanh_rom[579]=25'd16776804;
tanh_rom[580]=25'd16776812;
tanh_rom[581]=25'd16776820;
tanh_rom[582]=25'd16776828;
tanh_rom[583]=25'd16776835;
tanh_rom[584]=25'd16776843;
tanh_rom[585]=25'd16776850;
tanh_rom[586]=25'd16776857;
tanh_rom[587]=25'd16776864;
tanh_rom[588]=25'd16776871;
tanh_rom[589]=25'd16776877;
tanh_rom[590]=25'd16776884;
tanh_rom[591]=25'd16776890;
tanh_rom[592]=25'd16776897;
tanh_rom[593]=25'd16776903;
tanh_rom[594]=25'd16776909;
tanh_rom[595]=25'd16776915;
tanh_rom[596]=25'd16776921;
tanh_rom[597]=25'd16776926;
tanh_rom[598]=25'd16776932;
tanh_rom[599]=25'd16776937;
tanh_rom[600]=25'd16776943;
tanh_rom[601]=25'd16776948;
tanh_rom[602]=25'd16776953;
tanh_rom[603]=25'd16776958;
tanh_rom[604]=25'd16776963;
tanh_rom[605]=25'd16776968;
tanh_rom[606]=25'd16776973;
tanh_rom[607]=25'd16776978;
tanh_rom[608]=25'd16776982;
tanh_rom[609]=25'd16776987;
tanh_rom[610]=25'd16776991;
tanh_rom[611]=25'd16776996;
tanh_rom[612]=25'd16777000;
tanh_rom[613]=25'd16777004;
tanh_rom[614]=25'd16777008;
tanh_rom[615]=25'd16777012;
tanh_rom[616]=25'd16777016;
tanh_rom[617]=25'd16777020;
tanh_rom[618]=25'd16777024;
tanh_rom[619]=25'd16777028;
tanh_rom[620]=25'd16777031;
tanh_rom[621]=25'd16777035;
tanh_rom[622]=25'd16777038;
tanh_rom[623]=25'd16777042;
tanh_rom[624]=25'd16777045;
tanh_rom[625]=25'd16777048;
tanh_rom[626]=25'd16777052;
tanh_rom[627]=25'd16777055;
tanh_rom[628]=25'd16777058;
tanh_rom[629]=25'd16777061;
tanh_rom[630]=25'd16777064;
tanh_rom[631]=25'd16777067;
tanh_rom[632]=25'd16777070;
tanh_rom[633]=25'd16777073;
tanh_rom[634]=25'd16777075;
tanh_rom[635]=25'd16777078;
tanh_rom[636]=25'd16777081;
tanh_rom[637]=25'd16777083;
tanh_rom[638]=25'd16777086;
tanh_rom[639]=25'd16777088;
tanh_rom[640]=25'd16777091;
tanh_rom[641]=25'd16777093;
tanh_rom[642]=25'd16777096;
tanh_rom[643]=25'd16777098;
tanh_rom[644]=25'd16777100;
tanh_rom[645]=25'd16777103;
tanh_rom[646]=25'd16777105;
tanh_rom[647]=25'd16777107;
tanh_rom[648]=25'd16777109;
tanh_rom[649]=25'd16777111;
tanh_rom[650]=25'd16777113;
tanh_rom[651]=25'd16777115;
tanh_rom[652]=25'd16777117;
tanh_rom[653]=25'd16777119;
tanh_rom[654]=25'd16777121;
tanh_rom[655]=25'd16777123;
tanh_rom[656]=25'd16777125;
tanh_rom[657]=25'd16777126;
tanh_rom[658]=25'd16777128;
tanh_rom[659]=25'd16777130;
tanh_rom[660]=25'd16777131;
tanh_rom[661]=25'd16777133;
tanh_rom[662]=25'd16777135;
tanh_rom[663]=25'd16777136;
tanh_rom[664]=25'd16777138;
tanh_rom[665]=25'd16777139;
tanh_rom[666]=25'd16777141;
tanh_rom[667]=25'd16777142;
tanh_rom[668]=25'd16777144;
tanh_rom[669]=25'd16777145;
tanh_rom[670]=25'd16777146;
tanh_rom[671]=25'd16777148;
tanh_rom[672]=25'd16777149;
tanh_rom[673]=25'd16777150;
tanh_rom[674]=25'd16777152;
tanh_rom[675]=25'd16777153;
tanh_rom[676]=25'd16777154;
tanh_rom[677]=25'd16777155;
tanh_rom[678]=25'd16777156;
tanh_rom[679]=25'd16777158;
tanh_rom[680]=25'd16777159;
tanh_rom[681]=25'd16777160;
tanh_rom[682]=25'd16777161;
tanh_rom[683]=25'd16777162;
tanh_rom[684]=25'd16777163;
tanh_rom[685]=25'd16777164;
tanh_rom[686]=25'd16777165;
tanh_rom[687]=25'd16777166;
tanh_rom[688]=25'd16777167;
tanh_rom[689]=25'd16777168;
tanh_rom[690]=25'd16777169;
tanh_rom[691]=25'd16777170;
tanh_rom[692]=25'd16777171;
tanh_rom[693]=25'd16777172;
tanh_rom[694]=25'd16777172;
tanh_rom[695]=25'd16777173;
tanh_rom[696]=25'd16777174;
tanh_rom[697]=25'd16777175;
tanh_rom[698]=25'd16777176;
tanh_rom[699]=25'd16777176;
tanh_rom[700]=25'd16777177;
tanh_rom[701]=25'd16777178;
tanh_rom[702]=25'd16777179;
tanh_rom[703]=25'd16777179;
tanh_rom[704]=25'd16777180;
tanh_rom[705]=25'd16777181;
tanh_rom[706]=25'd16777182;
tanh_rom[707]=25'd16777182;
tanh_rom[708]=25'd16777183;
tanh_rom[709]=25'd16777184;
tanh_rom[710]=25'd16777184;
tanh_rom[711]=25'd16777185;
tanh_rom[712]=25'd16777185;
tanh_rom[713]=25'd16777186;
tanh_rom[714]=25'd16777187;
tanh_rom[715]=25'd16777187;
tanh_rom[716]=25'd16777188;
tanh_rom[717]=25'd16777188;
tanh_rom[718]=25'd16777189;
tanh_rom[719]=25'd16777189;
tanh_rom[720]=25'd16777190;
tanh_rom[721]=25'd16777190;
tanh_rom[722]=25'd16777191;
tanh_rom[723]=25'd16777191;
tanh_rom[724]=25'd16777192;
tanh_rom[725]=25'd16777192;
tanh_rom[726]=25'd16777193;
tanh_rom[727]=25'd16777193;
tanh_rom[728]=25'd16777194;
tanh_rom[729]=25'd16777194;
tanh_rom[730]=25'd16777194;
tanh_rom[731]=25'd16777195;
tanh_rom[732]=25'd16777195;
tanh_rom[733]=25'd16777196;
tanh_rom[734]=25'd16777196;
tanh_rom[735]=25'd16777196;
tanh_rom[736]=25'd16777197;
tanh_rom[737]=25'd16777197;
tanh_rom[738]=25'd16777198;
tanh_rom[739]=25'd16777198;
tanh_rom[740]=25'd16777198;
tanh_rom[741]=25'd16777199;
tanh_rom[742]=25'd16777199;
tanh_rom[743]=25'd16777199;
tanh_rom[744]=25'd16777200;
tanh_rom[745]=25'd16777200;
tanh_rom[746]=25'd16777200;
tanh_rom[747]=25'd16777201;
tanh_rom[748]=25'd16777201;
tanh_rom[749]=25'd16777201;
tanh_rom[750]=25'd16777201;
tanh_rom[751]=25'd16777202;
tanh_rom[752]=25'd16777202;
tanh_rom[753]=25'd16777202;
tanh_rom[754]=25'd16777203;
tanh_rom[755]=25'd16777203;
tanh_rom[756]=25'd16777203;
tanh_rom[757]=25'd16777203;
tanh_rom[758]=25'd16777204;
tanh_rom[759]=25'd16777204;
tanh_rom[760]=25'd16777204;
tanh_rom[761]=25'd16777204;
tanh_rom[762]=25'd16777204;
tanh_rom[763]=25'd16777205;
tanh_rom[764]=25'd16777205;
tanh_rom[765]=25'd16777205;
tanh_rom[766]=25'd16777205;
tanh_rom[767]=25'd16777206;
tanh_rom[768]=25'd16777206;
tanh_rom[769]=25'd16777206;
tanh_rom[770]=25'd16777206;
tanh_rom[771]=25'd16777206;
tanh_rom[772]=25'd16777207;
tanh_rom[773]=25'd16777207;
tanh_rom[774]=25'd16777207;
tanh_rom[775]=25'd16777207;
tanh_rom[776]=25'd16777207;
tanh_rom[777]=25'd16777207;
tanh_rom[778]=25'd16777208;
tanh_rom[779]=25'd16777208;
tanh_rom[780]=25'd16777208;
tanh_rom[781]=25'd16777208;
tanh_rom[782]=25'd16777208;
tanh_rom[783]=25'd16777208;
tanh_rom[784]=25'd16777208;
tanh_rom[785]=25'd16777209;
tanh_rom[786]=25'd16777209;
tanh_rom[787]=25'd16777209;
tanh_rom[788]=25'd16777209;
tanh_rom[789]=25'd16777209;
tanh_rom[790]=25'd16777209;
tanh_rom[791]=25'd16777209;
tanh_rom[792]=25'd16777210;
tanh_rom[793]=25'd16777210;
tanh_rom[794]=25'd16777210;
tanh_rom[795]=25'd16777210;
tanh_rom[796]=25'd16777210;
tanh_rom[797]=25'd16777210;
tanh_rom[798]=25'd16777210;
tanh_rom[799]=25'd16777210;
tanh_rom[800]=25'd16777211;
tanh_rom[801]=25'd16777211;
tanh_rom[802]=25'd16777211;
tanh_rom[803]=25'd16777211;
tanh_rom[804]=25'd16777211;
tanh_rom[805]=25'd16777211;
tanh_rom[806]=25'd16777211;
tanh_rom[807]=25'd16777211;
tanh_rom[808]=25'd16777211;
tanh_rom[809]=25'd16777211;
tanh_rom[810]=25'd16777211;
tanh_rom[811]=25'd16777212;
tanh_rom[812]=25'd16777212;
tanh_rom[813]=25'd16777212;
tanh_rom[814]=25'd16777212;
tanh_rom[815]=25'd16777212;
tanh_rom[816]=25'd16777212;
tanh_rom[817]=25'd16777212;
tanh_rom[818]=25'd16777212;
tanh_rom[819]=25'd16777212;
tanh_rom[820]=25'd16777212;
tanh_rom[821]=25'd16777212;
tanh_rom[822]=25'd16777212;
tanh_rom[823]=25'd16777212;
tanh_rom[824]=25'd16777213;
tanh_rom[825]=25'd16777213;
tanh_rom[826]=25'd16777213;
tanh_rom[827]=25'd16777213;
tanh_rom[828]=25'd16777213;
tanh_rom[829]=25'd16777213;
tanh_rom[830]=25'd16777213;
tanh_rom[831]=25'd16777213;
tanh_rom[832]=25'd16777213;
tanh_rom[833]=25'd16777213;
tanh_rom[834]=25'd16777213;
tanh_rom[835]=25'd16777213;
tanh_rom[836]=25'd16777213;
tanh_rom[837]=25'd16777213;
tanh_rom[838]=25'd16777213;
tanh_rom[839]=25'd16777213;
tanh_rom[840]=25'd16777213;
tanh_rom[841]=25'd16777214;
tanh_rom[842]=25'd16777214;
tanh_rom[843]=25'd16777214;
tanh_rom[844]=25'd16777214;
tanh_rom[845]=25'd16777214;
tanh_rom[846]=25'd16777214;
tanh_rom[847]=25'd16777214;
tanh_rom[848]=25'd16777214;
tanh_rom[849]=25'd16777214;
tanh_rom[850]=25'd16777214;
tanh_rom[851]=25'd16777214;
tanh_rom[852]=25'd16777214;
tanh_rom[853]=25'd16777214;
tanh_rom[854]=25'd16777214;
tanh_rom[855]=25'd16777214;
tanh_rom[856]=25'd16777214;
tanh_rom[857]=25'd16777214;
tanh_rom[858]=25'd16777214;
tanh_rom[859]=25'd16777214;
tanh_rom[860]=25'd16777214;
tanh_rom[861]=25'd16777214;
tanh_rom[862]=25'd16777214;
tanh_rom[863]=25'd16777214;
tanh_rom[864]=25'd16777214;
tanh_rom[865]=25'd16777214;
tanh_rom[866]=25'd16777214;
tanh_rom[867]=25'd16777215;
tanh_rom[868]=25'd16777215;
tanh_rom[869]=25'd16777215;
tanh_rom[870]=25'd16777215;
tanh_rom[871]=25'd16777215;
tanh_rom[872]=25'd16777215;
tanh_rom[873]=25'd16777215;
tanh_rom[874]=25'd16777215;
tanh_rom[875]=25'd16777215;
tanh_rom[876]=25'd16777215;
tanh_rom[877]=25'd16777215;
tanh_rom[878]=25'd16777215;
tanh_rom[879]=25'd16777215;
tanh_rom[880]=25'd16777215;
tanh_rom[881]=25'd16777215;
tanh_rom[882]=25'd16777215;
tanh_rom[883]=25'd16777215;
tanh_rom[884]=25'd16777215;
tanh_rom[885]=25'd16777215;
tanh_rom[886]=25'd16777215;
tanh_rom[887]=25'd16777215;
tanh_rom[888]=25'd16777215;
tanh_rom[889]=25'd16777215;
tanh_rom[890]=25'd16777215;
tanh_rom[891]=25'd16777215;
tanh_rom[892]=25'd16777215;
tanh_rom[893]=25'd16777215;
tanh_rom[894]=25'd16777215;
tanh_rom[895]=25'd16777215;
tanh_rom[896]=25'd16777215;
tanh_rom[897]=25'd16777215;
tanh_rom[898]=25'd16777215;
tanh_rom[899]=25'd16777215;
tanh_rom[900]=25'd16777215;
tanh_rom[901]=25'd16777215;
tanh_rom[902]=25'd16777215;
tanh_rom[903]=25'd16777215;
tanh_rom[904]=25'd16777215;
tanh_rom[905]=25'd16777215;
tanh_rom[906]=25'd16777215;
tanh_rom[907]=25'd16777215;
tanh_rom[908]=25'd16777215;
tanh_rom[909]=25'd16777215;
tanh_rom[910]=25'd16777215;
tanh_rom[911]=25'd16777215;
tanh_rom[912]=25'd16777215;
tanh_rom[913]=25'd16777215;
tanh_rom[914]=25'd16777215;
tanh_rom[915]=25'd16777215;
tanh_rom[916]=25'd16777215;
tanh_rom[917]=25'd16777215;
tanh_rom[918]=25'd16777215;
tanh_rom[919]=25'd16777215;
tanh_rom[920]=25'd16777215;
tanh_rom[921]=25'd16777215;
tanh_rom[922]=25'd16777215;
tanh_rom[923]=25'd16777216;
tanh_rom[924]=25'd16777216;
tanh_rom[925]=25'd16777216;
tanh_rom[926]=25'd16777216;
tanh_rom[927]=25'd16777216;
tanh_rom[928]=25'd16777216;
tanh_rom[929]=25'd16777216;
tanh_rom[930]=25'd16777216;
tanh_rom[931]=25'd16777216;
tanh_rom[932]=25'd16777216;
tanh_rom[933]=25'd16777216;
tanh_rom[934]=25'd16777216;
tanh_rom[935]=25'd16777216;
tanh_rom[936]=25'd16777216;
tanh_rom[937]=25'd16777216;
tanh_rom[938]=25'd16777216;
tanh_rom[939]=25'd16777216;
tanh_rom[940]=25'd16777216;
tanh_rom[941]=25'd16777216;
tanh_rom[942]=25'd16777216;
tanh_rom[943]=25'd16777216;
tanh_rom[944]=25'd16777216;
tanh_rom[945]=25'd16777216;
tanh_rom[946]=25'd16777216;
tanh_rom[947]=25'd16777216;
tanh_rom[948]=25'd16777216;
tanh_rom[949]=25'd16777216;
tanh_rom[950]=25'd16777216;
tanh_rom[951]=25'd16777216;
tanh_rom[952]=25'd16777216;
tanh_rom[953]=25'd16777216;
tanh_rom[954]=25'd16777216;
tanh_rom[955]=25'd16777216;
tanh_rom[956]=25'd16777216;
tanh_rom[957]=25'd16777216;
tanh_rom[958]=25'd16777216;
tanh_rom[959]=25'd16777216;
tanh_rom[960]=25'd16777216;
tanh_rom[961]=25'd16777216;
tanh_rom[962]=25'd16777216;
tanh_rom[963]=25'd16777216;
tanh_rom[964]=25'd16777216;
tanh_rom[965]=25'd16777216;
tanh_rom[966]=25'd16777216;
tanh_rom[967]=25'd16777216;
tanh_rom[968]=25'd16777216;
tanh_rom[969]=25'd16777216;
tanh_rom[970]=25'd16777216;
tanh_rom[971]=25'd16777216;
tanh_rom[972]=25'd16777216;
tanh_rom[973]=25'd16777216;
tanh_rom[974]=25'd16777216;
tanh_rom[975]=25'd16777216;
tanh_rom[976]=25'd16777216;
tanh_rom[977]=25'd16777216;
tanh_rom[978]=25'd16777216;
tanh_rom[979]=25'd16777216;
tanh_rom[980]=25'd16777216;
tanh_rom[981]=25'd16777216;
tanh_rom[982]=25'd16777216;
tanh_rom[983]=25'd16777216;
tanh_rom[984]=25'd16777216;
tanh_rom[985]=25'd16777216;
tanh_rom[986]=25'd16777216;
tanh_rom[987]=25'd16777216;
tanh_rom[988]=25'd16777216;
tanh_rom[989]=25'd16777216;
tanh_rom[990]=25'd16777216;
tanh_rom[991]=25'd16777216;
tanh_rom[992]=25'd16777216;
tanh_rom[993]=25'd16777216;
tanh_rom[994]=25'd16777216;
tanh_rom[995]=25'd16777216;
tanh_rom[996]=25'd16777216;
tanh_rom[997]=25'd16777216;
tanh_rom[998]=25'd16777216;
tanh_rom[999]=25'd16777216;
tanh_rom[1000]=25'd16777216;
tanh_rom[1001]=25'd16777216;
tanh_rom[1002]=25'd16777216;
tanh_rom[1003]=25'd16777216;
tanh_rom[1004]=25'd16777216;
tanh_rom[1005]=25'd16777216;
tanh_rom[1006]=25'd16777216;
tanh_rom[1007]=25'd16777216;
tanh_rom[1008]=25'd16777216;
tanh_rom[1009]=25'd16777216;
tanh_rom[1010]=25'd16777216;
tanh_rom[1011]=25'd16777216;
tanh_rom[1012]=25'd16777216;
tanh_rom[1013]=25'd16777216;
tanh_rom[1014]=25'd16777216;
tanh_rom[1015]=25'd16777216;
tanh_rom[1016]=25'd16777216;
tanh_rom[1017]=25'd16777216;
tanh_rom[1018]=25'd16777216;
tanh_rom[1019]=25'd16777216;
tanh_rom[1020]=25'd16777216;
tanh_rom[1021]=25'd16777216;
tanh_rom[1022]=25'd16777216;
tanh_rom[1023]=25'd16777216;
end
always @(posedge clk) tanh_word<=tanh_rom[lut_address];
reg signed[31:0] coefficient_rom[0:1023] /* synthesis syn_romstyle = "block_rom" */;reg signed[31:0] coefficient_word;reg coefficient_dynamic;reg multiply_wait;reg signed[31:0] multiplier_b;wire signed[31:0] coefficient_operand=coefficient_dynamic ? mb : coefficient_word;
initial begin
coefficient_rom[0]=32'sd0;
coefficient_rom[1]=32'sd0;
coefficient_rom[2]=32'sd0;
coefficient_rom[3]=32'sd0;
coefficient_rom[4]=32'sd0;
coefficient_rom[5]=32'sd0;
coefficient_rom[6]=32'sd0;
coefficient_rom[7]=32'sd0;
coefficient_rom[8]=32'sd0;
coefficient_rom[9]=32'sd0;
coefficient_rom[10]=32'sd0;
coefficient_rom[11]=32'sd0;
coefficient_rom[12]=32'sd0;
coefficient_rom[13]=32'sd11286646;
coefficient_rom[14]=32'sd0;
coefficient_rom[15]=32'sd0;
coefficient_rom[16]=-32'sd31967;
coefficient_rom[17]=32'sd0;
coefficient_rom[18]=32'sd0;
coefficient_rom[19]=-32'sd16442;
coefficient_rom[20]=32'sd0;
coefficient_rom[21]=32'sd0;
coefficient_rom[22]=32'sd102789;
coefficient_rom[23]=32'sd0;
coefficient_rom[24]=32'sd0;
coefficient_rom[25]=-32'sd184055;
coefficient_rom[26]=32'sd0;
coefficient_rom[27]=32'sd0;
coefficient_rom[28]=32'sd272;
coefficient_rom[29]=32'sd0;
coefficient_rom[30]=32'sd0;
coefficient_rom[31]=32'sd1386;
coefficient_rom[32]=32'sd0;
coefficient_rom[33]=32'sd0;
coefficient_rom[34]=-32'sd546481;
coefficient_rom[35]=32'sd0;
coefficient_rom[36]=32'sd0;
coefficient_rom[37]=-32'sd28;
coefficient_rom[38]=32'sd0;
coefficient_rom[39]=32'sd0;
coefficient_rom[40]=32'sd5641830;
coefficient_rom[41]=32'sd0;
coefficient_rom[42]=32'sd0;
coefficient_rom[43]=32'sd16442;
coefficient_rom[44]=32'sd0;
coefficient_rom[45]=32'sd0;
coefficient_rom[46]=32'sd0;
coefficient_rom[47]=32'sd0;
coefficient_rom[48]=-32'sd34177534;
coefficient_rom[49]=32'sd0;
coefficient_rom[50]=32'sd0;
coefficient_rom[51]=32'sd15381777;
coefficient_rom[52]=32'sd0;
coefficient_rom[53]=32'sd0;
coefficient_rom[54]=-32'sd87634;
coefficient_rom[55]=32'sd0;
coefficient_rom[56]=32'sd0;
coefficient_rom[57]=32'sd643490;
coefficient_rom[58]=32'sd0;
coefficient_rom[59]=32'sd0;
coefficient_rom[60]=-32'sd1154404;
coefficient_rom[61]=32'sd0;
coefficient_rom[62]=32'sd0;
coefficient_rom[63]=-32'sd40426;
coefficient_rom[64]=32'sd0;
coefficient_rom[65]=32'sd0;
coefficient_rom[66]=-32'sd252652;
coefficient_rom[67]=32'sd0;
coefficient_rom[68]=32'sd0;
coefficient_rom[69]=-32'sd3418282;
coefficient_rom[70]=32'sd0;
coefficient_rom[71]=32'sd0;
coefficient_rom[72]=-32'sd42;
coefficient_rom[73]=32'sd0;
coefficient_rom[74]=32'sd0;
coefficient_rom[75]=32'sd35277151;
coefficient_rom[76]=32'sd0;
coefficient_rom[77]=32'sd0;
coefficient_rom[78]=32'sd87634;
coefficient_rom[79]=32'sd0;
coefficient_rom[80]=32'sd0;
coefficient_rom[81]=32'sd0;
coefficient_rom[82]=32'sd0;
coefficient_rom[83]=-32'sd83134;
coefficient_rom[84]=32'sd0;
coefficient_rom[85]=32'sd0;
coefficient_rom[86]=-32'sd2871;
coefficient_rom[87]=32'sd0;
coefficient_rom[88]=32'sd0;
coefficient_rom[89]=32'sd15690392;
coefficient_rom[90]=32'sd0;
coefficient_rom[91]=32'sd0;
coefficient_rom[92]=32'sd158363;
coefficient_rom[93]=32'sd0;
coefficient_rom[94]=32'sd0;
coefficient_rom[95]=-32'sd2279;
coefficient_rom[96]=32'sd0;
coefficient_rom[97]=32'sd0;
coefficient_rom[98]=32'sd6;
coefficient_rom[99]=32'sd0;
coefficient_rom[100]=32'sd0;
coefficient_rom[101]=32'sd42;
coefficient_rom[102]=32'sd0;
coefficient_rom[103]=32'sd0;
coefficient_rom[104]=-32'sd8022;
coefficient_rom[105]=32'sd0;
coefficient_rom[106]=32'sd0;
coefficient_rom[107]=-32'sd21;
coefficient_rom[108]=32'sd0;
coefficient_rom[109]=32'sd0;
coefficient_rom[110]=32'sd85266;
coefficient_rom[111]=32'sd0;
coefficient_rom[112]=32'sd0;
coefficient_rom[113]=32'sd1086824;
coefficient_rom[114]=32'sd0;
coefficient_rom[115]=32'sd0;
coefficient_rom[116]=32'sd0;
coefficient_rom[117]=32'sd0;
coefficient_rom[118]=-32'sd747153;
coefficient_rom[119]=32'sd0;
coefficient_rom[120]=32'sd0;
coefficient_rom[121]=32'sd3020;
coefficient_rom[122]=32'sd0;
coefficient_rom[123]=32'sd0;
coefficient_rom[124]=-32'sd904581;
coefficient_rom[125]=32'sd0;
coefficient_rom[126]=32'sd0;
coefficient_rom[127]=32'sd16760913;
coefficient_rom[128]=32'sd0;
coefficient_rom[129]=32'sd0;
coefficient_rom[130]=32'sd62472;
coefficient_rom[131]=32'sd0;
coefficient_rom[132]=32'sd0;
coefficient_rom[133]=32'sd29;
coefficient_rom[134]=32'sd0;
coefficient_rom[135]=32'sd0;
coefficient_rom[136]=32'sd119;
coefficient_rom[137]=32'sd0;
coefficient_rom[138]=32'sd0;
coefficient_rom[139]=-32'sd3103;
coefficient_rom[140]=32'sd0;
coefficient_rom[141]=32'sd0;
coefficient_rom[142]=-32'sd4383;
coefficient_rom[143]=32'sd0;
coefficient_rom[144]=32'sd0;
coefficient_rom[145]=32'sd736356;
coefficient_rom[146]=32'sd0;
coefficient_rom[147]=32'sd0;
coefficient_rom[148]=32'sd904581;
coefficient_rom[149]=32'sd0;
coefficient_rom[150]=32'sd0;
coefficient_rom[151]=32'sd0;
coefficient_rom[152]=32'sd0;
coefficient_rom[153]=-32'sd25403164;
coefficient_rom[154]=32'sd0;
coefficient_rom[155]=32'sd0;
coefficient_rom[156]=-32'sd920498;
coefficient_rom[157]=32'sd0;
coefficient_rom[158]=32'sd0;
coefficient_rom[159]=32'sd5012;
coefficient_rom[160]=32'sd0;
coefficient_rom[161]=32'sd0;
coefficient_rom[162]=32'sd476306;
coefficient_rom[163]=32'sd0;
coefficient_rom[164]=32'sd0;
coefficient_rom[165]=32'sd2857938;
coefficient_rom[166]=32'sd0;
coefficient_rom[167]=32'sd0;
coefficient_rom[168]=32'sd1961;
coefficient_rom[169]=32'sd0;
coefficient_rom[170]=32'sd0;
coefficient_rom[171]=32'sd13023;
coefficient_rom[172]=32'sd0;
coefficient_rom[173]=32'sd0;
coefficient_rom[174]=32'sd8460885;
coefficient_rom[175]=32'sd0;
coefficient_rom[176]=32'sd0;
coefficient_rom[177]=-32'sd130;
coefficient_rom[178]=32'sd0;
coefficient_rom[179]=32'sd0;
coefficient_rom[180]=32'sd26099228;
coefficient_rom[181]=32'sd0;
coefficient_rom[182]=32'sd0;
coefficient_rom[183]=-32'sd5012;
coefficient_rom[184]=32'sd0;
coefficient_rom[185]=32'sd0;
coefficient_rom[186]=32'sd0;
coefficient_rom[187]=32'sd0;
coefficient_rom[188]=-32'sd2054030;
coefficient_rom[189]=32'sd0;
coefficient_rom[190]=32'sd0;
coefficient_rom[191]=32'sd786677;
coefficient_rom[192]=32'sd0;
coefficient_rom[193]=32'sd0;
coefficient_rom[194]=32'sd99401;
coefficient_rom[195]=32'sd0;
coefficient_rom[196]=32'sd0;
coefficient_rom[197]=-32'sd569297;
coefficient_rom[198]=32'sd0;
coefficient_rom[199]=32'sd0;
coefficient_rom[200]=32'sd2336525;
coefficient_rom[201]=32'sd0;
coefficient_rom[202]=32'sd0;
coefficient_rom[203]=32'sd11974284;
coefficient_rom[204]=32'sd0;
coefficient_rom[205]=32'sd0;
coefficient_rom[206]=-32'sd6886;
coefficient_rom[207]=32'sd0;
coefficient_rom[208]=32'sd0;
coefficient_rom[209]=32'sd1919669;
coefficient_rom[210]=32'sd0;
coefficient_rom[211]=32'sd0;
coefficient_rom[212]=-32'sd125392;
coefficient_rom[213]=32'sd0;
coefficient_rom[214]=32'sd0;
coefficient_rom[215]=32'sd1215306;
coefficient_rom[216]=32'sd0;
coefficient_rom[217]=32'sd0;
coefficient_rom[218]=-32'sd99401;
coefficient_rom[219]=32'sd0;
coefficient_rom[220]=32'sd0;
coefficient_rom[221]=32'sd0;
coefficient_rom[222]=32'sd0;
coefficient_rom[223]=-32'sd1384029;
coefficient_rom[224]=32'sd0;
coefficient_rom[225]=32'sd0;
coefficient_rom[226]=32'sd87381;
coefficient_rom[227]=32'sd0;
coefficient_rom[228]=32'sd0;
coefficient_rom[229]=32'sd97028;
coefficient_rom[230]=32'sd0;
coefficient_rom[231]=32'sd0;
coefficient_rom[232]=-32'sd69994;
coefficient_rom[233]=32'sd0;
coefficient_rom[234]=32'sd0;
coefficient_rom[235]=32'sd435582;
coefficient_rom[236]=32'sd0;
coefficient_rom[237]=32'sd0;
coefficient_rom[238]=32'sd4037998;
coefficient_rom[239]=32'sd0;
coefficient_rom[240]=32'sd0;
coefficient_rom[241]=32'sd11974831;
coefficient_rom[242]=32'sd0;
coefficient_rom[243]=32'sd0;
coefficient_rom[244]=32'sd117084;
coefficient_rom[245]=32'sd0;
coefficient_rom[246]=32'sd0;
coefficient_rom[247]=-32'sd19965;
coefficient_rom[248]=32'sd0;
coefficient_rom[249]=32'sd0;
coefficient_rom[250]=32'sd1279261;
coefficient_rom[251]=32'sd0;
coefficient_rom[252]=32'sd0;
coefficient_rom[253]=-32'sd97028;
coefficient_rom[254]=32'sd0;
coefficient_rom[255]=32'sd0;
coefficient_rom[256]=32'sd0;
coefficient_rom[257]=32'sd0;
coefficient_rom[258]=32'sd0;
coefficient_rom[259]=32'sd0;
coefficient_rom[260]=32'sd0;
coefficient_rom[261]=32'sd0;
coefficient_rom[262]=32'sd0;
coefficient_rom[263]=32'sd0;
coefficient_rom[264]=32'sd0;
coefficient_rom[265]=32'sd16760;
coefficient_rom[266]=32'sd0;
coefficient_rom[267]=32'sd0;
coefficient_rom[268]=32'sd4194;
coefficient_rom[269]=32'sd0;
coefficient_rom[270]=32'sd0;
coefficient_rom[271]=32'sd0;
coefficient_rom[272]=32'sd0;
coefficient_rom[273]=32'sd0;
coefficient_rom[274]=32'sd0;
coefficient_rom[275]=32'sd0;
coefficient_rom[276]=32'sd117243313;
coefficient_rom[277]=32'sd0;
coefficient_rom[278]=32'sd0;
coefficient_rom[279]=32'sd11515862;
coefficient_rom[280]=32'sd0;
coefficient_rom[281]=32'sd0;
coefficient_rom[282]=32'sd91736295;
coefficient_rom[283]=32'sd0;
coefficient_rom[284]=32'sd0;
coefficient_rom[285]=-32'sd747125;
coefficient_rom[286]=32'sd0;
coefficient_rom[287]=32'sd0;
coefficient_rom[288]=-32'sd5912648;
coefficient_rom[289]=32'sd0;
coefficient_rom[290]=32'sd0;
coefficient_rom[291]=-32'sd7009661;
coefficient_rom[292]=32'sd0;
coefficient_rom[293]=32'sd0;
coefficient_rom[294]=32'sd475651;
coefficient_rom[295]=32'sd0;
coefficient_rom[296]=32'sd0;
coefficient_rom[297]=32'sd16777216;
coefficient_rom[298]=32'sd0;
coefficient_rom[299]=32'sd0;
coefficient_rom[300]=-32'sd8388608;
coefficient_rom[301]=32'sd0;
coefficient_rom[302]=32'sd0;
coefficient_rom[303]=32'sd0;
coefficient_rom[304]=32'sd0;
coefficient_rom[305]=32'sd0;
coefficient_rom[306]=32'sd0;
coefficient_rom[307]=32'sd0;
coefficient_rom[308]=32'sd0;
coefficient_rom[309]=32'sd0;
coefficient_rom[310]=32'sd0;
coefficient_rom[311]=32'sd0;
coefficient_rom[312]=32'sd0;
coefficient_rom[313]=32'sd16777;
coefficient_rom[314]=32'sd0;
coefficient_rom[315]=32'sd0;
coefficient_rom[316]=32'sd0;
coefficient_rom[317]=32'sd0;
coefficient_rom[318]=32'sd0;
coefficient_rom[319]=32'sd0;
coefficient_rom[320]=32'sd0;
coefficient_rom[321]=32'sd0;
coefficient_rom[322]=32'sd0;
coefficient_rom[323]=32'sd0;
coefficient_rom[324]=32'sd0;
coefficient_rom[325]=32'sd0;
coefficient_rom[326]=32'sd0;
coefficient_rom[327]=32'sd0;
coefficient_rom[328]=32'sd0;
coefficient_rom[329]=32'sd0;
coefficient_rom[330]=32'sd0;
coefficient_rom[331]=32'sd0;
coefficient_rom[332]=32'sd0;
coefficient_rom[333]=32'sd0;
coefficient_rom[334]=32'sd0;
coefficient_rom[335]=32'sd0;
coefficient_rom[336]=32'sd0;
coefficient_rom[337]=32'sd0;
coefficient_rom[338]=32'sd0;
coefficient_rom[339]=32'sd0;
coefficient_rom[340]=32'sd0;
coefficient_rom[341]=32'sd0;
coefficient_rom[342]=32'sd0;
coefficient_rom[343]=32'sd0;
coefficient_rom[344]=32'sd0;
coefficient_rom[345]=32'sd0;
coefficient_rom[346]=32'sd0;
coefficient_rom[347]=32'sd0;
coefficient_rom[348]=32'sd0;
coefficient_rom[349]=32'sd0;
coefficient_rom[350]=32'sd0;
coefficient_rom[351]=32'sd0;
coefficient_rom[352]=32'sd0;
coefficient_rom[353]=32'sd0;
coefficient_rom[354]=32'sd0;
coefficient_rom[355]=32'sd0;
coefficient_rom[356]=32'sd0;
coefficient_rom[357]=32'sd0;
coefficient_rom[358]=32'sd0;
coefficient_rom[359]=32'sd0;
coefficient_rom[360]=32'sd0;
coefficient_rom[361]=32'sd0;
coefficient_rom[362]=32'sd0;
coefficient_rom[363]=32'sd0;
coefficient_rom[364]=32'sd0;
coefficient_rom[365]=32'sd0;
coefficient_rom[366]=32'sd0;
coefficient_rom[367]=32'sd0;
coefficient_rom[368]=32'sd0;
coefficient_rom[369]=32'sd0;
coefficient_rom[370]=32'sd0;
coefficient_rom[371]=32'sd0;
coefficient_rom[372]=32'sd0;
coefficient_rom[373]=32'sd0;
coefficient_rom[374]=32'sd0;
coefficient_rom[375]=32'sd0;
coefficient_rom[376]=32'sd0;
coefficient_rom[377]=32'sd0;
coefficient_rom[378]=32'sd0;
coefficient_rom[379]=32'sd0;
coefficient_rom[380]=32'sd0;
coefficient_rom[381]=32'sd0;
coefficient_rom[382]=32'sd0;
coefficient_rom[383]=32'sd0;
coefficient_rom[384]=32'sd0;
coefficient_rom[385]=32'sd0;
coefficient_rom[386]=32'sd0;
coefficient_rom[387]=32'sd0;
coefficient_rom[388]=32'sd0;
coefficient_rom[389]=32'sd0;
coefficient_rom[390]=32'sd0;
coefficient_rom[391]=32'sd0;
coefficient_rom[392]=32'sd0;
coefficient_rom[393]=32'sd0;
coefficient_rom[394]=32'sd0;
coefficient_rom[395]=32'sd0;
coefficient_rom[396]=32'sd0;
coefficient_rom[397]=32'sd0;
coefficient_rom[398]=32'sd0;
coefficient_rom[399]=32'sd0;
coefficient_rom[400]=32'sd0;
coefficient_rom[401]=32'sd0;
coefficient_rom[402]=32'sd0;
coefficient_rom[403]=32'sd0;
coefficient_rom[404]=32'sd0;
coefficient_rom[405]=32'sd0;
coefficient_rom[406]=32'sd0;
coefficient_rom[407]=32'sd0;
coefficient_rom[408]=32'sd0;
coefficient_rom[409]=32'sd0;
coefficient_rom[410]=32'sd0;
coefficient_rom[411]=32'sd0;
coefficient_rom[412]=32'sd0;
coefficient_rom[413]=32'sd0;
coefficient_rom[414]=32'sd0;
coefficient_rom[415]=32'sd0;
coefficient_rom[416]=32'sd0;
coefficient_rom[417]=32'sd0;
coefficient_rom[418]=32'sd0;
coefficient_rom[419]=32'sd0;
coefficient_rom[420]=32'sd0;
coefficient_rom[421]=32'sd0;
coefficient_rom[422]=32'sd0;
coefficient_rom[423]=32'sd0;
coefficient_rom[424]=32'sd0;
coefficient_rom[425]=32'sd0;
coefficient_rom[426]=32'sd0;
coefficient_rom[427]=32'sd0;
coefficient_rom[428]=32'sd0;
coefficient_rom[429]=32'sd0;
coefficient_rom[430]=32'sd0;
coefficient_rom[431]=32'sd0;
coefficient_rom[432]=32'sd0;
coefficient_rom[433]=32'sd0;
coefficient_rom[434]=32'sd0;
coefficient_rom[435]=32'sd0;
coefficient_rom[436]=32'sd0;
coefficient_rom[437]=32'sd0;
coefficient_rom[438]=32'sd0;
coefficient_rom[439]=32'sd0;
coefficient_rom[440]=32'sd0;
coefficient_rom[441]=32'sd0;
coefficient_rom[442]=32'sd0;
coefficient_rom[443]=32'sd0;
coefficient_rom[444]=32'sd0;
coefficient_rom[445]=32'sd0;
coefficient_rom[446]=32'sd0;
coefficient_rom[447]=32'sd0;
coefficient_rom[448]=32'sd0;
coefficient_rom[449]=32'sd0;
coefficient_rom[450]=32'sd0;
coefficient_rom[451]=32'sd0;
coefficient_rom[452]=32'sd0;
coefficient_rom[453]=32'sd0;
coefficient_rom[454]=32'sd0;
coefficient_rom[455]=32'sd0;
coefficient_rom[456]=32'sd0;
coefficient_rom[457]=32'sd0;
coefficient_rom[458]=32'sd0;
coefficient_rom[459]=32'sd0;
coefficient_rom[460]=32'sd0;
coefficient_rom[461]=32'sd0;
coefficient_rom[462]=32'sd0;
coefficient_rom[463]=32'sd0;
coefficient_rom[464]=32'sd0;
coefficient_rom[465]=32'sd0;
coefficient_rom[466]=32'sd0;
coefficient_rom[467]=32'sd0;
coefficient_rom[468]=32'sd0;
coefficient_rom[469]=32'sd0;
coefficient_rom[470]=32'sd0;
coefficient_rom[471]=32'sd0;
coefficient_rom[472]=32'sd0;
coefficient_rom[473]=32'sd0;
coefficient_rom[474]=32'sd0;
coefficient_rom[475]=32'sd0;
coefficient_rom[476]=32'sd0;
coefficient_rom[477]=32'sd0;
coefficient_rom[478]=32'sd0;
coefficient_rom[479]=32'sd0;
coefficient_rom[480]=32'sd0;
coefficient_rom[481]=32'sd0;
coefficient_rom[482]=32'sd0;
coefficient_rom[483]=32'sd0;
coefficient_rom[484]=32'sd0;
coefficient_rom[485]=32'sd0;
coefficient_rom[486]=32'sd0;
coefficient_rom[487]=32'sd0;
coefficient_rom[488]=32'sd0;
coefficient_rom[489]=32'sd0;
coefficient_rom[490]=32'sd0;
coefficient_rom[491]=32'sd0;
coefficient_rom[492]=32'sd0;
coefficient_rom[493]=32'sd0;
coefficient_rom[494]=32'sd0;
coefficient_rom[495]=32'sd0;
coefficient_rom[496]=32'sd0;
coefficient_rom[497]=32'sd0;
coefficient_rom[498]=32'sd0;
coefficient_rom[499]=32'sd0;
coefficient_rom[500]=32'sd0;
coefficient_rom[501]=32'sd0;
coefficient_rom[502]=32'sd0;
coefficient_rom[503]=32'sd0;
coefficient_rom[504]=32'sd0;
coefficient_rom[505]=32'sd0;
coefficient_rom[506]=32'sd0;
coefficient_rom[507]=32'sd0;
coefficient_rom[508]=32'sd0;
coefficient_rom[509]=32'sd0;
coefficient_rom[510]=32'sd0;
coefficient_rom[511]=32'sd0;
end
initial begin
coefficient_rom[512]=32'sd0;
coefficient_rom[513]=32'sd0;
coefficient_rom[514]=32'sd0;
coefficient_rom[515]=32'sd0;
coefficient_rom[516]=32'sd0;
coefficient_rom[517]=32'sd0;
coefficient_rom[518]=32'sd0;
coefficient_rom[519]=32'sd0;
coefficient_rom[520]=32'sd0;
coefficient_rom[521]=32'sd0;
coefficient_rom[522]=32'sd0;
coefficient_rom[523]=32'sd0;
coefficient_rom[524]=32'sd0;
coefficient_rom[525]=32'sd11286646;
coefficient_rom[526]=32'sd0;
coefficient_rom[527]=32'sd0;
coefficient_rom[528]=-32'sd31967;
coefficient_rom[529]=32'sd0;
coefficient_rom[530]=32'sd0;
coefficient_rom[531]=-32'sd16442;
coefficient_rom[532]=32'sd0;
coefficient_rom[533]=32'sd0;
coefficient_rom[534]=32'sd102789;
coefficient_rom[535]=32'sd0;
coefficient_rom[536]=32'sd0;
coefficient_rom[537]=-32'sd184055;
coefficient_rom[538]=32'sd0;
coefficient_rom[539]=32'sd0;
coefficient_rom[540]=32'sd272;
coefficient_rom[541]=32'sd0;
coefficient_rom[542]=32'sd0;
coefficient_rom[543]=32'sd1386;
coefficient_rom[544]=32'sd0;
coefficient_rom[545]=32'sd0;
coefficient_rom[546]=-32'sd546481;
coefficient_rom[547]=32'sd0;
coefficient_rom[548]=32'sd0;
coefficient_rom[549]=-32'sd28;
coefficient_rom[550]=32'sd0;
coefficient_rom[551]=32'sd0;
coefficient_rom[552]=32'sd5641830;
coefficient_rom[553]=32'sd0;
coefficient_rom[554]=32'sd0;
coefficient_rom[555]=32'sd16442;
coefficient_rom[556]=32'sd0;
coefficient_rom[557]=32'sd0;
coefficient_rom[558]=-32'sd83392;
coefficient_rom[559]=32'sd0;
coefficient_rom[560]=-32'sd34177534;
coefficient_rom[561]=32'sd0;
coefficient_rom[562]=32'sd0;
coefficient_rom[563]=32'sd15381777;
coefficient_rom[564]=32'sd0;
coefficient_rom[565]=32'sd0;
coefficient_rom[566]=-32'sd87634;
coefficient_rom[567]=32'sd0;
coefficient_rom[568]=32'sd0;
coefficient_rom[569]=32'sd643490;
coefficient_rom[570]=32'sd0;
coefficient_rom[571]=32'sd0;
coefficient_rom[572]=-32'sd1154404;
coefficient_rom[573]=32'sd0;
coefficient_rom[574]=32'sd0;
coefficient_rom[575]=-32'sd40426;
coefficient_rom[576]=32'sd0;
coefficient_rom[577]=32'sd0;
coefficient_rom[578]=-32'sd252652;
coefficient_rom[579]=32'sd0;
coefficient_rom[580]=32'sd0;
coefficient_rom[581]=-32'sd3418282;
coefficient_rom[582]=32'sd0;
coefficient_rom[583]=32'sd0;
coefficient_rom[584]=-32'sd42;
coefficient_rom[585]=32'sd0;
coefficient_rom[586]=32'sd0;
coefficient_rom[587]=32'sd35277151;
coefficient_rom[588]=32'sd0;
coefficient_rom[589]=32'sd0;
coefficient_rom[590]=32'sd87634;
coefficient_rom[591]=32'sd0;
coefficient_rom[592]=32'sd0;
coefficient_rom[593]=-32'sd521939;
coefficient_rom[594]=32'sd0;
coefficient_rom[595]=-32'sd83134;
coefficient_rom[596]=32'sd0;
coefficient_rom[597]=32'sd0;
coefficient_rom[598]=-32'sd2871;
coefficient_rom[599]=32'sd0;
coefficient_rom[600]=32'sd0;
coefficient_rom[601]=32'sd15690392;
coefficient_rom[602]=32'sd0;
coefficient_rom[603]=32'sd0;
coefficient_rom[604]=32'sd158363;
coefficient_rom[605]=32'sd0;
coefficient_rom[606]=32'sd0;
coefficient_rom[607]=-32'sd2279;
coefficient_rom[608]=32'sd0;
coefficient_rom[609]=32'sd0;
coefficient_rom[610]=32'sd6;
coefficient_rom[611]=32'sd0;
coefficient_rom[612]=32'sd0;
coefficient_rom[613]=32'sd42;
coefficient_rom[614]=32'sd0;
coefficient_rom[615]=32'sd0;
coefficient_rom[616]=-32'sd8022;
coefficient_rom[617]=32'sd0;
coefficient_rom[618]=32'sd0;
coefficient_rom[619]=-32'sd21;
coefficient_rom[620]=32'sd0;
coefficient_rom[621]=32'sd0;
coefficient_rom[622]=32'sd85266;
coefficient_rom[623]=32'sd0;
coefficient_rom[624]=32'sd0;
coefficient_rom[625]=32'sd1086824;
coefficient_rom[626]=32'sd0;
coefficient_rom[627]=32'sd0;
coefficient_rom[628]=-32'sd1182;
coefficient_rom[629]=32'sd0;
coefficient_rom[630]=-32'sd747152;
coefficient_rom[631]=32'sd0;
coefficient_rom[632]=32'sd0;
coefficient_rom[633]=32'sd3019;
coefficient_rom[634]=32'sd0;
coefficient_rom[635]=32'sd0;
coefficient_rom[636]=-32'sd995040;
coefficient_rom[637]=32'sd0;
coefficient_rom[638]=32'sd0;
coefficient_rom[639]=32'sd16760009;
coefficient_rom[640]=32'sd0;
coefficient_rom[641]=32'sd0;
coefficient_rom[642]=32'sd62469;
coefficient_rom[643]=32'sd0;
coefficient_rom[644]=32'sd0;
coefficient_rom[645]=32'sd29;
coefficient_rom[646]=32'sd0;
coefficient_rom[647]=32'sd0;
coefficient_rom[648]=32'sd119;
coefficient_rom[649]=32'sd0;
coefficient_rom[650]=32'sd0;
coefficient_rom[651]=-32'sd3105;
coefficient_rom[652]=32'sd0;
coefficient_rom[653]=32'sd0;
coefficient_rom[654]=-32'sd4383;
coefficient_rom[655]=32'sd0;
coefficient_rom[656]=32'sd0;
coefficient_rom[657]=32'sd736356;
coefficient_rom[658]=32'sd0;
coefficient_rom[659]=32'sd0;
coefficient_rom[660]=32'sd995040;
coefficient_rom[661]=32'sd0;
coefficient_rom[662]=32'sd0;
coefficient_rom[663]=32'sd5878;
coefficient_rom[664]=32'sd0;
coefficient_rom[665]=-32'sd25403164;
coefficient_rom[666]=32'sd0;
coefficient_rom[667]=32'sd0;
coefficient_rom[668]=-32'sd920498;
coefficient_rom[669]=32'sd0;
coefficient_rom[670]=32'sd0;
coefficient_rom[671]=32'sd5012;
coefficient_rom[672]=32'sd0;
coefficient_rom[673]=32'sd0;
coefficient_rom[674]=32'sd476306;
coefficient_rom[675]=32'sd0;
coefficient_rom[676]=32'sd0;
coefficient_rom[677]=32'sd2857938;
coefficient_rom[678]=32'sd0;
coefficient_rom[679]=32'sd0;
coefficient_rom[680]=32'sd1961;
coefficient_rom[681]=32'sd0;
coefficient_rom[682]=32'sd0;
coefficient_rom[683]=32'sd13023;
coefficient_rom[684]=32'sd0;
coefficient_rom[685]=32'sd0;
coefficient_rom[686]=32'sd8460885;
coefficient_rom[687]=32'sd0;
coefficient_rom[688]=32'sd0;
coefficient_rom[689]=-32'sd130;
coefficient_rom[690]=32'sd0;
coefficient_rom[691]=32'sd0;
coefficient_rom[692]=32'sd26099228;
coefficient_rom[693]=32'sd0;
coefficient_rom[694]=32'sd0;
coefficient_rom[695]=-32'sd5012;
coefficient_rom[696]=32'sd0;
coefficient_rom[697]=32'sd0;
coefficient_rom[698]=-32'sd385769;
coefficient_rom[699]=32'sd0;
coefficient_rom[700]=-32'sd2054030;
coefficient_rom[701]=32'sd0;
coefficient_rom[702]=32'sd0;
coefficient_rom[703]=32'sd786677;
coefficient_rom[704]=32'sd0;
coefficient_rom[705]=32'sd0;
coefficient_rom[706]=32'sd99401;
coefficient_rom[707]=32'sd0;
coefficient_rom[708]=32'sd0;
coefficient_rom[709]=-32'sd569297;
coefficient_rom[710]=32'sd0;
coefficient_rom[711]=32'sd0;
coefficient_rom[712]=32'sd2336525;
coefficient_rom[713]=32'sd0;
coefficient_rom[714]=32'sd0;
coefficient_rom[715]=32'sd11974284;
coefficient_rom[716]=32'sd0;
coefficient_rom[717]=32'sd0;
coefficient_rom[718]=-32'sd6886;
coefficient_rom[719]=32'sd0;
coefficient_rom[720]=32'sd0;
coefficient_rom[721]=32'sd1919669;
coefficient_rom[722]=32'sd0;
coefficient_rom[723]=32'sd0;
coefficient_rom[724]=-32'sd125392;
coefficient_rom[725]=32'sd0;
coefficient_rom[726]=32'sd0;
coefficient_rom[727]=32'sd1215306;
coefficient_rom[728]=32'sd0;
coefficient_rom[729]=32'sd0;
coefficient_rom[730]=-32'sd99401;
coefficient_rom[731]=32'sd0;
coefficient_rom[732]=32'sd0;
coefficient_rom[733]=32'sd461937;
coefficient_rom[734]=32'sd0;
coefficient_rom[735]=-32'sd1384029;
coefficient_rom[736]=32'sd0;
coefficient_rom[737]=32'sd0;
coefficient_rom[738]=32'sd87381;
coefficient_rom[739]=32'sd0;
coefficient_rom[740]=32'sd0;
coefficient_rom[741]=32'sd97028;
coefficient_rom[742]=32'sd0;
coefficient_rom[743]=32'sd0;
coefficient_rom[744]=-32'sd69994;
coefficient_rom[745]=32'sd0;
coefficient_rom[746]=32'sd0;
coefficient_rom[747]=32'sd435582;
coefficient_rom[748]=32'sd0;
coefficient_rom[749]=32'sd0;
coefficient_rom[750]=32'sd4037998;
coefficient_rom[751]=32'sd0;
coefficient_rom[752]=32'sd0;
coefficient_rom[753]=32'sd11974831;
coefficient_rom[754]=32'sd0;
coefficient_rom[755]=32'sd0;
coefficient_rom[756]=32'sd117084;
coefficient_rom[757]=32'sd0;
coefficient_rom[758]=32'sd0;
coefficient_rom[759]=-32'sd19965;
coefficient_rom[760]=32'sd0;
coefficient_rom[761]=32'sd0;
coefficient_rom[762]=32'sd1279261;
coefficient_rom[763]=32'sd0;
coefficient_rom[764]=32'sd0;
coefficient_rom[765]=-32'sd97028;
coefficient_rom[766]=32'sd0;
coefficient_rom[767]=32'sd0;
coefficient_rom[768]=32'sd57482;
coefficient_rom[769]=32'sd0;
coefficient_rom[770]=32'sd0;
coefficient_rom[771]=32'sd11974831;
coefficient_rom[772]=32'sd0;
coefficient_rom[773]=32'sd0;
coefficient_rom[774]=32'sd117084;
coefficient_rom[775]=32'sd0;
coefficient_rom[776]=32'sd0;
coefficient_rom[777]=-32'sd19965;
coefficient_rom[778]=32'sd0;
coefficient_rom[779]=32'sd0;
coefficient_rom[780]=32'sd1279261;
coefficient_rom[781]=32'sd0;
coefficient_rom[782]=32'sd0;
coefficient_rom[783]=-32'sd97028;
coefficient_rom[784]=32'sd0;
coefficient_rom[785]=32'sd0;
coefficient_rom[786]=32'sd57482;
coefficient_rom[787]=32'sd0;
coefficient_rom[788]=32'sd0;
coefficient_rom[789]=32'sd0;
coefficient_rom[790]=32'sd0;
coefficient_rom[791]=32'sd4;
coefficient_rom[792]=32'sd0;
coefficient_rom[793]=32'sd0;
coefficient_rom[794]=-32'sd4;
coefficient_rom[795]=32'sd0;
coefficient_rom[796]=32'sd0;
coefficient_rom[797]=-32'sd419430;
coefficient_rom[798]=32'sd0;
coefficient_rom[799]=32'sd0;
coefficient_rom[800]=-32'sd4192;
coefficient_rom[801]=32'sd0;
coefficient_rom[802]=32'sd0;
coefficient_rom[803]=-32'sd13;
coefficient_rom[804]=32'sd0;
coefficient_rom[805]=32'sd0;
coefficient_rom[806]=32'sd0;
coefficient_rom[807]=32'sd0;
coefficient_rom[808]=32'sd0;
coefficient_rom[809]=32'sd0;
coefficient_rom[810]=32'sd0;
coefficient_rom[811]=32'sd0;
coefficient_rom[812]=32'sd16777214;
coefficient_rom[813]=32'sd0;
coefficient_rom[814]=32'sd0;
coefficient_rom[815]=-32'sd6;
coefficient_rom[816]=32'sd0;
coefficient_rom[817]=32'sd0;
coefficient_rom[818]=32'sd1;
coefficient_rom[819]=32'sd0;
coefficient_rom[820]=32'sd0;
coefficient_rom[821]=32'sd0;
coefficient_rom[822]=32'sd0;
coefficient_rom[823]=32'sd0;
coefficient_rom[824]=32'sd419430;
coefficient_rom[825]=32'sd0;
coefficient_rom[826]=32'sd0;
coefficient_rom[827]=32'sd0;
coefficient_rom[828]=32'sd0;
coefficient_rom[829]=32'sd0;
coefficient_rom[830]=32'sd0;
coefficient_rom[831]=32'sd0;
coefficient_rom[832]=32'sd0;
coefficient_rom[833]=32'sd0;
coefficient_rom[834]=32'sd0;
coefficient_rom[835]=32'sd0;
coefficient_rom[836]=32'sd16760;
coefficient_rom[837]=32'sd0;
coefficient_rom[838]=32'sd0;
coefficient_rom[839]=32'sd4194;
coefficient_rom[840]=32'sd0;
coefficient_rom[841]=32'sd0;
coefficient_rom[842]=32'sd0;
coefficient_rom[843]=32'sd0;
coefficient_rom[844]=32'sd0;
coefficient_rom[845]=32'sd0;
coefficient_rom[846]=32'sd0;
coefficient_rom[847]=32'sd117243313;
coefficient_rom[848]=32'sd0;
coefficient_rom[849]=32'sd0;
coefficient_rom[850]=32'sd11515862;
coefficient_rom[851]=32'sd0;
coefficient_rom[852]=32'sd0;
coefficient_rom[853]=32'sd91736295;
coefficient_rom[854]=32'sd0;
coefficient_rom[855]=32'sd0;
coefficient_rom[856]=-32'sd747125;
coefficient_rom[857]=32'sd0;
coefficient_rom[858]=32'sd0;
coefficient_rom[859]=-32'sd5912648;
coefficient_rom[860]=32'sd0;
coefficient_rom[861]=32'sd0;
coefficient_rom[862]=-32'sd7009661;
coefficient_rom[863]=32'sd0;
coefficient_rom[864]=32'sd0;
coefficient_rom[865]=32'sd475651;
coefficient_rom[866]=32'sd0;
coefficient_rom[867]=32'sd0;
coefficient_rom[868]=32'sd16777216;
coefficient_rom[869]=32'sd0;
coefficient_rom[870]=32'sd0;
coefficient_rom[871]=-32'sd8388608;
coefficient_rom[872]=32'sd0;
coefficient_rom[873]=32'sd0;
coefficient_rom[874]=-32'sd2069841;
coefficient_rom[875]=32'sd0;
coefficient_rom[876]=32'sd0;
coefficient_rom[877]=32'sd0;
coefficient_rom[884]=32'sd0;
coefficient_rom[885]=32'sd0;
coefficient_rom[886]=32'sd0;
coefficient_rom[887]=32'sd0;
coefficient_rom[888]=32'sd0;
coefficient_rom[889]=32'sd0;
coefficient_rom[890]=32'sd0;
coefficient_rom[891]=32'sd0;
coefficient_rom[892]=32'sd0;
coefficient_rom[893]=32'sd16777;
coefficient_rom[894]=32'sd0;
coefficient_rom[895]=32'sd0;
coefficient_rom[896]=32'sd0;
coefficient_rom[897]=32'sd0;
coefficient_rom[898]=32'sd0;
coefficient_rom[899]=32'sd0;
coefficient_rom[900]=32'sd0;
coefficient_rom[901]=32'sd0;
coefficient_rom[902]=32'sd0;
coefficient_rom[903]=32'sd0;
coefficient_rom[904]=32'sd0;
coefficient_rom[899]=32'sd0;
coefficient_rom[900]=32'sd0;
coefficient_rom[901]=32'sd0;
coefficient_rom[902]=32'sd0;
coefficient_rom[903]=32'sd0;
coefficient_rom[904]=32'sd0;
coefficient_rom[905]=32'sd0;
coefficient_rom[906]=32'sd0;
coefficient_rom[907]=32'sd0;
coefficient_rom[908]=32'sd0;
coefficient_rom[909]=32'sd0;
coefficient_rom[910]=32'sd0;
coefficient_rom[911]=32'sd0;
coefficient_rom[912]=32'sd0;
coefficient_rom[913]=32'sd0;
coefficient_rom[914]=32'sd0;
coefficient_rom[915]=32'sd0;
coefficient_rom[916]=32'sd0;
coefficient_rom[917]=32'sd0;
coefficient_rom[918]=32'sd0;
coefficient_rom[919]=32'sd0;
coefficient_rom[920]=32'sd0;
coefficient_rom[921]=32'sd0;
coefficient_rom[922]=32'sd0;
coefficient_rom[923]=32'sd0;
coefficient_rom[924]=32'sd0;
coefficient_rom[925]=32'sd0;
coefficient_rom[926]=32'sd0;
coefficient_rom[927]=32'sd0;
coefficient_rom[928]=32'sd0;
coefficient_rom[929]=32'sd0;
coefficient_rom[930]=32'sd0;
coefficient_rom[931]=32'sd0;
coefficient_rom[932]=32'sd0;
coefficient_rom[933]=32'sd0;
coefficient_rom[934]=32'sd0;
coefficient_rom[935]=32'sd0;
coefficient_rom[936]=32'sd0;
coefficient_rom[937]=32'sd0;
coefficient_rom[938]=32'sd0;
coefficient_rom[939]=32'sd0;
coefficient_rom[940]=32'sd0;
coefficient_rom[941]=32'sd0;
coefficient_rom[942]=32'sd4;
coefficient_rom[943]=32'sd0;
coefficient_rom[944]=32'sd0;
coefficient_rom[945]=-32'sd4;
coefficient_rom[946]=32'sd0;
coefficient_rom[947]=32'sd0;
coefficient_rom[948]=-32'sd419430;
coefficient_rom[949]=32'sd0;
coefficient_rom[950]=32'sd0;
coefficient_rom[951]=-32'sd4192;
coefficient_rom[952]=32'sd0;
coefficient_rom[953]=32'sd0;
coefficient_rom[954]=-32'sd13;
coefficient_rom[955]=32'sd0;
coefficient_rom[956]=32'sd0;
coefficient_rom[957]=32'sd0;
coefficient_rom[958]=32'sd0;
coefficient_rom[959]=32'sd0;
coefficient_rom[960]=32'sd0;
coefficient_rom[961]=32'sd0;
coefficient_rom[962]=32'sd0;
coefficient_rom[963]=32'sd16777214;
coefficient_rom[964]=32'sd0;
coefficient_rom[965]=32'sd0;
coefficient_rom[966]=-32'sd6;
coefficient_rom[967]=32'sd0;
coefficient_rom[968]=32'sd0;
coefficient_rom[969]=32'sd1;
coefficient_rom[970]=32'sd0;
coefficient_rom[971]=32'sd0;
coefficient_rom[972]=32'sd0;
coefficient_rom[973]=32'sd0;
coefficient_rom[974]=32'sd0;
coefficient_rom[975]=32'sd419430;
coefficient_rom[976]=32'sd0;
coefficient_rom[977]=32'sd0;
coefficient_rom[978]=32'sd0;
coefficient_rom[979]=32'sd0;
coefficient_rom[980]=32'sd0;
coefficient_rom[981]=32'sd0;
coefficient_rom[982]=32'sd0;
coefficient_rom[983]=32'sd0;
coefficient_rom[984]=32'sd0;
coefficient_rom[985]=32'sd0;
coefficient_rom[986]=32'sd0;
coefficient_rom[987]=32'sd0;
coefficient_rom[988]=32'sd0;
coefficient_rom[989]=32'sd0;
coefficient_rom[990]=32'sd0;
coefficient_rom[991]=32'sd0;
coefficient_rom[992]=32'sd0;
coefficient_rom[993]=32'sd0;
coefficient_rom[994]=32'sd0;
coefficient_rom[995]=32'sd0;
coefficient_rom[996]=32'sd0;
coefficient_rom[997]=32'sd0;
coefficient_rom[998]=32'sd0;
coefficient_rom[999]=32'sd0;
coefficient_rom[1000]=32'sd0;
coefficient_rom[1001]=32'sd0;
coefficient_rom[1002]=32'sd0;
coefficient_rom[1003]=32'sd0;
coefficient_rom[1004]=32'sd0;
coefficient_rom[1005]=32'sd0;
coefficient_rom[1006]=32'sd0;
coefficient_rom[1007]=32'sd0;
coefficient_rom[1008]=32'sd0;
coefficient_rom[1009]=32'sd0;
coefficient_rom[1010]=32'sd0;
coefficient_rom[1011]=32'sd0;
coefficient_rom[1012]=32'sd0;
coefficient_rom[1013]=32'sd0;
coefficient_rom[1014]=32'sd0;
coefficient_rom[1015]=32'sd0;
coefficient_rom[1016]=32'sd0;
coefficient_rom[1017]=32'sd0;
coefficient_rom[1018]=32'sd0;
coefficient_rom[1019]=32'sd0;
coefficient_rom[1020]=32'sd0;
coefficient_rom[1021]=32'sd0;
coefficient_rom[1022]=32'sd0;
coefficient_rom[1023]=32'sd0;

































































































































































































































































































































































































































































































































end

always @(posedge clk) coefficient_word<=coefficient_rom[{r39_mode,state}];

reg signed[31:0] r35_stasis_error;
reg[7:0] r35_stasis_samples;
reg r41t1_reference_changed;
always @(posedge clk)
 r41t1_reference_changed <= reference_s!=(reference_offset<<<8);
// R36 timing candidate: retain the 32-bit Q24 subtraction exactly and
// schedule its register in an existing cycle, before any n-state is consumed.
reg signed[31:0] r36_phi_error_s;
reg r36_platform_clamp_s;
wire signed[31:0] r36_feedback_velocity=feedback_velocity;
wire r35_static_i_eligible=automatic_s && mature_static_s && !brake_s && !motion_latched && velocity_s==0 && r39t5_data_cmp14 && r39t5_data_cmp15 && r36_phi_error_s<=32'sd1464064 && r36_phi_error_s>= -32'sd1464064 && r39t5_data_cmp16 && r39t5_data_cmp17 && r39t5_data_cmp18 && r39t5_data_cmp19;
// Reuse the existing same-error counter, but never freeze explicit remaining
// error>1 legacy degree. Large-error stasis only accelerates I; it does not
// expand the real-platform arm-speed/friction clamp or alter any observer state.
wire r38_endpoint_stasis_eligible=r38_explicit_endpoint_s &&
 (r36_phi_error_s>32'sd292864 || r36_phi_error_s< -32'sd292864) &&
 r39t5_data_cmp20 && r39t5_data_cmp21 &&
 r39t5_data_cmp16 && r39t5_data_cmp17 && r39t5_data_cmp18 && r39t5_data_cmp19;
reg r40_unwind_active;
// R41: all operands are transaction snapshots. Correct an actual lead
// during explicit ordinary motion before the reference reaches its end.
// Reuse the 1048 integral MAC; current/automatic/brake/A exclusion are
// already latched in r40_position_unwind_s. No new pipeline stage.
wire r41_moving_lead_i=r40_position_unwind_s &&
 ((velocity_s>32'sd64 && r36_phi_error_s>0 && integral_lp>0) ||
  (velocity_s< -32'sd64 && r36_phi_error_s<0 && integral_lp<0));
wire r40_unwind_opposed=(integral_lp>0 && integral<0)||(integral_lp<0 && integral>0);
wire r40_unwind_trigger=r40_position_unwind_s && r40_unwind_opposed &&
 (integral_lp>32'sd1218492 || integral_lp< -32'sd1218492) &&
 (integral>=32'sd838861 || integral<= -32'sd838861);
wire r40_position_unwind=r40_position_unwind_s && r40_unwind_opposed && (r40_unwind_active || r40_unwind_trigger);
wire r40_unwind_zero=(integral>0 && integral+integral_step<=0)||(integral<0 && integral+integral_step>=0);
wire signed[31:0] r40_unwind_step=product>>>24;
wire r40_integral_update_allowed=age>=1000 && (!r35_static_i_unwind_only || (integral>0 && integral_step<0) || (integral<0 && integral_step>0)) && r39t5_data_cmp46 && r39t5_data_cmp47 && ((r39t5_data_cmp16 && r39t5_data_cmp17 && r39t5_data_cmp18 && r39t5_data_cmp19) || r41_moving_lead_i || r40_position_unwind || ((!automatic_s && manual_moved && velocity_s==0 && r39t5_data_cmp46 && r39t5_data_cmp47) && ((integral_step>0 && integral<0) || (integral_step<0 && integral>0)))) && ((r39t5_data_cmp48 && r39t5_data_cmp49) || (integral_step>0 && output_error<0) || (integral_step<0 && output_error>0));
always @(posedge clk or negedge rst_n) begin
 if(!rst_n)r40_unwind_active<=0;
 else if(clear || fault)r40_unwind_active<=0;
 else if(start&&!busy)begin
  if(!automatic_policy || automatic_brake || !motion_assist_permission || !(measurement_current===1'b1) || (ext2_warm===1'b1))r40_unwind_active<=0;
 end else if(busy&&!r39_mode&&state==9'd316)begin
  if(!r40_position_unwind_s || integral==0 || (r40_integral_update_allowed && r40_position_unwind && r40_unwind_zero))r40_unwind_active<=0;
  else if(r40_unwind_trigger)r40_unwind_active<=1;
 end
end

// R41: retain explicit-endpoint validity, quiet state, >1degree error and
// ordinary mode. Encoder dithering no longer demands201 exactly equal samples.
// Existing4194 coefficient, anti-windup, theta/current guards and bounds remain.
wire r38_endpoint_i_fast=r38_endpoint_stasis_eligible;
// samples1 is the first observation. Old count200 plus this unchanged
// observation proves200 completed1ms intervals, so no199ms early freeze.
wire r35_static_i_unwind_only=r35_static_i_eligible && !r38_endpoint_stasis_eligible && r35_stasis_samples>=8'd200 && r35_stasis_error==r36_phi_error_s;
// R36 candidate: mean arm motion on a real same-encoder plateau is bounded.
// Restrict only feedback/friction; retain all seven observer state updates.
// Q24 normalized v/10: 50332 corresponds to approximately0.03legacy rad/s.
// n1/n3 are complete before state255. The predicate and selected velocity
// occupy existing cycles255/256, leaving state257 as one abs operation.
// The same selected value remains live until feedback multiply state282.


always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin r40_transfer_ticks<=0;r39_mode<=0;r39_matrix_added<=0;r39_matrix_return<=0;motion_s<=0;z7<=0;n7<=0;bias_clipped<=0;r39t1_scratch33_a<=0;encoder_velocity_correction<=0;feedback_velocity<=0;boost_clipped<=0;boosted_position<=0;feedback_lp<=0;feedback_delta<=0;shape_source<=0;shape_fast<=0;shape_slow<=0;shape_fast_next<=0;shape_slow_next<=0;shape_correction<=0;r39t1_scratch33_b<=0;shape_valid<=0;sine_s<=0;sine_valid<=0;sine_reference_s<=0;sine_bias<=0;sine_bias_next<=0;sine_position_request<=0;sine_position_committed<=0;sine_reference_committed<=0;smooth_position<=0;smooth_velocity_step<=0;smooth_position_next<=0;smooth_velocity_step_next<=0;smooth_valid<=0;smooth_sine_active_committed<=0;smooth_normalized_next<=0;smooth_previous_next<=0;smooth_correction_next<=0;smooth_bound_next<=0;smooth_residual<=0;smooth_error<=0;smooth_normalized_wide<=0;smooth_normalized_committed<=0;smooth_previous_committed<=0;smooth_correction_committed<=0;smooth_bound_committed<=0;smooth_innovation_committed<=0;smooth_dividend<=0;smooth_quotient<=0;smooth_seed_rounded<=0;smooth_remainder<=0;smooth_divide_bits<=0;smooth_seed_negative<=0;r36_phi_error_s<=0;r36_platform_clamp_s<=0;r35_stasis_error<=0;r35_stasis_samples<=0;slew_low<=0;slew_high<=0;prediction_previous<=0;output_slew<=20;manual_moved<=0;motion_watch<=0;motion_bad_ticks<=0;motion_latched<=0;motion_phase<=0;motion_ticks<=0;automatic_s<=0;mature_static_s<=0;hold_s<=0;brake_s<=0;drift_s<=0;blend<=0;fallback_s<=0;integral<=0;integral_lp<=0;age<=0;busy<=0;done<=0;fault<=0;ready<=0;duty<=0;state<=0;samples<=0;z0<=0;z1<=0;z2<=0;z3<=0;z4<=0;z5<=0;z6<=0; end
 else if(clear)begin r40_transfer_ticks<=0;r39_mode<=0;r39_matrix_added<=0;r39_matrix_return<=0;motion_s<=0;z7<=0;n7<=0;bias_clipped<=0;r39t1_scratch33_a<=0;encoder_velocity_correction<=0;feedback_velocity<=0;boost_clipped<=0;boosted_position<=0;feedback_lp<=0;feedback_delta<=0;shape_source<=0;shape_fast<=0;shape_slow<=0;shape_fast_next<=0;shape_slow_next<=0;shape_correction<=0;r39t1_scratch33_b<=0;shape_valid<=0;sine_s<=0;sine_valid<=0;sine_reference_s<=0;sine_bias<=0;sine_bias_next<=0;sine_position_request<=0;sine_position_committed<=0;sine_reference_committed<=0;smooth_position<=0;smooth_velocity_step<=0;smooth_position_next<=0;smooth_velocity_step_next<=0;smooth_valid<=0;smooth_sine_active_committed<=0;smooth_normalized_next<=0;smooth_previous_next<=0;smooth_correction_next<=0;smooth_bound_next<=0;smooth_residual<=0;smooth_error<=0;smooth_normalized_wide<=0;smooth_normalized_committed<=0;smooth_previous_committed<=0;smooth_correction_committed<=0;smooth_bound_committed<=0;smooth_innovation_committed<=0;smooth_dividend<=0;smooth_quotient<=0;smooth_seed_rounded<=0;smooth_remainder<=0;smooth_divide_bits<=0;smooth_seed_negative<=0;r36_phi_error_s<=0;r36_platform_clamp_s<=0;r35_stasis_error<=0;r35_stasis_samples<=0;slew_low<=0;slew_high<=0;prediction_previous<=0;output_slew<=20;manual_moved<=0;motion_watch<=0;motion_bad_ticks<=0;motion_latched<=0;motion_phase<=0;motion_ticks<=0;automatic_s<=0;mature_static_s<=0;hold_s<=0;brake_s<=0;drift_s<=0;blend<=0;fallback_s<=0;integral<=0;integral_lp<=0;age<=0;busy<=0;done<=0;fault<=0;ready<=0;duty<=0;state<=0;samples<=0;z0<=0;z1<=0;z2<=0;z3<=0;z4<=0;z5<=0;z6<=0; end
 else begin
 done<=0;
 if(!busy)begin
  if(start && !fault)begin
   // reference_s retains the previous completed transaction inQ24.
   // Reuse this existing latch to break stasis even for equal-error co-motion.
   // R41T1: exact old/new-reference decision retires on the first busy clock.
   // R22R2: snapshot first; range decision never drives the wide input CE.
   if(!automatic_policy && (reference_velocity>32'sd64 || reference_velocity< -32'sd64))manual_moved<=1;
   if(automatic_policy && !automatic_brake && motion_assist_permission && (reference_velocity>32'sd64 || reference_velocity< -32'sd64))motion_watch<=1;
   if(automatic_policy && !automatic_brake && motion_assist_permission && (theta>32'sd1144 || theta< -32'sd1144))begin
    if(motion_bad_ticks<20)motion_bad_ticks<=motion_bad_ticks+1'b1;
    if(motion_bad_ticks>=19)motion_latched<=1;
   end else motion_bad_ticks<=0;
   automatic_s<=automatic_policy && !(ext2_warm===1'b1);mature_static_s<=(mature_static_hold===1'b1) && automatic_policy && !automatic_brake && reference_velocity==0;hold_s<=automatic_hold;brake_s<=automatic_brake;fallback_s<=fallback_duty;
   theta_s<=theta;phi_s<=phi;reference_s<=reference_offset;
   velocity_s<=reference_velocity<<<8;previous<=previous_duty;prediction_previous<=automatic_policy?prediction_duty:previous_duty;cap<=limit_permille;
   // Same measured state/origin crosses profiles; only A-only memories are seeded.
   if((ext2_warm===1'b1)!=r39_mode)begin
    r40_transfer_ticks<=8'd128;
    if(ext2_warm===1'b1)begin z7<=0;shape_valid<=0;smooth_valid<=0;sine_valid<=0;feedback_lp<=0;end
   end else if(r40_transfer_ticks!=0)r40_transfer_ticks<=r40_transfer_ticks-1'b1;
   r39_mode<=ext2_warm===1'b1;motion_s<=ext2_motion===1'b1;sine_s<=ext2_enable===1'b1;sine_reference_s<=ext2_reference;
   busy<=1;state<=(ext2_warm===1'b1)?9'd391:9'd323;

  end
 end else begin
  state<=state+1;
  if(((state==9'd323 && !r39_mode) || (state==9'd391 && r39_mode)) &&
     r41t1_reference_changed)r35_stasis_samples<=0;
  // R41T1: parallel disjoint mode/state decode; no state>=316
  // cascade feeding the A-only write muxes. State numbers/timing unchanged.
  case({r39_mode,state})
10'd828:begin sine_bias_next<=!sine_ff_enabled ? 32'sd0 : !sine_valid ? sine_reference_s : (r39t5_data_cmp22 && r39t5_data_cmp23) ? 32'sd0 : sine_bias-(sine_bias>>>5); r39t1_scratch33_a <= $signed({phi_s[31],phi_s})-$signed({n2[31],n2});   end
10'd829:begin r39t1_scratch33_a<=$signed({sine_reference_s[31],sine_reference_s})-$signed({sine_bias_next[31],sine_bias_next}); encoder_velocity_correction <= (MANUAL_FILTERED_MOTION && MANUAL_ENCODER_FEEDBACK && MANUAL_LOW_GAIN && !1'b0 && motion_s) ? (r39t5_data_cmp24 ? 32'sd335544 : r39t5_data_cmp25 ? -32'sd335544 : ($signed(encoder_residual[31:0])<<<1)) : (r39t5_data_cmp26 ? 32'sd335544 : r39t5_data_cmp27 ? -32'sd335544 : $signed(encoder_residual[31:0]));  end
10'd830:begin sine_position_request<=sine_ff_enabled ? sat($signed({{31{sine_delta[32]}},sine_delta})>>>3) : 32'sd0; feedback_delta <= encoder_velocity_correction-feedback_lp; feedback_velocity <= r39t1_add_sat;  end
10'd831:begin feedback_lp <= !(MANUAL_FILTERED_MOTION && MANUAL_ENCODER_FEEDBACK && MANUAL_LOW_GAIN && !1'b0) ? 32'sd0 : motion_s ? feedback_lp+(feedback_delta>>>2) : encoder_velocity_correction;   end
10'd832:begin if(MANUAL_FILTERED_MOTION && MANUAL_ENCODER_FEEDBACK && MANUAL_LOW_GAIN && !1'b0) feedback_velocity<=r39t1_add_sat; /* synchronous table read occurs in independent clocked ROM port */ end
10'd833:begin  end
10'd834:begin if(sine_ff_enabled)sine_position_request<=r39t1_add_sat; end
10'd835:begin position_error <= n2-reference_s; end
10'd836:begin boost_clipped <= r39t5_data_cmp28 ? 32'sd5856353 : r39t5_data_cmp29 ? -32'sd5856353 : position_error;    end
10'd837:begin if(!multiply_wait)state<=state; end
10'd838:begin r39t1_scratch33_a <= $signed({position_error[31],position_error})+($signed({boost_clipped[31],boost_clipped})>>>3); integral_lp <= sine_ff_enabled ? sat($signed({{32{integral_lp[31]}},integral_lp})+(product >>> 24)) : (motion_s && ((phi_s>=reference_s && integral_lp<0) || (phi_s<=reference_s && integral_lp>0))) ? 32'sd0 : (!(1'b0 && drift_s) && (r39t5_data_cmp30 || r39t5_data_cmp31 || r39t5_data_cmp32 || r39t5_data_cmp33)) ? ((1'b0 || r39t5_data_cmp34 || r39t5_data_cmp35) && r39t5_data_cmp36 && r39t5_data_cmp37 ? integral_lp:32'sd0) : sat($signed({{32{integral_lp[31]}},integral_lp})+(product >>> 24)); end
10'd839:begin boosted_position <= boost_wide>33'sd2147483647 ? 32'sh7fffffff : boost_wide< -33'sd2147483648 ? 32'sh80000000 : boost_wide[31:0];    end
10'd840:begin if(!multiply_wait)state<=state; end
10'd841:begin integral_step <= sat(product >>> 24);if(smooth_enabled)state<=9'd400;end
10'd842:begin theta_gain <= 1'b0 ? (brake_s ? 32'sd82569382 : 32'sd67937104) : MANUAL_LOW_GAIN ? (MANUAL_DAMPED_FEEDBACK ? 32'sd74306208 : 32'sd84921380) : (r39t5_data_cmp38 || r39t5_data_cmp39 || r39t5_data_cmp40 || r39t5_data_cmp41) ? 32'sd180666024 : 32'sd126466217; end
10'd843:begin  end
10'd844:begin    end
10'd845:begin if(!multiply_wait)state<=state; end
10'd846:begin  end
10'd847:begin    end
10'd848:begin if(!multiply_wait)state<=state; end
10'd849:begin  end
10'd850:begin    end
10'd851:begin if(!multiply_wait)state<=state; end
10'd852:begin  end
10'd853:begin    end
10'd854:begin if(!multiply_wait)state<=state; end
10'd855:begin  end
10'd856:begin    end
10'd857:begin if(!multiply_wait)state<=state; end
10'd858:begin  end
10'd859:begin    end
10'd860:begin if(!multiply_wait)state<=state; end
10'd861:begin  end
10'd862:begin    end
10'd863:begin if(!multiply_wait)state<=state; end
10'd864:begin  end
10'd865:begin shape_source<=shape_enabled ? sat(acc) : 32'sd0;   end
10'd866:begin if(!multiply_wait)state<=state; end
10'd867:begin  end
10'd868:begin   end
10'd869:begin if(!multiply_wait)state<=state; end
10'd870:begin  end
10'd871:begin    end
10'd872:begin if(!multiply_wait)state<=state; end
10'd873:begin  end
10'd874:begin   end
10'd875:begin if(!multiply_wait)state<=state; end
10'd876:begin  end
10'd877:begin request <= sat(acc); state<=shape_enabled ? 9'd366 : sine_ff_enabled ? 9'd371 : 9'd372;
 if(!shape_enabled)begin shape_fast<=0;shape_slow<=0;shape_valid<=0;shape_correction<=0;end end
10'd878:begin r39t1_scratch33_b<=$signed({shape_source[31],shape_source})-$signed({shape_fast[31],shape_fast});r39t1_scratch33_a<=$signed({shape_source[31],shape_source})-$signed({shape_slow[31],shape_slow});end
10'd879:begin
 // Convex IIR updates stay signed32, even for the full signed32 source range.
 shape_fast_next<=shape_valid ? $signed({shape_fast[31],shape_fast})+(shape_fast_delta>>>4) : shape_source;
 shape_slow_next<=shape_valid ? $signed({shape_slow[31],shape_slow})+(shape_slow_delta>>>6) : shape_source;end
10'd880:begin r39t1_scratch33_b<=$signed({shape_fast_next[31],shape_fast_next})-$signed({shape_slow_next[31],shape_slow_next});end
10'd881:begin
 if(sine_damping_enabled)shape_correction<=r39t5_data_cmp42 ? 32'sd671088 : r39t5_data_cmp43 ? -32'sd671088 : shape_band[31:0];
 else shape_correction<=r39t5_data_cmp44 ? 32'sd671088 : r39t5_data_cmp45 ? -32'sd671088 : (shape_band>>>2);
end
10'd882:begin request<=r39t1_add_sat;end
10'd883:begin if(sine_ff_enabled)request<=r39t1_add_sat;end
10'd884:begin    end
10'd885:begin if(!multiply_wait)state<=state; end
10'd886:begin request <= sat((product >>> 24) + (request == 0 ? 64'sd0 : request < 0 ? -64'sd298096 : 64'sd298096)) ; end
10'd887:begin slew_low<=previous-32'sd20;slew_high<=previous+32'sd20;conversion <= (($signed({{32{request[31]}},request})<<<10)-($signed({{32{request[31]}},request})<<<4)-($signed({{32{request[31]}},request})<<<3)); end
// R41T2: intersect slew bounds with the current cap on the existing rounding clock.
// A shrinking hard limit takes priority, exactly as in the ordinary branch.
10'd888:begin slew_low<=slew_low>cap?cap:slew_low< -cap?-cap:slew_low;slew_high<=slew_high>cap?cap:slew_high< -cap?-cap:slew_high;magnitude <= conversion < 0 ? -conversion : conversion; negative <= conversion < 0; end
10'd889:begin rounded <= magnitude[55:24]+{31'd0,((magnitude[23:0]>24'd8388608)||((magnitude[23:0]==24'd8388608)&&magnitude[24]))}; end
10'd890:begin wanted <= negative ? -$signed(rounded) : $signed(rounded); end
10'd891:begin bounded <= wanted > cap ? cap : wanted < -cap ? -cap : wanted; end
10'd892:begin delivered <= bounded > slew_high ? slew_high : bounded < slew_low ? slew_low : bounded; end
10'd893:begin   end
10'd894:begin if(!multiply_wait)state<=state; end
10'd895:begin /* scratch output_error written in independent datapath */ end
10'd896:begin if((sine_ff_enabled || !(MOTION_I_HOLD && motion_s)) && age>=1000 && r39t5_data_cmp46 && r39t5_data_cmp47 && (sine_ff_enabled || (r39t5_data_cmp16 && r39t5_data_cmp17 && r39t5_data_cmp18 && r39t5_data_cmp19)) && ((r39t5_data_cmp48 && r39t5_data_cmp49) || (integral_step>0 && output_error<0) || (integral_step<0 && output_error>0))) integral <= integral+integral_step>32'sd5872026 ? 32'sd5872026:integral+integral_step< -32'sd5872026 ? -32'sd5872026:integral+integral_step; if(age<1000)age<=age+1'b1; end
10'd897:begin
 smooth_position<=smooth_enabled?smooth_position_next:32'sd0;smooth_velocity_step<=smooth_enabled?smooth_velocity_step_next:32'sd0;smooth_valid<=smooth_enabled;
 smooth_sine_active_committed<=smooth_enabled&&sine_ff_enabled;
 smooth_normalized_committed<=smooth_enabled?smooth_normalized_next:32'sd0;smooth_previous_committed<=smooth_enabled?smooth_previous_next:32'sd0;
 smooth_correction_committed<=smooth_enabled?smooth_correction_next:32'sd0;smooth_bound_committed<=smooth_enabled?smooth_bound_next:32'sd0;smooth_innovation_committed<=smooth_enabled?sat(smooth_error_wide):32'sd0;
 sine_bias<=sine_bias_next;sine_valid<=sine_ff_enabled;sine_position_committed<=sine_ff_enabled?sine_position_request:32'sd0;sine_reference_committed<=sine_ff_enabled?sine_reference_s:32'sd0; if(shape_enabled)begin shape_fast<=shape_fast_next;shape_slow<=shape_slow_next;shape_valid<=1;end duty <= delivered; z0<=n0; z1<=n1; z2<=n2; z3<=n3; z4<=n4; z5<=n5; z6<=n6;z7<=n7; if(samples<200)samples<=samples+1; ready<=samples>=200;
 if(!1'b0)begin busy<=0;done<=1;state<=0;end
 else if(age>=200 && hold_s)blend<=blend>32'sd16693330 ? 32'sd16777216:blend+32'sd83886;
 else blend<=blend<32'sd1048576 ? 32'sd0:blend-32'sd1048576; end
10'd898:begin  end
10'd899:begin if(!multiply_wait)state<=state; end
10'd900:begin magnitude<=product<0?-product:product;negative<=product<0; end
10'd901:begin rounded<=magnitude[55:24]+{31'd0,((magnitude[23:0]>24'd8388608)||((magnitude[23:0]==24'd8388608)&&magnitude[24]))}; end
10'd902:begin duty<=fallback_s+(negative?-$signed(rounded):$signed(rounded));busy<=0;done<=1;state<=0; end
10'd903:begin drift_s<=drift_s || (1'b0 && (($signed({phi_s[31],phi_s})-$signed({reference_s[31],reference_s}))>33'sd5719 || ($signed({phi_s[31],phi_s})-$signed({reference_s[31],reference_s}))< -33'sd5719));
 if(risk_bad_A)begin fault<=1;duty<=0;done<=1;busy<=0;state<=0;end
 else begin theta_s<=theta_s<<<8;phi_s<=phi_s<<<8;reference_s<=reference_s<<<8;sine_reference_s<=sine_reference_s<<<8;state<=9'd392;end end
10'd904:begin prior_q<=r39t1_prior_q;state<=1; end
10'd912:begin
 smooth_previous_next<=feedback_velocity;smooth_correction_next<=0;smooth_error<=0;
 smooth_bound_next<=!sine_ff_enabled ? 32'd0 : MANUAL_SMOOTH_CONTINUITY ? 32'd167772 : (smooth_reference_abs>>6)>32'd167772 ? 32'd167772 : (smooth_reference_abs>>6);
 if(!smooth_valid)begin
  smooth_position_next<=phi_s;smooth_dividend<=feedback_velocity[31] ? (~feedback_velocity)+32'd1 : feedback_velocity;
  smooth_seed_negative<=feedback_velocity[31];smooth_quotient<=0;smooth_remainder<=0;smooth_divide_bits<=32;state<=9'd401;
 end else begin r39t1_scratch33_b<=$signed({smooth_position[31],smooth_position})+$signed({smooth_velocity_step[31],smooth_velocity_step});state<=9'd404;end
end
10'd913:begin
 smooth_dividend<=smooth_dividend<<1;
 if(smooth_trial_remainder>=8'd100)begin smooth_remainder<=smooth_trial_remainder-8'd100;smooth_quotient<=(smooth_quotient<<1)|32'd1;end
 else begin smooth_remainder<=smooth_trial_remainder[6:0];smooth_quotient<=smooth_quotient<<1;end
 smooth_divide_bits<=smooth_divide_bits-1'b1;state<=smooth_divide_bits==1 ? 9'd402 : 9'd401;
end
10'd914:begin smooth_seed_rounded<=smooth_quotient+((smooth_remainder>7'd50 || (smooth_remainder==7'd50 && smooth_quotient[0]))?32'd1:32'd0);end
10'd915:begin
 smooth_velocity_step_next<=smooth_seed_negative ? -$signed(smooth_seed_rounded) : $signed(smooth_seed_rounded);
 smooth_normalized_wide<=r39t1_scale100;
 state<=9'd409;
end
10'd916:begin smooth_residual<=$signed({{2{phi_s[31]}},phi_s})-$signed({smooth_prediction[32],smooth_prediction});end
10'd917:begin smooth_error<=r39t5_data_cmp50 ? smooth_residual-34'sd50688 : r39t5_data_cmp51 ? smooth_residual+34'sd50688 : 34'sd0;end
10'd918:begin boost_clipped<=smooth_round6((smooth_error_wide<<<4)-smooth_error_wide);encoder_velocity_correction<=smooth_round6(smooth_error_wide);end
10'd919:begin smooth_position_next<=sat($signed({{30{smooth_position_sum[33]}},smooth_position_sum}));smooth_velocity_step_next<=sat($signed({{31{smooth_step_sum[32]}},smooth_step_sum}));end
10'd920:begin smooth_normalized_wide<=r39t1_scale100;end
10'd921:begin smooth_normalized_next<=sat(smooth_normalized_wide);if(!smooth_valid)state<=9'd412;end
10'd922:begin r39t1_scratch33_a<=$signed({smooth_normalized_next[31],smooth_normalized_next})-$signed({smooth_previous_next[31],smooth_previous_next});end
10'd923:begin
 if(MANUAL_SMOOTH_CONTINUITY)state<=9'd413;
 if(!sine_ff_enabled)smooth_correction_next<=0;
 else if(smooth_half(smooth_velocity_delta_wide)>$signed({32'd0,smooth_bound_next}))smooth_correction_next<=smooth_bound_next;
 else if(smooth_half(smooth_velocity_delta_wide)< -$signed({32'd0,smooth_bound_next}))smooth_correction_next<=-smooth_bound_next;
 else smooth_correction_next<=smooth_half(smooth_velocity_delta_wide);
end
// A39: recursive local correction continuity; final PWM20permille/ms is unchanged.
10'd924:begin feedback_velocity<=r39t1_add_sat;state<=9'd330;end
10'd925:begin
 if(!sine_ff_enabled)smooth_correction_next<=0;
 else if($signed({smooth_correction_next[31],smooth_correction_next})>r41t1_smooth_high)
  smooth_correction_next<=r41t1_smooth_high;
 else if($signed({smooth_correction_next[31],smooth_correction_next})<r41t1_smooth_low)
  smooth_correction_next<=r41t1_smooth_low;
 state<=9'd412;
end
10'd926:begin  end
10'd927:begin if(!multiply_wait)state<=state; end
10'd928:begin end
10'd929:begin state<=r39_matrix_return;end
10'd942:begin   end
10'd943:begin if(!multiply_wait)state<=state; end
10'd944:begin  end
10'd945:begin   end
10'd946:begin if(!multiply_wait)state<=state; end
10'd947:begin  end
10'd948:begin   end
10'd949:begin if(!multiply_wait)state<=state; end
10'd950:begin  end
10'd951:begin   end
10'd952:begin if(!multiply_wait)state<=state; end
10'd953:begin  end
10'd954:begin   end
10'd955:begin if(!multiply_wait)state<=state; end
10'd956:begin  end
10'd957:begin   end
10'd958:begin if(!multiply_wait)state<=state; end
10'd959:begin  end
10'd960:begin   end
10'd961:begin if(!multiply_wait)state<=state; end
10'd962:begin  end
10'd963:begin   end
10'd964:begin if(!multiply_wait)state<=state; end
10'd965:begin  end
10'd966:begin   end
10'd967:begin if(!multiply_wait)state<=state; end
10'd968:begin  end
10'd969:begin   end
10'd970:begin if(!multiply_wait)state<=state; end
10'd971:begin  end
10'd972:begin   end
10'd973:begin if(!multiply_wait)state<=state; end
10'd974:begin  end
10'd975:begin   end
10'd976:begin if(!multiply_wait)state<=state; end
10'd977:begin  end
10'd978:begin n7 <= !(DISTURBANCE_OBSERVER && !1'b0) ? 32'sd0 : r39t5_data_cmp52 ? 32'sd5033165 : r39t5_data_cmp53 ? -32'sd5033165 : acc[31:0];bias_clipped <= DISTURBANCE_OBSERVER && !1'b0 && (r39t5_data_cmp52 || r39t5_data_cmp53); state<=316;end
10'd323:begin
 
 drift_s<=drift_s || (automatic_s && (($signed({phi_s[31],phi_s})-$signed({reference_s[31],reference_s}))>33'sd5719 || ($signed({phi_s[31],phi_s})-$signed({reference_s[31],reference_s}))< -33'sd5719));
 if(risk_bad_base)begin fault<=1;duty<=0;done<=1;busy<=0;state<=0;end
 else begin theta_s<=theta_s<<<8;phi_s<=phi_s<<<8;reference_s<=reference_s<<<8;state<=9'd324;end
end
10'd324:begin r36_phi_error_s<=phi_s-reference_s;prior_q<=r39t1_prior_q;state<=1;end
10'd1,10'd513:begin   end
10'd2,10'd514:begin  end
10'd3,10'd515:begin  end
10'd4,10'd516:begin   end
10'd5,10'd517:begin /* synchronous table read occurs in independent clocked ROM port */ end
10'd6,10'd518:begin  end
10'd7,10'd519:begin  end
10'd8,10'd520:begin effective <= prior_q >= 0 ? (prior_q > DEAD ? prior_q-DEAD : 0) : (prior_q < -DEAD ? prior_q+DEAD : 0); end
10'd9,10'd521:begin    end
10'd10,10'd522:begin if(!multiply_wait)state<=state; end
10'd11,10'd523:begin effective <= sat(product >>> 24); end
10'd12,10'd524:begin  end
10'd13,10'd525:begin   end
10'd14,10'd526:begin if(!multiply_wait)state<=state; end
10'd15,10'd527:begin  end
10'd16,10'd528:begin   end
10'd17,10'd529:begin if(!multiply_wait)state<=state; end
10'd18,10'd530:begin  end
10'd19,10'd531:begin   end
10'd20,10'd532:begin if(!multiply_wait)state<=state; end
10'd21,10'd533:begin  end
10'd22,10'd534:begin   end
10'd23,10'd535:begin if(!multiply_wait)state<=state; end
10'd24,10'd536:begin  end
10'd25,10'd537:begin   end
10'd26,10'd538:begin if(!multiply_wait)state<=state; end
10'd27,10'd539:begin  end
10'd28,10'd540:begin   end
10'd29,10'd541:begin if(!multiply_wait)state<=state; end
10'd30,10'd542:begin  end
10'd31,10'd543:begin   end
10'd32,10'd544:begin if(!multiply_wait)state<=state; end
10'd33,10'd545:begin  end
10'd34,10'd546:begin   end
10'd35,10'd547:begin if(!multiply_wait)state<=state; end
10'd36,10'd548:begin  end
10'd37,10'd549:begin   end
10'd38,10'd550:begin if(!multiply_wait)state<=state; end
10'd39,10'd551:begin  end
10'd40,10'd552:begin   end
10'd41,10'd553:begin if(!multiply_wait)state<=state; end
10'd42,10'd554:begin  end
10'd43,10'd555:begin   end
10'd44,10'd556:begin if(!multiply_wait)state<=state; end
10'd45,10'd557:begin  end
10'd46,10'd558:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd46;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd47,10'd559:begin  end
10'd48,10'd560:begin   end
10'd49,10'd561:begin if(!multiply_wait)state<=state; end
10'd50,10'd562:begin  end
10'd51,10'd563:begin   end
10'd52,10'd564:begin if(!multiply_wait)state<=state; end
10'd53,10'd565:begin  end
10'd54,10'd566:begin   end
10'd55,10'd567:begin if(!multiply_wait)state<=state; end
10'd56,10'd568:begin  end
10'd57,10'd569:begin   end
10'd58,10'd570:begin if(!multiply_wait)state<=state; end
10'd59,10'd571:begin  end
10'd60,10'd572:begin   end
10'd61,10'd573:begin if(!multiply_wait)state<=state; end
10'd62,10'd574:begin  end
10'd63,10'd575:begin   end
10'd64,10'd576:begin if(!multiply_wait)state<=state; end
10'd65,10'd577:begin  end
10'd66,10'd578:begin   end
10'd67,10'd579:begin if(!multiply_wait)state<=state; end
10'd68,10'd580:begin  end
10'd69,10'd581:begin   end
10'd70,10'd582:begin if(!multiply_wait)state<=state; end
10'd71,10'd583:begin  end
10'd72,10'd584:begin   end
10'd73,10'd585:begin if(!multiply_wait)state<=state; end
10'd74,10'd586:begin  end
10'd75,10'd587:begin   end
10'd76,10'd588:begin if(!multiply_wait)state<=state; end
10'd77,10'd589:begin  end
10'd78,10'd590:begin   end
10'd79,10'd591:begin if(!multiply_wait)state<=state; end
10'd80,10'd592:begin  end
10'd81,10'd593:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd81;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd82,10'd594:begin  end
10'd83,10'd595:begin   end
10'd84,10'd596:begin if(!multiply_wait)state<=state; end
10'd85,10'd597:begin  end
10'd86,10'd598:begin   end
10'd87,10'd599:begin if(!multiply_wait)state<=state; end
10'd88,10'd600:begin  end
10'd89,10'd601:begin   end
10'd90,10'd602:begin if(!multiply_wait)state<=state; end
10'd91,10'd603:begin  end
10'd92,10'd604:begin   end
10'd93,10'd605:begin if(!multiply_wait)state<=state; end
10'd94,10'd606:begin  end
10'd95,10'd607:begin   end
10'd96,10'd608:begin if(!multiply_wait)state<=state; end
10'd97,10'd609:begin  end
10'd98,10'd610:begin   end
10'd99,10'd611:begin if(!multiply_wait)state<=state; end
10'd100,10'd612:begin  end
10'd101,10'd613:begin   end
10'd102,10'd614:begin if(!multiply_wait)state<=state; end
10'd103,10'd615:begin  end
10'd104,10'd616:begin   end
10'd105,10'd617:begin if(!multiply_wait)state<=state; end
10'd106,10'd618:begin  end
10'd107,10'd619:begin   end
10'd108,10'd620:begin if(!multiply_wait)state<=state; end
10'd109,10'd621:begin  end
10'd110,10'd622:begin   end
10'd111,10'd623:begin if(!multiply_wait)state<=state; end
10'd112,10'd624:begin  end
10'd113,10'd625:begin   end
10'd114,10'd626:begin if(!multiply_wait)state<=state; end
10'd115,10'd627:begin  end
10'd116,10'd628:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd116;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd117,10'd629:begin  end
10'd118,10'd630:begin   end
10'd119,10'd631:begin if(!multiply_wait)state<=state; end
10'd120,10'd632:begin  end
10'd121,10'd633:begin   end
10'd122,10'd634:begin if(!multiply_wait)state<=state; end
10'd123,10'd635:begin  end
10'd124,10'd636:begin   end
10'd125,10'd637:begin if(!multiply_wait)state<=state; end
10'd126,10'd638:begin  end
10'd127,10'd639:begin   end
10'd128,10'd640:begin if(!multiply_wait)state<=state; end
10'd129,10'd641:begin  end
10'd130,10'd642:begin   end
10'd131,10'd643:begin if(!multiply_wait)state<=state; end
10'd132,10'd644:begin  end
10'd133,10'd645:begin   end
10'd134,10'd646:begin if(!multiply_wait)state<=state; end
10'd135,10'd647:begin  end
10'd136,10'd648:begin   end
10'd137,10'd649:begin if(!multiply_wait)state<=state; end
10'd138,10'd650:begin  end
10'd139,10'd651:begin   end
10'd140,10'd652:begin if(!multiply_wait)state<=state; end
10'd141,10'd653:begin  end
10'd142,10'd654:begin   end
10'd143,10'd655:begin if(!multiply_wait)state<=state; end
10'd144,10'd656:begin  end
10'd145,10'd657:begin   end
10'd146,10'd658:begin if(!multiply_wait)state<=state; end
10'd147,10'd659:begin  end
10'd148,10'd660:begin   end
10'd149,10'd661:begin if(!multiply_wait)state<=state; end
10'd150,10'd662:begin  end
10'd151,10'd663:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd151;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd152,10'd664:begin  end
10'd153,10'd665:begin   end
10'd154,10'd666:begin if(!multiply_wait)state<=state; end
10'd155,10'd667:begin  end
10'd156,10'd668:begin   end
10'd157,10'd669:begin if(!multiply_wait)state<=state; end
10'd158,10'd670:begin  end
10'd159,10'd671:begin   end
10'd160,10'd672:begin if(!multiply_wait)state<=state; end
10'd161,10'd673:begin  end
10'd162,10'd674:begin   end
10'd163,10'd675:begin if(!multiply_wait)state<=state; end
10'd164,10'd676:begin  end
10'd165,10'd677:begin   end
10'd166,10'd678:begin if(!multiply_wait)state<=state; end
10'd167,10'd679:begin  end
10'd168,10'd680:begin   end
10'd169,10'd681:begin if(!multiply_wait)state<=state; end
10'd170,10'd682:begin  end
10'd171,10'd683:begin   end
10'd172,10'd684:begin if(!multiply_wait)state<=state; end
10'd173,10'd685:begin  end
10'd174,10'd686:begin   end
10'd175,10'd687:begin if(!multiply_wait)state<=state; end
10'd176,10'd688:begin  end
10'd177,10'd689:begin   end
10'd178,10'd690:begin if(!multiply_wait)state<=state; end
10'd179,10'd691:begin  end
10'd180,10'd692:begin   end
10'd181,10'd693:begin if(!multiply_wait)state<=state; end
10'd182,10'd694:begin  end
10'd183,10'd695:begin   end
10'd184,10'd696:begin if(!multiply_wait)state<=state; end
10'd185,10'd697:begin  end
10'd186,10'd698:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd186;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd187,10'd699:begin  end
10'd188,10'd700:begin   end
10'd189,10'd701:begin if(!multiply_wait)state<=state; end
10'd190,10'd702:begin  end
10'd191,10'd703:begin   end
10'd192,10'd704:begin if(!multiply_wait)state<=state; end
10'd193,10'd705:begin  end
10'd194,10'd706:begin   end
10'd195,10'd707:begin if(!multiply_wait)state<=state; end
10'd196,10'd708:begin  end
10'd197,10'd709:begin   end
10'd198,10'd710:begin if(!multiply_wait)state<=state; end
10'd199,10'd711:begin  end
10'd200,10'd712:begin   end
10'd201,10'd713:begin if(!multiply_wait)state<=state; end
10'd202,10'd714:begin  end
10'd203,10'd715:begin   end
10'd204,10'd716:begin if(!multiply_wait)state<=state; end
10'd205,10'd717:begin  end
10'd206,10'd718:begin   end
10'd207,10'd719:begin if(!multiply_wait)state<=state; end
10'd208,10'd720:begin  end
10'd209,10'd721:begin   end
10'd210,10'd722:begin if(!multiply_wait)state<=state; end
10'd211,10'd723:begin  end
10'd212,10'd724:begin   end
10'd213,10'd725:begin if(!multiply_wait)state<=state; end
10'd214,10'd726:begin  end
10'd215,10'd727:begin   end
10'd216,10'd728:begin if(!multiply_wait)state<=state; end
10'd217,10'd729:begin  end
10'd218,10'd730:begin   end
10'd219,10'd731:begin if(!multiply_wait)state<=state; end
10'd220,10'd732:begin  end
10'd221,10'd733:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd221;state<=9'd414;end else begin r39_matrix_added<=0; if(r39_mode)begin end end end
10'd222,10'd734:begin  end
10'd223,10'd735:begin   end
10'd224,10'd736:begin if(!multiply_wait)state<=state; end
10'd225,10'd737:begin  end
10'd226,10'd738:begin   end
10'd227,10'd739:begin if(!multiply_wait)state<=state; end
10'd228,10'd740:begin  end
10'd229,10'd741:begin   end
10'd230,10'd742:begin if(!multiply_wait)state<=state; end
10'd231,10'd743:begin  end
10'd232,10'd744:begin   end
10'd233,10'd745:begin if(!multiply_wait)state<=state; end
10'd234,10'd746:begin  end
10'd235,10'd747:begin   end
10'd236,10'd748:begin if(!multiply_wait)state<=state; end
10'd237,10'd749:begin  end
10'd238,10'd750:begin   end
10'd239,10'd751:begin if(!multiply_wait)state<=state; end
10'd240,10'd752:begin  end
10'd241,10'd753:begin   end
10'd242,10'd754:begin if(!multiply_wait)state<=state; end
10'd243,10'd755:begin  end
10'd244,10'd756:begin   end
10'd245,10'd757:begin if(!multiply_wait)state<=state; end
10'd246,10'd758:begin  end
10'd247,10'd759:begin   end
10'd248,10'd760:begin if(!multiply_wait)state<=state; end
10'd249,10'd761:begin  end
10'd250,10'd762:begin   end
10'd251,10'd763:begin if(!multiply_wait)state<=state; end
10'd252,10'd764:begin  end
10'd253,10'd765:begin   end
10'd254,10'd766:begin if(!multiply_wait)state<=state; end
10'd255,10'd767:begin  r36_platform_clamp_s<=r35_static_i_unwind_only&&r36_fast_good>=201; end
10'd256,10'd768:begin if(r39_mode && !r39_matrix_added)begin r39_matrix_added<=1;r39_matrix_return<=9'd256;state<=9'd414;end else begin r39_matrix_added<=0;feedback_velocity<=r36_platform_clamp_s ? (r39t5_data_cmp54 ? 32'sd50332:(r39t5_data_cmp55 ? -32'sd50332:n3)):n3; if(r39_mode)begin state<=9'd430;end end end
10'd257,10'd769:begin   end
10'd258,10'd770:begin  end
10'd259,10'd771:begin  end
10'd260,10'd772:begin   end
10'd261,10'd773:begin /* existing synchronous table read */ end
10'd262,10'd774:begin effective<=r36_rate_correction[31:0]; end
10'd263,10'd775:begin  end
10'd264,10'd776:begin position_error <= n2-reference_s; end
10'd265,10'd777:begin    end
10'd266,10'd778:begin if(!multiply_wait)state<=state; end
10'd267,10'd779:begin integral_lp <= (!automatic_s && manual_moved && velocity_s==0 && r39t5_data_cmp46 && r39t5_data_cmp47) ? sat($signed({{32{integral_lp[31]}},integral_lp})+(product >>> 24)) : (!(automatic_s && drift_s) && (r39t5_data_cmp30 || r39t5_data_cmp31 || r39t5_data_cmp32 || r39t5_data_cmp33)) ? ((automatic_s || r39t5_data_cmp34 || r39t5_data_cmp35) && r39t5_data_cmp36 && r39t5_data_cmp37 ? integral_lp:32'sd0) : sat($signed({{32{integral_lp[31]}},integral_lp})+(product >>> 24)); end
10'd268,10'd780:begin    end
10'd269,10'd781:begin if(!multiply_wait)state<=state; end
10'd270,10'd782:begin integral_step <= r40_position_unwind ? (r40_unwind_step>32'sd16777 ? 32'sd16777 : r40_unwind_step< -32'sd16777 ? -32'sd16777 : r40_unwind_step) : sat(product >>> 24); end
10'd271,10'd783:begin output_slew<=(automatic_s && motion_latched && !brake_s && r39t5_data_cmp56 && r39t5_data_cmp57 && (velocity_s==0 ? (r39t5_data_cmp16 && r39t5_data_cmp17):(r38_current_s && r39t5_data_cmp58 && r39t5_data_cmp59)))?32'sd5:32'sd20; theta_gain <= automatic_s ? (brake_s ? 32'sd67937104 : motion_gain_0(motion_phase)) : (r39t5_data_cmp38 || r39t5_data_cmp39 || r39t5_data_cmp40 || r39t5_data_cmp41) ? 32'sd180666024 : 32'sd126466217; end
10'd272,10'd784:begin  end
10'd273,10'd785:begin    end
10'd274,10'd786:begin if(!multiply_wait)state<=state; end
10'd275,10'd787:begin  end
10'd276,10'd788:begin    end
10'd277,10'd789:begin if(!multiply_wait)state<=state; end
10'd278,10'd790:begin  end
10'd279,10'd791:begin    end
10'd280,10'd792:begin if(!multiply_wait)state<=state; end
10'd281,10'd793:begin  end
10'd282,10'd794:begin    end
10'd283,10'd795:begin if(!multiply_wait)state<=state; end
10'd284,10'd796:begin  end
10'd285,10'd797:begin    end
10'd286,10'd798:begin if(!multiply_wait)state<=state; end
10'd287,10'd799:begin  end
10'd288,10'd800:begin    end
10'd289,10'd801:begin if(!multiply_wait)state<=state; end
10'd290,10'd802:begin  end
10'd291,10'd803:begin    end
10'd292,10'd804:begin if(!multiply_wait)state<=state; end
10'd293,10'd805:begin  end
10'd294,10'd806:begin   end
10'd295,10'd807:begin if(!multiply_wait)state<=state; end
10'd296,10'd808:begin  end
10'd297,10'd809:begin   end
10'd298,10'd810:begin if(!multiply_wait)state<=state; end
10'd299,10'd811:begin  end
10'd300,10'd812:begin    end
10'd301,10'd813:begin if(!multiply_wait)state<=state; end
10'd302,10'd814:begin  end
10'd303,10'd815:begin request <= sat(acc); end
10'd304,10'd816:begin    end
10'd305,10'd817:begin if(!multiply_wait)state<=state; end
10'd306,10'd818:begin request <= sat((product >>> 24) + (request == 0 ? 64'sd0 : request < 0 ? -64'sd298096 : 64'sd298096)) ; end
// Pipeline slew bounds then intersect them with the hard cap. A cap reduction
// overrides slew without adding latency or a new saturation chain at commit.
10'd307,10'd819:begin slew_low<=previous-output_slew;slew_high<=previous+output_slew;conversion <= (($signed({{32{request[31]}},request})<<<10)-($signed({{32{request[31]}},request})<<<4)-($signed({{32{request[31]}},request})<<<3)); end
10'd308,10'd820:begin slew_low<=slew_low>cap?cap:slew_low< -cap?-cap:slew_low;slew_high<=slew_high>cap?cap:slew_high< -cap?-cap:slew_high;magnitude <= conversion < 0 ? -conversion : conversion; negative <= conversion < 0; end
10'd309,10'd821:begin rounded <= magnitude[55:24]+{31'd0,((magnitude[23:0]>24'd8388608)||((magnitude[23:0]==24'd8388608)&&magnitude[24]))}; end
10'd310,10'd822:begin wanted <= negative ? -$signed(rounded) : $signed(rounded); end
10'd311,10'd823:begin bounded <= wanted > cap ? cap : wanted < -cap ? -cap : wanted; end
10'd312,10'd824:begin delivered <= bounded > slew_high ? slew_high : bounded < slew_low ? slew_low : bounded; end
10'd313,10'd825:begin   end
10'd314,10'd826:begin if(!multiply_wait)state<=state; end
10'd315,10'd827:begin /* scratch output_error written in independent datapath */ end
// R35: only unwind I in an already balanced static automatic HOLD.
// No new gain, observer state, output limit or frame/transaction stage.
10'd316:begin
 if(!r35_static_i_eligible && !r38_endpoint_stasis_eligible)r35_stasis_samples<=0;
 else if(r35_stasis_samples!=0 && r35_stasis_error==r36_phi_error_s)begin
  if(r35_stasis_samples<8'd201)r35_stasis_samples<=r35_stasis_samples+1'b1;
 end else r35_stasis_samples<=8'd1;
 r35_stasis_error<=r36_phi_error_s;
 if(r40_integral_update_allowed) integral <= (r40_position_unwind && r40_unwind_zero) ? 32'sd0 : (((r40_position_unwind || (!automatic_s && manual_moved && velocity_s==0 && r39t5_data_cmp46 && r39t5_data_cmp47)) && !(r39t5_data_cmp16 && r39t5_data_cmp17 && r39t5_data_cmp18 && r39t5_data_cmp19) && ((integral>=0 && integral+integral_step<0) || (integral<0 && integral+integral_step>0))) || (r35_static_i_unwind_only && ((integral>0 && integral+integral_step<0) || (integral<0 && integral+integral_step>0)))) ? 32'sd0 : integral+integral_step>32'sd5872026 ? 32'sd5872026:integral+integral_step< -32'sd5872026 ? -32'sd5872026:integral+integral_step; if(age<1000)age<=age+1'b1; end
10'd317:begin
 if(automatic_s && motion_latched && !brake_s && age>=200 && motion_phase<3)begin
  if(motion_ticks>=49)begin motion_ticks<=0;motion_phase<=motion_phase+1'b1;end else motion_ticks<=motion_ticks+1'b1;
 end
 duty <= delivered; z0<=n0; z1<=n1; z2<=n2; z3<=n3; z4<=n4; z5<=n5; z6<=n6; if(samples<200)samples<=samples+1; ready<=samples>=200;
 if(!automatic_s)begin busy<=0;done<=1;state<=0;end
 else if(age>=200 && (hold_s || motion_latched))blend<=blend>32'sd16693330 ? 32'sd16777216:blend+32'sd83886;
 else blend<=blend<32'sd1048576 ? 32'sd0:blend-32'sd1048576; end
10'd318:begin end
10'd319:begin if(!multiply_wait)state<=state; end
10'd320:begin magnitude<=product<0?-product:product;negative<=product<0;end
10'd321:begin rounded<=magnitude[55:24]+{31'd0,((magnitude[23:0]>24'd8388608)||((magnitude[23:0]==24'd8388608)&&magnitude[24]))};end
10'd322:begin duty<=fallback_s+(negative?-$signed(rounded):$signed(rounded));busy<=0;done<=1;state<=0;end
   default:begin fault<=1;busy<=0;done<=1;duty<=0;state<=0;end
  endcase
 end
 // Called after a complete guard transaction; retain observer state/age/I.
 // Only the original feedback handover weight restarts from zero.
 if(handoff_reblend)blend<=0;
 end
end

// R39T5 data-only constant-comparison decomposition; no storage or clock change.
wire r39t5_data_cmp0=((!velocity_s[31]) && ((|velocity_s[30:23]) || (velocity_s[22:0] > 23'd4194304)));
wire r39t5_data_cmp1=(velocity_s[31] && ((!(&velocity_s[30:23])) || (velocity_s[22:0] < 23'd4194304)));
wire r39t5_data_cmp2=(acc[43] || ((~|acc[42:13]) && (acc[12:0] <= 13'd5719)));
wire r39t5_data_cmp3=((!acc[43]) || ((&acc[42:13]) && (acc[12:0] >= 13'd2473)));
wire r39t5_data_cmp4=((!r36_rate_error[36]) && ((|r36_rate_error[35:24]) || (r36_rate_error[23:0] > 24'd13421773)));
wire r39t5_data_cmp5=(r36_rate_error[36] && ((!(&r36_rate_error[35:24])) || (r36_rate_error[23:0] < 24'd3355443)));
wire r39t5_data_cmp6=(theta_s[31] || ((~|theta_s[30:22]) && (theta_s[21:0] < 22'd2516480)));
wire r39t5_data_cmp7=((!theta_s[31]) || ((&theta_s[30:22]) && (theta_s[21:0] > 22'd1677824)));
wire r39t5_data_cmp8=((!acc[43]) && ((|acc[42:24]) || (acc[23:0] > 24'd13421773)));
wire r39t5_data_cmp9=(acc[43] && ((!(&acc[42:24])) || (acc[23:0] < 24'd3355443)));
wire r39t5_data_cmp10=((!r36_rate_error[36]) && ((|r36_rate_error[35:22]) || (r36_rate_error[21:0] > 22'd3355443)));
wire r39t5_data_cmp11=(r36_rate_error[36] && ((!(&r36_rate_error[35:22])) || (r36_rate_error[21:0] < 22'd838861)));
wire r39t5_data_cmp12=((!r38_rate_replace_sum[36]) && ((|r38_rate_replace_sum[35:18]) || (r38_rate_replace_sum[17:0] > 18'd243012)));
wire r39t5_data_cmp13=(r38_rate_replace_sum[36] && ((!(&r38_rate_replace_sum[35:18])) || (r38_rate_replace_sum[17:0] < 18'd19132)));
wire r39t5_data_cmp14=(theta_s[31] || ((~|theta_s[30:19]) && (theta_s[18:0] < 19'd292864)));
wire r39t5_data_cmp15=((!theta_s[31]) || ((&theta_s[30:19]) && (theta_s[18:0] > 19'd231424)));
wire r39t5_data_cmp16=(n1[31] || ((~|n1[30:20]) && (n1[19:0] < 20'd838861)));
wire r39t5_data_cmp17=((!n1[31]) || ((&n1[30:20]) && (n1[19:0] > 20'd209715)));
wire r39t5_data_cmp18=(n3[31] || ((~|n3[30:20]) && (n3[19:0] < 20'd838861)));
wire r39t5_data_cmp19=((!n3[31]) || ((&n3[30:20]) && (n3[19:0] > 20'd209715)));
wire r39t5_data_cmp20=(theta_s[31] || ((~|theta_s[30:20]) && (theta_s[19:0] < 20'd878592)));
wire r39t5_data_cmp21=((!theta_s[31]) || ((&theta_s[30:20]) && (theta_s[19:0] > 20'd169984)));
wire r39t5_data_cmp22=((!sine_bias[31]) || ((&sine_bias[30:5]) && (sine_bias[4:0] >= 5'd1)));
wire r39t5_data_cmp23=(sine_bias[31] || ((~|sine_bias[30:5]) && (sine_bias[4:0] <= 5'd31)));
wire r39t5_data_cmp24=((!encoder_residual[32]) && ((|encoder_residual[31:18]) || (encoder_residual[17:0] > 18'd167772)));
wire r39t5_data_cmp25=(encoder_residual[32] && ((!(&encoder_residual[31:18])) || (encoder_residual[17:0] < 18'd94372)));
wire r39t5_data_cmp26=((!encoder_residual[32]) && ((|encoder_residual[31:19]) || (encoder_residual[18:0] > 19'd335544)));
wire r39t5_data_cmp27=(encoder_residual[32] && ((!(&encoder_residual[31:19])) || (encoder_residual[18:0] < 19'd188744)));
wire r39t5_data_cmp28=((!position_error[31]) && ((|position_error[30:23]) || (position_error[22:0] > 23'd5856353)));
wire r39t5_data_cmp29=(position_error[31] && ((!(&position_error[30:23])) || (position_error[22:0] < 23'd2532255)));
wire r39t5_data_cmp30=((!n1[31]) && ((|n1[30:20]) || (n1[19:0] > 20'd838861)));
wire r39t5_data_cmp31=(n1[31] && ((!(&n1[30:20])) || (n1[19:0] < 20'd209715)));
wire r39t5_data_cmp32=((!n3[31]) && ((|n3[30:20]) || (n3[19:0] > 20'd838861)));
wire r39t5_data_cmp33=(n3[31] && ((!(&n3[30:20])) || (n3[19:0] < 20'd209715)));
wire r39t5_data_cmp34=((!reference_s[31]) && ((|reference_s[30:20]) || (reference_s[19:0] > 20'd585728)));
wire r39t5_data_cmp35=(reference_s[31] && ((!(&reference_s[30:20])) || (reference_s[19:0] < 20'd462848)));
wire r39t5_data_cmp36=(velocity_s[31] || ((~|velocity_s[30:21]) && (velocity_s[20:0] < 21'd1677824)));
wire r39t5_data_cmp37=((!velocity_s[31]) || ((&velocity_s[30:21]) && (velocity_s[20:0] > 21'd419328)));
wire r39t5_data_cmp38=((!n1[31]) && ((|n1[30:22]) || (n1[21:0] > 22'd2516582)));
wire r39t5_data_cmp39=(n1[31] && ((!(&n1[30:22])) || (n1[21:0] < 22'd1677722)));
wire r39t5_data_cmp40=((!n0[31]) && ((|n0[30:21]) || (n0[20:0] > 21'd1464088)));
wire r39t5_data_cmp41=(n0[31] && ((!(&n0[30:21])) || (n0[20:0] < 21'd633064)));
wire r39t5_data_cmp42=((!shape_band[32]) && ((|shape_band[31:20]) || (shape_band[19:0] > 20'd671088)));
wire r39t5_data_cmp43=(shape_band[32] && ((!(&shape_band[31:20])) || (shape_band[19:0] < 20'd377488)));
wire r39t5_data_cmp44=((!shape_band[32]) && ((|shape_band[31:22]) || (shape_band[21:0] > 22'd2684355)));
wire r39t5_data_cmp45=(shape_band[32] && ((!(&shape_band[31:22])) || (shape_band[21:0] < 22'd1509952)));
wire r39t5_data_cmp46=(theta_s[31] || ((~|theta_s[30:22]) && (theta_s[21:0] < 22'd2348810)));
wire r39t5_data_cmp47=((!theta_s[31]) || ((&theta_s[30:22]) && (theta_s[21:0] > 22'd1845494)));
wire r39t5_data_cmp48=(output_error[31] || ((~|output_error[30:16]) && (output_error[15:0] < 16'd50332)));
wire r39t5_data_cmp49=((!output_error[31]) || ((&output_error[30:16]) && (output_error[15:0] > 16'd15204)));
wire r39t5_data_cmp50=((!smooth_residual[33]) && ((|smooth_residual[32:16]) || (smooth_residual[15:0] > 16'd50688)));
wire r39t5_data_cmp51=(smooth_residual[33] && ((!(&smooth_residual[32:16])) || (smooth_residual[15:0] < 16'd14848)));
wire r39t5_data_cmp52=((!acc[43]) && ((|acc[42:23]) || (acc[22:0] > 23'd5033165)));
wire r39t5_data_cmp53=(acc[43] && ((!(&acc[42:23])) || (acc[22:0] < 23'd3355443)));
wire r39t5_data_cmp54=((!n3[31]) && ((|n3[30:16]) || (n3[15:0] > 16'd50332)));
wire r39t5_data_cmp55=(n3[31] && ((!(&n3[30:16])) || (n3[15:0] < 16'd15204)));
wire r39t5_data_cmp56=(theta_s[31] || ((~|theta_s[30:20]) && (theta_s[19:0] < 20'd670976)));
wire r39t5_data_cmp57=((!theta_s[31]) || ((&theta_s[30:20]) && (theta_s[19:0] > 20'd377600)));
wire r39t5_data_cmp58=(r36_omega_s[31] || ((~|r36_omega_s[30:24]) && (r36_omega_s[23:0] < 24'd8388608)));
wire r39t5_data_cmp59=((!r36_omega_s[31]) || ((&r36_omega_s[30:24]) && (r36_omega_s[23:0] > 24'd8388608)));
// R39T7 current-state lookahead micro-op ROM, original output edges.
reg[68:0] r39t7_look_rom[0:1023] /* synthesis syn_romstyle = "block_rom" */;
reg[68:0] r39t7_look_q;
reg r39t7_wait_q,r39t7_added_q,r39t7_shape_q,r39t7_auto_q;
reg[2:0]r39t7_return_q;
reg[1:0]r39t7_override_q;
reg[2:0]r39t7_return_op;
always @* begin
 r39t7_return_op=0;
 case(r39_matrix_return)
 9'd46:r39t7_return_op=1;9'd81:r39t7_return_op=2;
 9'd116:r39t7_return_op=3;9'd151:r39t7_return_op=4;
 9'd186:r39t7_return_op=5;9'd221:r39t7_return_op=6;
 9'd256:r39t7_return_op=7;default:begin end
 endcase
end
initial begin
r39t7_look_rom[0]=69'h000000000000000000;
r39t7_look_rom[1]=69'h000000000000400000;
r39t7_look_rom[2]=69'h000000000000500000;
r39t7_look_rom[3]=69'h000000000000600000;
r39t7_look_rom[4]=69'h000000000000000000;
r39t7_look_rom[5]=69'h000000000000700000;
r39t7_look_rom[6]=69'h000000000000800000;
r39t7_look_rom[7]=69'h000000000000000000;
r39t7_look_rom[8]=69'h000000040000001408;
r39t7_look_rom[9]=69'h000000000000008000;
r39t7_look_rom[10]=69'h000000000100000000;
r39t7_look_rom[11]=69'h000000000000030000;
r39t7_look_rom[12]=69'h00200000000000081b;
r39t7_look_rom[13]=69'h000000000000008000;
r39t7_look_rom[14]=69'h000000000100040000;
r39t7_look_rom[15]=69'h00400000000000081c;
r39t7_look_rom[16]=69'h000000000000008000;
r39t7_look_rom[17]=69'h000000000100040000;
r39t7_look_rom[18]=69'h00800000000000081d;
r39t7_look_rom[19]=69'h000000000000008000;
r39t7_look_rom[20]=69'h000000000100040000;
r39t7_look_rom[21]=69'h01000000000000081e;
r39t7_look_rom[22]=69'h000000000000008000;
r39t7_look_rom[23]=69'h000000000100040000;
r39t7_look_rom[24]=69'h02000000000000081f;
r39t7_look_rom[25]=69'h000000000000008000;
r39t7_look_rom[26]=69'h000000000100040000;
r39t7_look_rom[27]=69'h040000000000000820;
r39t7_look_rom[28]=69'h000000000000008000;
r39t7_look_rom[29]=69'h000000000100040000;
r39t7_look_rom[30]=69'h080000000000000821;
r39t7_look_rom[31]=69'h000000000000008000;
r39t7_look_rom[32]=69'h000000000100040000;
r39t7_look_rom[33]=69'h000000040000000808;
r39t7_look_rom[34]=69'h000000000000008000;
r39t7_look_rom[35]=69'h000000000100040000;
r39t7_look_rom[36]=69'h000400000000000818;
r39t7_look_rom[37]=69'h000000000000008000;
r39t7_look_rom[38]=69'h000000000100040000;
r39t7_look_rom[39]=69'h000800000000000819;
r39t7_look_rom[40]=69'h000000000000008000;
r39t7_look_rom[41]=69'h000000000100040000;
r39t7_look_rom[42]=69'h000004000000000810;
r39t7_look_rom[43]=69'h000000000000008000;
r39t7_look_rom[44]=69'h000000000100040000;
r39t7_look_rom[45]=69'h000000000004000000;
r39t7_look_rom[46]=69'h000000000000030000;
r39t7_look_rom[47]=69'h00200000000000081b;
r39t7_look_rom[48]=69'h000000000000008000;
r39t7_look_rom[49]=69'h000000000100040000;
r39t7_look_rom[50]=69'h00400000000000081c;
r39t7_look_rom[51]=69'h000000000000008000;
r39t7_look_rom[52]=69'h000000000100040000;
r39t7_look_rom[53]=69'h00800000000000081d;
r39t7_look_rom[54]=69'h000000000000008000;
r39t7_look_rom[55]=69'h000000000100040000;
r39t7_look_rom[56]=69'h01000000000000081e;
r39t7_look_rom[57]=69'h000000000000008000;
r39t7_look_rom[58]=69'h000000000100040000;
r39t7_look_rom[59]=69'h02000000000000081f;
r39t7_look_rom[60]=69'h000000000000008000;
r39t7_look_rom[61]=69'h000000000100040000;
r39t7_look_rom[62]=69'h040000000000000820;
r39t7_look_rom[63]=69'h000000000000008000;
r39t7_look_rom[64]=69'h000000000100040000;
r39t7_look_rom[65]=69'h080000000000000821;
r39t7_look_rom[66]=69'h000000000000008000;
r39t7_look_rom[67]=69'h000000000100040000;
r39t7_look_rom[68]=69'h000000040000000808;
r39t7_look_rom[69]=69'h000000000000008000;
r39t7_look_rom[70]=69'h000000000100040000;
r39t7_look_rom[71]=69'h000400000000000818;
r39t7_look_rom[72]=69'h000000000000008000;
r39t7_look_rom[73]=69'h000000000100040000;
r39t7_look_rom[74]=69'h000800000000000819;
r39t7_look_rom[75]=69'h000000000000008000;
r39t7_look_rom[76]=69'h000000000100040000;
r39t7_look_rom[77]=69'h000004000000000810;
r39t7_look_rom[78]=69'h000000000000008000;
r39t7_look_rom[79]=69'h000000000100040000;
r39t7_look_rom[80]=69'h000000000008000000;
r39t7_look_rom[81]=69'h000000000000030000;
r39t7_look_rom[82]=69'h00200000000000081b;
r39t7_look_rom[83]=69'h000000000000008000;
r39t7_look_rom[84]=69'h000000000100040000;
r39t7_look_rom[85]=69'h00400000000000081c;
r39t7_look_rom[86]=69'h000000000000008000;
r39t7_look_rom[87]=69'h000000000100040000;
r39t7_look_rom[88]=69'h00800000000000081d;
r39t7_look_rom[89]=69'h000000000000008000;
r39t7_look_rom[90]=69'h000000000100040000;
r39t7_look_rom[91]=69'h01000000000000081e;
r39t7_look_rom[92]=69'h000000000000008000;
r39t7_look_rom[93]=69'h000000000100040000;
r39t7_look_rom[94]=69'h02000000000000081f;
r39t7_look_rom[95]=69'h000000000000008000;
r39t7_look_rom[96]=69'h000000000100040000;
r39t7_look_rom[97]=69'h040000000000000820;
r39t7_look_rom[98]=69'h000000000000008000;
r39t7_look_rom[99]=69'h000000000100040000;
r39t7_look_rom[100]=69'h080000000000000821;
r39t7_look_rom[101]=69'h000000000000008000;
r39t7_look_rom[102]=69'h000000000100040000;
r39t7_look_rom[103]=69'h000000040000000808;
r39t7_look_rom[104]=69'h000000000000008000;
r39t7_look_rom[105]=69'h000000000100040000;
r39t7_look_rom[106]=69'h000400000000000818;
r39t7_look_rom[107]=69'h000000000000008000;
r39t7_look_rom[108]=69'h000000000100040000;
r39t7_look_rom[109]=69'h000800000000000819;
r39t7_look_rom[110]=69'h000000000000008000;
r39t7_look_rom[111]=69'h000000000100040000;
r39t7_look_rom[112]=69'h000004000000000810;
r39t7_look_rom[113]=69'h000000000000008000;
r39t7_look_rom[114]=69'h000000000100040000;
r39t7_look_rom[115]=69'h00000000000c000000;
r39t7_look_rom[116]=69'h000000000000030000;
r39t7_look_rom[117]=69'h00200000000000081b;
r39t7_look_rom[118]=69'h000000000000008000;
r39t7_look_rom[119]=69'h000000000100040000;
r39t7_look_rom[120]=69'h00400000000000081c;
r39t7_look_rom[121]=69'h000000000000008000;
r39t7_look_rom[122]=69'h000000000100040000;
r39t7_look_rom[123]=69'h00800000000000081d;
r39t7_look_rom[124]=69'h000000000000008000;
r39t7_look_rom[125]=69'h000000000100040000;
r39t7_look_rom[126]=69'h01000000000000081e;
r39t7_look_rom[127]=69'h000000000000008000;
r39t7_look_rom[128]=69'h000000000100040000;
r39t7_look_rom[129]=69'h02000000000000081f;
r39t7_look_rom[130]=69'h000000000000008000;
r39t7_look_rom[131]=69'h000000000100040000;
r39t7_look_rom[132]=69'h040000000000000820;
r39t7_look_rom[133]=69'h000000000000008000;
r39t7_look_rom[134]=69'h000000000100040000;
r39t7_look_rom[135]=69'h080000000000000821;
r39t7_look_rom[136]=69'h000000000000008000;
r39t7_look_rom[137]=69'h000000000100040000;
r39t7_look_rom[138]=69'h000000040000000808;
r39t7_look_rom[139]=69'h000000000000008000;
r39t7_look_rom[140]=69'h000000000100040000;
r39t7_look_rom[141]=69'h000400000000000818;
r39t7_look_rom[142]=69'h000000000000008000;
r39t7_look_rom[143]=69'h000000000100040000;
r39t7_look_rom[144]=69'h000800000000000819;
r39t7_look_rom[145]=69'h000000000000008000;
r39t7_look_rom[146]=69'h000000000100040000;
r39t7_look_rom[147]=69'h000004000000000810;
r39t7_look_rom[148]=69'h000000000000008000;
r39t7_look_rom[149]=69'h000000000100040000;
r39t7_look_rom[150]=69'h000000000010000000;
r39t7_look_rom[151]=69'h000000000000030000;
r39t7_look_rom[152]=69'h00200000000000081b;
r39t7_look_rom[153]=69'h000000000000008000;
r39t7_look_rom[154]=69'h000000000100040000;
r39t7_look_rom[155]=69'h00400000000000081c;
r39t7_look_rom[156]=69'h000000000000008000;
r39t7_look_rom[157]=69'h000000000100040000;
r39t7_look_rom[158]=69'h00800000000000081d;
r39t7_look_rom[159]=69'h000000000000008000;
r39t7_look_rom[160]=69'h000000000100040000;
r39t7_look_rom[161]=69'h01000000000000081e;
r39t7_look_rom[162]=69'h000000000000008000;
r39t7_look_rom[163]=69'h000000000100040000;
r39t7_look_rom[164]=69'h02000000000000081f;
r39t7_look_rom[165]=69'h000000000000008000;
r39t7_look_rom[166]=69'h000000000100040000;
r39t7_look_rom[167]=69'h040000000000000820;
r39t7_look_rom[168]=69'h000000000000008000;
r39t7_look_rom[169]=69'h000000000100040000;
r39t7_look_rom[170]=69'h080000000000000821;
r39t7_look_rom[171]=69'h000000000000008000;
r39t7_look_rom[172]=69'h000000000100040000;
r39t7_look_rom[173]=69'h000000040000000808;
r39t7_look_rom[174]=69'h000000000000008000;
r39t7_look_rom[175]=69'h000000000100040000;
r39t7_look_rom[176]=69'h000400000000000818;
r39t7_look_rom[177]=69'h000000000000008000;
r39t7_look_rom[178]=69'h000000000100040000;
r39t7_look_rom[179]=69'h000800000000000819;
r39t7_look_rom[180]=69'h000000000000008000;
r39t7_look_rom[181]=69'h000000000100040000;
r39t7_look_rom[182]=69'h000004000000000810;
r39t7_look_rom[183]=69'h000000000000008000;
r39t7_look_rom[184]=69'h000000000100040000;
r39t7_look_rom[185]=69'h000000000014000000;
r39t7_look_rom[186]=69'h000000000000030000;
r39t7_look_rom[187]=69'h00200000000000081b;
r39t7_look_rom[188]=69'h000000000000008000;
r39t7_look_rom[189]=69'h000000000100040000;
r39t7_look_rom[190]=69'h00400000000000081c;
r39t7_look_rom[191]=69'h000000000000008000;
r39t7_look_rom[192]=69'h000000000100040000;
r39t7_look_rom[193]=69'h00800000000000081d;
r39t7_look_rom[194]=69'h000000000000008000;
r39t7_look_rom[195]=69'h000000000100040000;
r39t7_look_rom[196]=69'h01000000000000081e;
r39t7_look_rom[197]=69'h000000000000008000;
r39t7_look_rom[198]=69'h000000000100040000;
r39t7_look_rom[199]=69'h02000000000000081f;
r39t7_look_rom[200]=69'h000000000000008000;
r39t7_look_rom[201]=69'h000000000100040000;
r39t7_look_rom[202]=69'h040000000000000820;
r39t7_look_rom[203]=69'h000000000000008000;
r39t7_look_rom[204]=69'h000000000100040000;
r39t7_look_rom[205]=69'h080000000000000821;
r39t7_look_rom[206]=69'h000000000000008000;
r39t7_look_rom[207]=69'h000000000100040000;
r39t7_look_rom[208]=69'h000000040000000808;
r39t7_look_rom[209]=69'h000000000000008000;
r39t7_look_rom[210]=69'h000000000100040000;
r39t7_look_rom[211]=69'h000400000000000818;
r39t7_look_rom[212]=69'h000000000000008000;
r39t7_look_rom[213]=69'h000000000100040000;
r39t7_look_rom[214]=69'h000800000000000819;
r39t7_look_rom[215]=69'h000000000000008000;
r39t7_look_rom[216]=69'h000000000100040000;
r39t7_look_rom[217]=69'h000004000000000810;
r39t7_look_rom[218]=69'h000000000000008000;
r39t7_look_rom[219]=69'h000000000100040000;
r39t7_look_rom[220]=69'h000000000018000000;
r39t7_look_rom[221]=69'h000000000000030000;
r39t7_look_rom[222]=69'h00200000000000081b;
r39t7_look_rom[223]=69'h000000000000008000;
r39t7_look_rom[224]=69'h000000000100040000;
r39t7_look_rom[225]=69'h00400000000000081c;
r39t7_look_rom[226]=69'h000000000000008000;
r39t7_look_rom[227]=69'h000000000100040000;
r39t7_look_rom[228]=69'h00800000000000081d;
r39t7_look_rom[229]=69'h000000000000008000;
r39t7_look_rom[230]=69'h000000000100040000;
r39t7_look_rom[231]=69'h01000000000000081e;
r39t7_look_rom[232]=69'h000000000000008000;
r39t7_look_rom[233]=69'h000000000100040000;
r39t7_look_rom[234]=69'h02000000000000081f;
r39t7_look_rom[235]=69'h000000000000008000;
r39t7_look_rom[236]=69'h000000000100040000;
r39t7_look_rom[237]=69'h040000000000000820;
r39t7_look_rom[238]=69'h000000000000008000;
r39t7_look_rom[239]=69'h000000000100040000;
r39t7_look_rom[240]=69'h080000000000000821;
r39t7_look_rom[241]=69'h000000000000008000;
r39t7_look_rom[242]=69'h000000000100040000;
r39t7_look_rom[243]=69'h000000040000000808;
r39t7_look_rom[244]=69'h000000000000008000;
r39t7_look_rom[245]=69'h000000000100040000;
r39t7_look_rom[246]=69'h000400000000000818;
r39t7_look_rom[247]=69'h000000000000008000;
r39t7_look_rom[248]=69'h000000000100040000;
r39t7_look_rom[249]=69'h000800000000000819;
r39t7_look_rom[250]=69'h000000000000008000;
r39t7_look_rom[251]=69'h000000000100040000;
r39t7_look_rom[252]=69'h000004000000000810;
r39t7_look_rom[253]=69'h000000000000008000;
r39t7_look_rom[254]=69'h000000000100040000;
r39t7_look_rom[255]=69'h00000000001c090000;
r39t7_look_rom[256]=69'h000000000000380000;
r39t7_look_rom[257]=69'h000000000000420000;
r39t7_look_rom[258]=69'h000000000020505000;
r39t7_look_rom[259]=69'h000000000000660000;
r39t7_look_rom[260]=69'h000000000000050000;
r39t7_look_rom[261]=69'h000000000000700000;
r39t7_look_rom[262]=69'h000000000003000000;
r39t7_look_rom[263]=69'h000000000000000000;
r39t7_look_rom[264]=69'h000040000000003a14;
r39t7_look_rom[265]=69'h000000000000008000;
r39t7_look_rom[266]=69'h000000000100000000;
r39t7_look_rom[267]=69'h0000001000000041ca;
r39t7_look_rom[268]=69'h000000000000008000;
r39t7_look_rom[269]=69'h000000000100000000;
r39t7_look_rom[270]=69'h000000000000000000;
r39t7_look_rom[271]=69'h000000000000070000;
r39t7_look_rom[272]=69'h00000020000000154b;
r39t7_look_rom[273]=69'h000000000000008000;
r39t7_look_rom[274]=69'h000000000100040000;
r39t7_look_rom[275]=69'h0000800000000033d5;
r39t7_look_rom[276]=69'h000000000000008000;
r39t7_look_rom[277]=69'h000000000100040000;
r39t7_look_rom[278]=69'h000010000000003392;
r39t7_look_rom[279]=69'h000000000000008000;
r39t7_look_rom[280]=69'h000000000100040000;
r39t7_look_rom[281]=69'h000020000000003353;
r39t7_look_rom[282]=69'h000000000000008000;
r39t7_look_rom[283]=69'h000000000100040000;
r39t7_look_rom[284]=69'h00000080000000330d;
r39t7_look_rom[285]=69'h000000000000008000;
r39t7_look_rom[286]=69'h000000000100040000;
r39t7_look_rom[287]=69'h00000100000000328e;
r39t7_look_rom[288]=69'h000000000000008000;
r39t7_look_rom[289]=69'h000000000100040000;
r39t7_look_rom[290]=69'h0000020000000032cf;
r39t7_look_rom[291]=69'h000000000000008000;
r39t7_look_rom[292]=69'h000000000100040000;
r39t7_look_rom[293]=69'h000200000000000817;
r39t7_look_rom[294]=69'h000000000000008000;
r39t7_look_rom[295]=69'h000000000100040000;
r39t7_look_rom[296]=69'h000000080000000809;
r39t7_look_rom[297]=69'h000000000000008000;
r39t7_look_rom[298]=69'h000000000100040000;
r39t7_look_rom[299]=69'h001000000000004c9a;
r39t7_look_rom[300]=69'h000000000000008000;
r39t7_look_rom[301]=69'h000000000100040000;
r39t7_look_rom[302]=69'h000000000000000000;
r39t7_look_rom[303]=69'h0001000000000014d6;
r39t7_look_rom[304]=69'h000000000000008000;
r39t7_look_rom[305]=69'h000000000100000000;
r39t7_look_rom[306]=69'h000000000000000000;
r39t7_look_rom[307]=69'h000000000000000000;
r39t7_look_rom[308]=69'h000000000000000000;
r39t7_look_rom[309]=69'h000000000000000000;
r39t7_look_rom[310]=69'h000000000000000000;
r39t7_look_rom[311]=69'h000000000000000000;
r39t7_look_rom[312]=69'h000000010000000806;
r39t7_look_rom[313]=69'h000000000000008000;
r39t7_look_rom[314]=69'h000000000140000000;
r39t7_look_rom[315]=69'h000000000000000000;
r39t7_look_rom[316]=69'h000000000000000000;
r39t7_look_rom[317]=69'h000000020500001247;
r39t7_look_rom[318]=69'h000000000000008000;
r39t7_look_rom[319]=69'h000000000100000000;
r39t7_look_rom[320]=69'h000000000000000000;
r39t7_look_rom[321]=69'h000000000000000000;
r39t7_look_rom[322]=69'h000000000000000000;
r39t7_look_rom[323]=69'h000000000000000000;
r39t7_look_rom[324]=69'h000000000000200000;
r39t7_look_rom[325]=69'h000000000000000000;
r39t7_look_rom[326]=69'h000000000000000000;
r39t7_look_rom[327]=69'h000000000000000000;
r39t7_look_rom[328]=69'h000000000000000000;
r39t7_look_rom[329]=69'h000000000000000000;
r39t7_look_rom[330]=69'h000000000000000000;
r39t7_look_rom[331]=69'h000000000000000000;
r39t7_look_rom[332]=69'h000000000000000000;
r39t7_look_rom[333]=69'h000000000000000000;
r39t7_look_rom[334]=69'h000000000000000000;
r39t7_look_rom[335]=69'h000000000000000000;
r39t7_look_rom[336]=69'h000000000000000000;
r39t7_look_rom[337]=69'h000000000000000000;
r39t7_look_rom[338]=69'h000000000000000000;
r39t7_look_rom[339]=69'h000000000000000000;
r39t7_look_rom[340]=69'h000000000000000000;
r39t7_look_rom[341]=69'h000000000000000000;
r39t7_look_rom[342]=69'h000000000000000000;
r39t7_look_rom[343]=69'h000000000000000000;
r39t7_look_rom[344]=69'h000000000000000000;
r39t7_look_rom[345]=69'h000000000000000000;
r39t7_look_rom[346]=69'h000000000000000000;
r39t7_look_rom[347]=69'h000000000000000000;
r39t7_look_rom[348]=69'h000000000000000000;
r39t7_look_rom[349]=69'h000000000000000000;
r39t7_look_rom[350]=69'h000000000000000000;
r39t7_look_rom[351]=69'h000000000000000000;
r39t7_look_rom[352]=69'h000000000000000000;
r39t7_look_rom[353]=69'h000000000000000000;
r39t7_look_rom[354]=69'h000000000000000000;
r39t7_look_rom[355]=69'h000000000000000000;
r39t7_look_rom[356]=69'h000000000000000000;
r39t7_look_rom[357]=69'h000000000000000000;
r39t7_look_rom[358]=69'h000000000000000000;
r39t7_look_rom[359]=69'h000000000000000000;
r39t7_look_rom[360]=69'h000000000000000000;
r39t7_look_rom[361]=69'h000000000000000000;
r39t7_look_rom[362]=69'h000000000000000000;
r39t7_look_rom[363]=69'h000000000000000000;
r39t7_look_rom[364]=69'h000000000000000000;
r39t7_look_rom[365]=69'h000000000000000000;
r39t7_look_rom[366]=69'h000000000000000000;
r39t7_look_rom[367]=69'h000000000000000000;
r39t7_look_rom[368]=69'h000000000000000000;
r39t7_look_rom[369]=69'h000000000000000000;
r39t7_look_rom[370]=69'h000000000000000000;
r39t7_look_rom[371]=69'h000000000000000000;
r39t7_look_rom[372]=69'h000000000000000000;
r39t7_look_rom[373]=69'h000000000000000000;
r39t7_look_rom[374]=69'h000000000000000000;
r39t7_look_rom[375]=69'h000000000000000000;
r39t7_look_rom[376]=69'h000000000000000000;
r39t7_look_rom[377]=69'h000000000000000000;
r39t7_look_rom[378]=69'h000000000000000000;
r39t7_look_rom[379]=69'h000000000000000000;
r39t7_look_rom[380]=69'h000000000000000000;
r39t7_look_rom[381]=69'h000000000000000000;
r39t7_look_rom[382]=69'h000000000000000000;
r39t7_look_rom[383]=69'h000000000000000000;
r39t7_look_rom[384]=69'h000000000000000000;
r39t7_look_rom[385]=69'h000000000000000000;
r39t7_look_rom[386]=69'h000000000000000000;
r39t7_look_rom[387]=69'h000000000000000000;
r39t7_look_rom[388]=69'h000000000000000000;
r39t7_look_rom[389]=69'h000000000000000000;
r39t7_look_rom[390]=69'h000000000000000000;
r39t7_look_rom[391]=69'h000000000000000000;
r39t7_look_rom[392]=69'h000000000000000000;
r39t7_look_rom[393]=69'h000000000000000000;
r39t7_look_rom[394]=69'h000000000000000000;
r39t7_look_rom[395]=69'h000000000000000000;
r39t7_look_rom[396]=69'h000000000000000000;
r39t7_look_rom[397]=69'h000000000000000000;
r39t7_look_rom[398]=69'h000000000000000000;
r39t7_look_rom[399]=69'h000000000000000000;
r39t7_look_rom[400]=69'h000000000000000000;
r39t7_look_rom[401]=69'h000000000000000000;
r39t7_look_rom[402]=69'h000000000000000000;
r39t7_look_rom[403]=69'h000000000000000000;
r39t7_look_rom[404]=69'h000000000000000000;
r39t7_look_rom[405]=69'h000000000000000000;
r39t7_look_rom[406]=69'h000000000000000000;
r39t7_look_rom[407]=69'h000000000000000000;
r39t7_look_rom[408]=69'h000000000000000000;
r39t7_look_rom[409]=69'h000000000000000000;
r39t7_look_rom[410]=69'h000000000000000000;
r39t7_look_rom[411]=69'h000000000000000000;
r39t7_look_rom[412]=69'h000000000000000000;
r39t7_look_rom[413]=69'h000000000000000000;
r39t7_look_rom[414]=69'h000000000000000000;
r39t7_look_rom[415]=69'h000000000000000000;
r39t7_look_rom[416]=69'h000000000000000000;
r39t7_look_rom[417]=69'h000000000000000000;
r39t7_look_rom[418]=69'h000000000000000000;
r39t7_look_rom[419]=69'h000000000000000000;
r39t7_look_rom[420]=69'h000000000000000000;
r39t7_look_rom[421]=69'h000000000000000000;
r39t7_look_rom[422]=69'h000000000000000000;
r39t7_look_rom[423]=69'h000000000000000000;
r39t7_look_rom[424]=69'h000000000000000000;
r39t7_look_rom[425]=69'h000000000000000000;
r39t7_look_rom[426]=69'h000000000000000000;
r39t7_look_rom[427]=69'h000000000000000000;
r39t7_look_rom[428]=69'h000000000000000000;
r39t7_look_rom[429]=69'h000000000000000000;
r39t7_look_rom[430]=69'h000000000000000000;
r39t7_look_rom[431]=69'h000000000000000000;
r39t7_look_rom[432]=69'h000000000000000000;
r39t7_look_rom[433]=69'h000000000000000000;
r39t7_look_rom[434]=69'h000000000000000000;
r39t7_look_rom[435]=69'h000000000000000000;
r39t7_look_rom[436]=69'h000000000000000000;
r39t7_look_rom[437]=69'h000000000000000000;
r39t7_look_rom[438]=69'h000000000000000000;
r39t7_look_rom[439]=69'h000000000000000000;
r39t7_look_rom[440]=69'h000000000000000000;
r39t7_look_rom[441]=69'h000000000000000000;
r39t7_look_rom[442]=69'h000000000000000000;
r39t7_look_rom[443]=69'h000000000000000000;
r39t7_look_rom[444]=69'h000000000000000000;
r39t7_look_rom[445]=69'h000000000000000000;
r39t7_look_rom[446]=69'h000000000000000000;
r39t7_look_rom[447]=69'h000000000000000000;
r39t7_look_rom[448]=69'h000000000000000000;
r39t7_look_rom[449]=69'h000000000000000000;
r39t7_look_rom[450]=69'h000000000000000000;
r39t7_look_rom[451]=69'h000000000000000000;
r39t7_look_rom[452]=69'h000000000000000000;
r39t7_look_rom[453]=69'h000000000000000000;
r39t7_look_rom[454]=69'h000000000000000000;
r39t7_look_rom[455]=69'h000000000000000000;
r39t7_look_rom[456]=69'h000000000000000000;
r39t7_look_rom[457]=69'h000000000000000000;
r39t7_look_rom[458]=69'h000000000000000000;
r39t7_look_rom[459]=69'h000000000000000000;
r39t7_look_rom[460]=69'h000000000000000000;
r39t7_look_rom[461]=69'h000000000000000000;
r39t7_look_rom[462]=69'h000000000000000000;
r39t7_look_rom[463]=69'h000000000000000000;
r39t7_look_rom[464]=69'h000000000000000000;
r39t7_look_rom[465]=69'h000000000000000000;
r39t7_look_rom[466]=69'h000000000000000000;
r39t7_look_rom[467]=69'h000000000000000000;
r39t7_look_rom[468]=69'h000000000000000000;
r39t7_look_rom[469]=69'h000000000000000000;
r39t7_look_rom[470]=69'h000000000000000000;
r39t7_look_rom[471]=69'h000000000000000000;
r39t7_look_rom[472]=69'h000000000000000000;
r39t7_look_rom[473]=69'h000000000000000000;
r39t7_look_rom[474]=69'h000000000000000000;
r39t7_look_rom[475]=69'h000000000000000000;
r39t7_look_rom[476]=69'h000000000000000000;
r39t7_look_rom[477]=69'h000000000000000000;
r39t7_look_rom[478]=69'h000000000000000000;
r39t7_look_rom[479]=69'h000000000000000000;
r39t7_look_rom[480]=69'h000000000000000000;
r39t7_look_rom[481]=69'h000000000000000000;
r39t7_look_rom[482]=69'h000000000000000000;
r39t7_look_rom[483]=69'h000000000000000000;
r39t7_look_rom[484]=69'h000000000000000000;
r39t7_look_rom[485]=69'h000000000000000000;
r39t7_look_rom[486]=69'h000000000000000000;
r39t7_look_rom[487]=69'h000000000000000000;
r39t7_look_rom[488]=69'h000000000000000000;
r39t7_look_rom[489]=69'h000000000000000000;
r39t7_look_rom[490]=69'h000000000000000000;
r39t7_look_rom[491]=69'h000000000000000000;
r39t7_look_rom[492]=69'h000000000000000000;
r39t7_look_rom[493]=69'h000000000000000000;
r39t7_look_rom[494]=69'h000000000000000000;
r39t7_look_rom[495]=69'h000000000000000000;
r39t7_look_rom[496]=69'h000000000000000000;
r39t7_look_rom[497]=69'h000000000000000000;
r39t7_look_rom[498]=69'h000000000000000000;
r39t7_look_rom[499]=69'h000000000000000000;
r39t7_look_rom[500]=69'h000000000000000000;
r39t7_look_rom[501]=69'h000000000000000000;
r39t7_look_rom[502]=69'h000000000000000000;
r39t7_look_rom[503]=69'h000000000000000000;
r39t7_look_rom[504]=69'h000000000000000000;
r39t7_look_rom[505]=69'h000000000000000000;
r39t7_look_rom[506]=69'h000000000000000000;
r39t7_look_rom[507]=69'h000000000000000000;
r39t7_look_rom[508]=69'h000000000000000000;
r39t7_look_rom[509]=69'h000000000000000000;
r39t7_look_rom[510]=69'h000000000000000000;
r39t7_look_rom[511]=69'h000000000000000000;
r39t7_look_rom[512]=69'h000000000000000000;
r39t7_look_rom[513]=69'h000000000000400000;
r39t7_look_rom[514]=69'h000000000000500000;
r39t7_look_rom[515]=69'h000000000000600000;
r39t7_look_rom[516]=69'h000000000000000000;
r39t7_look_rom[517]=69'h000000000000700000;
r39t7_look_rom[518]=69'h000000000000800000;
r39t7_look_rom[519]=69'h000000000000000000;
r39t7_look_rom[520]=69'h000000040000001408;
r39t7_look_rom[521]=69'h000000000000008000;
r39t7_look_rom[522]=69'h000000000100000000;
r39t7_look_rom[523]=69'h000000000000030000;
r39t7_look_rom[524]=69'h00200000000000081b;
r39t7_look_rom[525]=69'h000000000000008000;
r39t7_look_rom[526]=69'h000000000100040000;
r39t7_look_rom[527]=69'h00400000000000081c;
r39t7_look_rom[528]=69'h000000000000008000;
r39t7_look_rom[529]=69'h000000000100040000;
r39t7_look_rom[530]=69'h00800000000000081d;
r39t7_look_rom[531]=69'h000000000000008000;
r39t7_look_rom[532]=69'h000000000100040000;
r39t7_look_rom[533]=69'h01000000000000081e;
r39t7_look_rom[534]=69'h000000000000008000;
r39t7_look_rom[535]=69'h000000000100040000;
r39t7_look_rom[536]=69'h02000000000000081f;
r39t7_look_rom[537]=69'h000000000000008000;
r39t7_look_rom[538]=69'h000000000100040000;
r39t7_look_rom[539]=69'h040000000000000820;
r39t7_look_rom[540]=69'h000000000000008000;
r39t7_look_rom[541]=69'h000000000100040000;
r39t7_look_rom[542]=69'h080000000000000821;
r39t7_look_rom[543]=69'h000000000000008000;
r39t7_look_rom[544]=69'h000000000100040000;
r39t7_look_rom[545]=69'h000000040000000808;
r39t7_look_rom[546]=69'h000000000000008000;
r39t7_look_rom[547]=69'h000000000100040000;
r39t7_look_rom[548]=69'h000400000000000818;
r39t7_look_rom[549]=69'h000000000000008000;
r39t7_look_rom[550]=69'h000000000100040000;
r39t7_look_rom[551]=69'h000800000000000819;
r39t7_look_rom[552]=69'h000000000000008000;
r39t7_look_rom[553]=69'h000000000100040000;
r39t7_look_rom[554]=69'h000004000000000810;
r39t7_look_rom[555]=69'h000000000000008000;
r39t7_look_rom[556]=69'h000000000100040000;
r39t7_look_rom[557]=69'h000000000004000000;
r39t7_look_rom[558]=69'h000000000200030000;
r39t7_look_rom[559]=69'h00200000000000081b;
r39t7_look_rom[560]=69'h000000000000008000;
r39t7_look_rom[561]=69'h000000000100040000;
r39t7_look_rom[562]=69'h00400000000000081c;
r39t7_look_rom[563]=69'h000000000000008000;
r39t7_look_rom[564]=69'h000000000100040000;
r39t7_look_rom[565]=69'h00800000000000081d;
r39t7_look_rom[566]=69'h000000000000008000;
r39t7_look_rom[567]=69'h000000000100040000;
r39t7_look_rom[568]=69'h01000000000000081e;
r39t7_look_rom[569]=69'h000000000000008000;
r39t7_look_rom[570]=69'h000000000100040000;
r39t7_look_rom[571]=69'h02000000000000081f;
r39t7_look_rom[572]=69'h000000000000008000;
r39t7_look_rom[573]=69'h000000000100040000;
r39t7_look_rom[574]=69'h040000000000000820;
r39t7_look_rom[575]=69'h000000000000008000;
r39t7_look_rom[576]=69'h000000000100040000;
r39t7_look_rom[577]=69'h080000000000000821;
r39t7_look_rom[578]=69'h000000000000008000;
r39t7_look_rom[579]=69'h000000000100040000;
r39t7_look_rom[580]=69'h000000040000000808;
r39t7_look_rom[581]=69'h000000000000008000;
r39t7_look_rom[582]=69'h000000000100040000;
r39t7_look_rom[583]=69'h000400000000000818;
r39t7_look_rom[584]=69'h000000000000008000;
r39t7_look_rom[585]=69'h000000000100040000;
r39t7_look_rom[586]=69'h000800000000000819;
r39t7_look_rom[587]=69'h000000000000008000;
r39t7_look_rom[588]=69'h000000000100040000;
r39t7_look_rom[589]=69'h000004000000000810;
r39t7_look_rom[590]=69'h000000000000008000;
r39t7_look_rom[591]=69'h000000000100040000;
r39t7_look_rom[592]=69'h000000000008000000;
r39t7_look_rom[593]=69'h000000000200030000;
r39t7_look_rom[594]=69'h00200000000000081b;
r39t7_look_rom[595]=69'h000000000000008000;
r39t7_look_rom[596]=69'h000000000100040000;
r39t7_look_rom[597]=69'h00400000000000081c;
r39t7_look_rom[598]=69'h000000000000008000;
r39t7_look_rom[599]=69'h000000000100040000;
r39t7_look_rom[600]=69'h00800000000000081d;
r39t7_look_rom[601]=69'h000000000000008000;
r39t7_look_rom[602]=69'h000000000100040000;
r39t7_look_rom[603]=69'h01000000000000081e;
r39t7_look_rom[604]=69'h000000000000008000;
r39t7_look_rom[605]=69'h000000000100040000;
r39t7_look_rom[606]=69'h02000000000000081f;
r39t7_look_rom[607]=69'h000000000000008000;
r39t7_look_rom[608]=69'h000000000100040000;
r39t7_look_rom[609]=69'h040000000000000820;
r39t7_look_rom[610]=69'h000000000000008000;
r39t7_look_rom[611]=69'h000000000100040000;
r39t7_look_rom[612]=69'h080000000000000821;
r39t7_look_rom[613]=69'h000000000000008000;
r39t7_look_rom[614]=69'h000000000100040000;
r39t7_look_rom[615]=69'h000000040000000808;
r39t7_look_rom[616]=69'h000000000000008000;
r39t7_look_rom[617]=69'h000000000100040000;
r39t7_look_rom[618]=69'h000400000000000818;
r39t7_look_rom[619]=69'h000000000000008000;
r39t7_look_rom[620]=69'h000000000100040000;
r39t7_look_rom[621]=69'h000800000000000819;
r39t7_look_rom[622]=69'h000000000000008000;
r39t7_look_rom[623]=69'h000000000100040000;
r39t7_look_rom[624]=69'h000004000000000810;
r39t7_look_rom[625]=69'h000000000000008000;
r39t7_look_rom[626]=69'h000000000100040000;
r39t7_look_rom[627]=69'h00000000000c000000;
r39t7_look_rom[628]=69'h000000000200030000;
r39t7_look_rom[629]=69'h00200000000000081b;
r39t7_look_rom[630]=69'h000000000000008000;
r39t7_look_rom[631]=69'h000000000100040000;
r39t7_look_rom[632]=69'h00400000000000081c;
r39t7_look_rom[633]=69'h000000000000008000;
r39t7_look_rom[634]=69'h000000000100040000;
r39t7_look_rom[635]=69'h00800000000000081d;
r39t7_look_rom[636]=69'h000000000000008000;
r39t7_look_rom[637]=69'h000000000100040000;
r39t7_look_rom[638]=69'h01000000000000081e;
r39t7_look_rom[639]=69'h000000000000008000;
r39t7_look_rom[640]=69'h000000000100040000;
r39t7_look_rom[641]=69'h02000000000000081f;
r39t7_look_rom[642]=69'h000000000000008000;
r39t7_look_rom[643]=69'h000000000100040000;
r39t7_look_rom[644]=69'h040000000000000820;
r39t7_look_rom[645]=69'h000000000000008000;
r39t7_look_rom[646]=69'h000000000100040000;
r39t7_look_rom[647]=69'h080000000000000821;
r39t7_look_rom[648]=69'h000000000000008000;
r39t7_look_rom[649]=69'h000000000100040000;
r39t7_look_rom[650]=69'h000000040000000808;
r39t7_look_rom[651]=69'h000000000000008000;
r39t7_look_rom[652]=69'h000000000100040000;
r39t7_look_rom[653]=69'h000400000000000818;
r39t7_look_rom[654]=69'h000000000000008000;
r39t7_look_rom[655]=69'h000000000100040000;
r39t7_look_rom[656]=69'h000800000000000819;
r39t7_look_rom[657]=69'h000000000000008000;
r39t7_look_rom[658]=69'h000000000100040000;
r39t7_look_rom[659]=69'h000004000000000810;
r39t7_look_rom[660]=69'h000000000000008000;
r39t7_look_rom[661]=69'h000000000100040000;
r39t7_look_rom[662]=69'h000000000010000000;
r39t7_look_rom[663]=69'h000000000200030000;
r39t7_look_rom[664]=69'h00200000000000081b;
r39t7_look_rom[665]=69'h000000000000008000;
r39t7_look_rom[666]=69'h000000000100040000;
r39t7_look_rom[667]=69'h00400000000000081c;
r39t7_look_rom[668]=69'h000000000000008000;
r39t7_look_rom[669]=69'h000000000100040000;
r39t7_look_rom[670]=69'h00800000000000081d;
r39t7_look_rom[671]=69'h000000000000008000;
r39t7_look_rom[672]=69'h000000000100040000;
r39t7_look_rom[673]=69'h01000000000000081e;
r39t7_look_rom[674]=69'h000000000000008000;
r39t7_look_rom[675]=69'h000000000100040000;
r39t7_look_rom[676]=69'h02000000000000081f;
r39t7_look_rom[677]=69'h000000000000008000;
r39t7_look_rom[678]=69'h000000000100040000;
r39t7_look_rom[679]=69'h040000000000000820;
r39t7_look_rom[680]=69'h000000000000008000;
r39t7_look_rom[681]=69'h000000000100040000;
r39t7_look_rom[682]=69'h080000000000000821;
r39t7_look_rom[683]=69'h000000000000008000;
r39t7_look_rom[684]=69'h000000000100040000;
r39t7_look_rom[685]=69'h000000040000000808;
r39t7_look_rom[686]=69'h000000000000008000;
r39t7_look_rom[687]=69'h000000000100040000;
r39t7_look_rom[688]=69'h000400000000000818;
r39t7_look_rom[689]=69'h000000000000008000;
r39t7_look_rom[690]=69'h000000000100040000;
r39t7_look_rom[691]=69'h000800000000000819;
r39t7_look_rom[692]=69'h000000000000008000;
r39t7_look_rom[693]=69'h000000000100040000;
r39t7_look_rom[694]=69'h000004000000000810;
r39t7_look_rom[695]=69'h000000000000008000;
r39t7_look_rom[696]=69'h000000000100040000;
r39t7_look_rom[697]=69'h000000000014000000;
r39t7_look_rom[698]=69'h000000000200030000;
r39t7_look_rom[699]=69'h00200000000000081b;
r39t7_look_rom[700]=69'h000000000000008000;
r39t7_look_rom[701]=69'h000000000100040000;
r39t7_look_rom[702]=69'h00400000000000081c;
r39t7_look_rom[703]=69'h000000000000008000;
r39t7_look_rom[704]=69'h000000000100040000;
r39t7_look_rom[705]=69'h00800000000000081d;
r39t7_look_rom[706]=69'h000000000000008000;
r39t7_look_rom[707]=69'h000000000100040000;
r39t7_look_rom[708]=69'h01000000000000081e;
r39t7_look_rom[709]=69'h000000000000008000;
r39t7_look_rom[710]=69'h000000000100040000;
r39t7_look_rom[711]=69'h02000000000000081f;
r39t7_look_rom[712]=69'h000000000000008000;
r39t7_look_rom[713]=69'h000000000100040000;
r39t7_look_rom[714]=69'h040000000000000820;
r39t7_look_rom[715]=69'h000000000000008000;
r39t7_look_rom[716]=69'h000000000100040000;
r39t7_look_rom[717]=69'h080000000000000821;
r39t7_look_rom[718]=69'h000000000000008000;
r39t7_look_rom[719]=69'h000000000100040000;
r39t7_look_rom[720]=69'h000000040000000808;
r39t7_look_rom[721]=69'h000000000000008000;
r39t7_look_rom[722]=69'h000000000100040000;
r39t7_look_rom[723]=69'h000400000000000818;
r39t7_look_rom[724]=69'h000000000000008000;
r39t7_look_rom[725]=69'h000000000100040000;
r39t7_look_rom[726]=69'h000800000000000819;
r39t7_look_rom[727]=69'h000000000000008000;
r39t7_look_rom[728]=69'h000000000100040000;
r39t7_look_rom[729]=69'h000004000000000810;
r39t7_look_rom[730]=69'h000000000000008000;
r39t7_look_rom[731]=69'h000000000100040000;
r39t7_look_rom[732]=69'h000000000018000000;
r39t7_look_rom[733]=69'h000000000200030000;
r39t7_look_rom[734]=69'h00200000000000081b;
r39t7_look_rom[735]=69'h000000000000008000;
r39t7_look_rom[736]=69'h000000000100040000;
r39t7_look_rom[737]=69'h00400000000000081c;
r39t7_look_rom[738]=69'h000000000000008000;
r39t7_look_rom[739]=69'h000000000100040000;
r39t7_look_rom[740]=69'h00800000000000081d;
r39t7_look_rom[741]=69'h000000000000008000;
r39t7_look_rom[742]=69'h000000000100040000;
r39t7_look_rom[743]=69'h01000000000000081e;
r39t7_look_rom[744]=69'h000000000000008000;
r39t7_look_rom[745]=69'h000000000100040000;
r39t7_look_rom[746]=69'h02000000000000081f;
r39t7_look_rom[747]=69'h000000000000008000;
r39t7_look_rom[748]=69'h000000000100040000;
r39t7_look_rom[749]=69'h040000000000000820;
r39t7_look_rom[750]=69'h000000000000008000;
r39t7_look_rom[751]=69'h000000000100040000;
r39t7_look_rom[752]=69'h080000000000000821;
r39t7_look_rom[753]=69'h000000000000008000;
r39t7_look_rom[754]=69'h000000000100040000;
r39t7_look_rom[755]=69'h000000040000000808;
r39t7_look_rom[756]=69'h000000000000008000;
r39t7_look_rom[757]=69'h000000000100040000;
r39t7_look_rom[758]=69'h000400000000000818;
r39t7_look_rom[759]=69'h000000000000008000;
r39t7_look_rom[760]=69'h000000000100040000;
r39t7_look_rom[761]=69'h000800000000000819;
r39t7_look_rom[762]=69'h000000000000008000;
r39t7_look_rom[763]=69'h000000000100040000;
r39t7_look_rom[764]=69'h000004000000000810;
r39t7_look_rom[765]=69'h000000000000008000;
r39t7_look_rom[766]=69'h000000000100040000;
r39t7_look_rom[767]=69'h00000000001c090000;
r39t7_look_rom[768]=69'h00200000020000081b;
r39t7_look_rom[769]=69'h000000000000420000;
r39t7_look_rom[770]=69'h000000000020505000;
r39t7_look_rom[771]=69'h000000000000660000;
r39t7_look_rom[772]=69'h000000000000050000;
r39t7_look_rom[773]=69'h000000000000700000;
r39t7_look_rom[774]=69'h000000000003000000;
r39t7_look_rom[775]=69'h000000000000000000;
r39t7_look_rom[776]=69'h000040000000003a14;
r39t7_look_rom[777]=69'h000000000000008000;
r39t7_look_rom[778]=69'h000000000100000000;
r39t7_look_rom[779]=69'h0000001000000041ca;
r39t7_look_rom[780]=69'h000000000000008000;
r39t7_look_rom[781]=69'h000000000100000000;
r39t7_look_rom[782]=69'h000000000000000000;
r39t7_look_rom[783]=69'h000000000000070000;
r39t7_look_rom[784]=69'h00000020000000154b;
r39t7_look_rom[785]=69'h000000000000008000;
r39t7_look_rom[786]=69'h000000000100040000;
r39t7_look_rom[787]=69'h0000800000000033d5;
r39t7_look_rom[788]=69'h000000000000008000;
r39t7_look_rom[789]=69'h000000000100040000;
r39t7_look_rom[790]=69'h000010000000003392;
r39t7_look_rom[791]=69'h000000000000008000;
r39t7_look_rom[792]=69'h000000000100040000;
r39t7_look_rom[793]=69'h000020000000003353;
r39t7_look_rom[794]=69'h000000000000008000;
r39t7_look_rom[795]=69'h000000000100040000;
r39t7_look_rom[796]=69'h00000080000000330d;
r39t7_look_rom[797]=69'h000000000000008000;
r39t7_look_rom[798]=69'h000000000100040000;
r39t7_look_rom[799]=69'h00000100000000328e;
r39t7_look_rom[800]=69'h000000000000008000;
r39t7_look_rom[801]=69'h000000000100040000;
r39t7_look_rom[802]=69'h0000020000000032cf;
r39t7_look_rom[803]=69'h000000000000008000;
r39t7_look_rom[804]=69'h000000000100040000;
r39t7_look_rom[805]=69'h000200000000000817;
r39t7_look_rom[806]=69'h000000000000008000;
r39t7_look_rom[807]=69'h000000000100040000;
r39t7_look_rom[808]=69'h000000080000000809;
r39t7_look_rom[809]=69'h000000000000008000;
r39t7_look_rom[810]=69'h000000000100040000;
r39t7_look_rom[811]=69'h001000000000004c9a;
r39t7_look_rom[812]=69'h000000000000008000;
r39t7_look_rom[813]=69'h000000000100040000;
r39t7_look_rom[814]=69'h000000000000000000;
r39t7_look_rom[815]=69'h0001000000000014d6;
r39t7_look_rom[816]=69'h000000000000008000;
r39t7_look_rom[817]=69'h000000000100000000;
r39t7_look_rom[818]=69'h000000000000000000;
r39t7_look_rom[819]=69'h000000000000000000;
r39t7_look_rom[820]=69'h000000000000000000;
r39t7_look_rom[821]=69'h000000000000000000;
r39t7_look_rom[822]=69'h000000000000000000;
r39t7_look_rom[823]=69'h000000000000000000;
r39t7_look_rom[824]=69'h000000010000000806;
r39t7_look_rom[825]=69'h000000000000008000;
r39t7_look_rom[826]=69'h000000000140000000;
r39t7_look_rom[827]=69'h000000000000100000;
r39t7_look_rom[828]=69'h000000000000400000;
r39t7_look_rom[829]=69'h000000000000500000;
r39t7_look_rom[830]=69'h000000000000600000;
r39t7_look_rom[831]=69'h000000000000000000;
r39t7_look_rom[832]=69'h000000000000700000;
r39t7_look_rom[833]=69'h000000000002000000;
r39t7_look_rom[834]=69'h000000000000000000;
r39t7_look_rom[835]=69'h000008000000001a11;
r39t7_look_rom[836]=69'h000000000000008000;
r39t7_look_rom[837]=69'h000000000100000000;
r39t7_look_rom[838]=69'h0000001000000059ca;
r39t7_look_rom[839]=69'h000000000000008000;
r39t7_look_rom[840]=69'h000000000100000000;
r39t7_look_rom[841]=69'h000000000000000000;
r39t7_look_rom[842]=69'h000000000000030000;
r39t7_look_rom[843]=69'h00000020000000154b;
r39t7_look_rom[844]=69'h000000000000008000;
r39t7_look_rom[845]=69'h000000000100040000;
r39t7_look_rom[846]=69'h00000040000000218c;
r39t7_look_rom[847]=69'h000000000000008000;
r39t7_look_rom[848]=69'h000000000100040000;
r39t7_look_rom[849]=69'h000000004000002144;
r39t7_look_rom[850]=69'h000000000000008000;
r39t7_look_rom[851]=69'h000000000100040000;
r39t7_look_rom[852]=69'h000000002000002103;
r39t7_look_rom[853]=69'h000000000000008000;
r39t7_look_rom[854]=69'h000000000100040000;
r39t7_look_rom[855]=69'h0000008000000020cd;
r39t7_look_rom[856]=69'h000000000000008000;
r39t7_look_rom[857]=69'h000000000100040000;
r39t7_look_rom[858]=69'h00000100000000204e;
r39t7_look_rom[859]=69'h000000000000008000;
r39t7_look_rom[860]=69'h000000000100040000;
r39t7_look_rom[861]=69'h00000200000000208f;
r39t7_look_rom[862]=69'h000000000000008000;
r39t7_look_rom[863]=69'h000000000100040000;
r39t7_look_rom[864]=69'h000200000000000817;
r39t7_look_rom[865]=69'h000000000000008000;
r39t7_look_rom[866]=69'h000000000100040000;
r39t7_look_rom[867]=69'h000000080000000809;
r39t7_look_rom[868]=69'h000000000000008000;
r39t7_look_rom[869]=69'h000000000100040000;
r39t7_look_rom[870]=69'h000000008000002d05;
r39t7_look_rom[871]=69'h000000000000008000;
r39t7_look_rom[872]=69'h000000000100040000;
r39t7_look_rom[873]=69'h000000001000000802;
r39t7_look_rom[874]=69'h000000000000008000;
r39t7_look_rom[875]=69'h000000000100040000;
r39t7_look_rom[876]=69'h000000000000000000;
r39t7_look_rom[877]=69'h0001000003000014d6;
r39t7_look_rom[878]=69'h000000000000000000;
r39t7_look_rom[879]=69'h000000000000000000;
r39t7_look_rom[880]=69'h000000000000000000;
r39t7_look_rom[881]=69'h000000000000000000;
r39t7_look_rom[882]=69'h000000000000000000;
r39t7_look_rom[883]=69'h0001000000000014d6;
r39t7_look_rom[884]=69'h000000000000008000;
r39t7_look_rom[885]=69'h000000000100000000;
r39t7_look_rom[886]=69'h000000000000000000;
r39t7_look_rom[887]=69'h000000000000000000;
r39t7_look_rom[888]=69'h000000000000000000;
r39t7_look_rom[889]=69'h000000000000000000;
r39t7_look_rom[890]=69'h000000000000000000;
r39t7_look_rom[891]=69'h000000000000000000;
r39t7_look_rom[892]=69'h000000010000000806;
r39t7_look_rom[893]=69'h000000000000008000;
r39t7_look_rom[894]=69'h000000000180000000;
r39t7_look_rom[895]=69'h000000000000000000;
r39t7_look_rom[896]=69'h000000000000000000;
r39t7_look_rom[897]=69'h000000000000000000;
r39t7_look_rom[898]=69'h000000000000008000;
r39t7_look_rom[899]=69'h000000000100000000;
r39t7_look_rom[900]=69'h000000000000000000;
r39t7_look_rom[901]=69'h000000000000000000;
r39t7_look_rom[902]=69'h000000000000000000;
r39t7_look_rom[903]=69'h000000000000000000;
r39t7_look_rom[904]=69'h000000000000200000;
r39t7_look_rom[905]=69'h000000000000000000;
r39t7_look_rom[906]=69'h000000000000000000;
r39t7_look_rom[907]=69'h000000000000000000;
r39t7_look_rom[908]=69'h000000000000000000;
r39t7_look_rom[909]=69'h000000000000000000;
r39t7_look_rom[910]=69'h000000000000000000;
r39t7_look_rom[911]=69'h000000000000000000;
r39t7_look_rom[912]=69'h000000000000000000;
r39t7_look_rom[913]=69'h000000000000000000;
r39t7_look_rom[914]=69'h000000000000000000;
r39t7_look_rom[915]=69'h000000000000000000;
r39t7_look_rom[916]=69'h000000000000000000;
r39t7_look_rom[917]=69'h000000000000000000;
r39t7_look_rom[918]=69'h000000000000000000;
r39t7_look_rom[919]=69'h000000000000000000;
r39t7_look_rom[920]=69'h000000000000000000;
r39t7_look_rom[921]=69'h000000000000000000;
r39t7_look_rom[922]=69'h000000000000000000;
r39t7_look_rom[923]=69'h000000000000000000;
r39t7_look_rom[924]=69'h000000000000000000;
r39t7_look_rom[925]=69'h000000000000000000;
r39t7_look_rom[926]=69'h000000000000008000;
r39t7_look_rom[927]=69'h000000000100040000;
r39t7_look_rom[928]=69'h000000000000000000;
r39t7_look_rom[929]=69'h000000000400000000;
r39t7_look_rom[930]=69'h000000000000000000;
r39t7_look_rom[931]=69'h000000000000000000;
r39t7_look_rom[932]=69'h000000000000000000;
r39t7_look_rom[933]=69'h000000000000000000;
r39t7_look_rom[934]=69'h000000000000000000;
r39t7_look_rom[935]=69'h000000000000000000;
r39t7_look_rom[936]=69'h000000000000000000;
r39t7_look_rom[937]=69'h000000000000000000;
r39t7_look_rom[938]=69'h000000000000000000;
r39t7_look_rom[939]=69'h000000000000000000;
r39t7_look_rom[940]=69'h000000000000000000;
r39t7_look_rom[941]=69'h000000000000000000;
r39t7_look_rom[942]=69'h000000000000008000;
r39t7_look_rom[943]=69'h000000000100040000;
r39t7_look_rom[944]=69'h00400000000000081c;
r39t7_look_rom[945]=69'h000000000000008000;
r39t7_look_rom[946]=69'h000000000100040000;
r39t7_look_rom[947]=69'h00800000000000081d;
r39t7_look_rom[948]=69'h000000000000008000;
r39t7_look_rom[949]=69'h000000000100040000;
r39t7_look_rom[950]=69'h01000000000000081e;
r39t7_look_rom[951]=69'h000000000000008000;
r39t7_look_rom[952]=69'h000000000100040000;
r39t7_look_rom[953]=69'h02000000000000081f;
r39t7_look_rom[954]=69'h000000000000008000;
r39t7_look_rom[955]=69'h000000000100040000;
r39t7_look_rom[956]=69'h040000000000000820;
r39t7_look_rom[957]=69'h000000000000008000;
r39t7_look_rom[958]=69'h000000000100040000;
r39t7_look_rom[959]=69'h080000000000000821;
r39t7_look_rom[960]=69'h000000000000008000;
r39t7_look_rom[961]=69'h000000000100040000;
r39t7_look_rom[962]=69'h000000000800000801;
r39t7_look_rom[963]=69'h000000000000008000;
r39t7_look_rom[964]=69'h000000000100040000;
r39t7_look_rom[965]=69'h000000040000000808;
r39t7_look_rom[966]=69'h000000000000008000;
r39t7_look_rom[967]=69'h000000000100040000;
r39t7_look_rom[968]=69'h000400000000000818;
r39t7_look_rom[969]=69'h000000000000008000;
r39t7_look_rom[970]=69'h000000000100040000;
r39t7_look_rom[971]=69'h000800000000000819;
r39t7_look_rom[972]=69'h000000000000008000;
r39t7_look_rom[973]=69'h000000000100040000;
r39t7_look_rom[974]=69'h000004000000000810;
r39t7_look_rom[975]=69'h000000000000008000;
r39t7_look_rom[976]=69'h000000000100040000;
r39t7_look_rom[977]=69'h000000000000000000;
r39t7_look_rom[978]=69'h000000000000100000;
r39t7_look_rom[979]=69'h000000000000000000;
r39t7_look_rom[980]=69'h000000000000000000;
r39t7_look_rom[981]=69'h000000000000000000;
r39t7_look_rom[982]=69'h000000000000000000;
r39t7_look_rom[983]=69'h000000000000000000;
r39t7_look_rom[984]=69'h000000000000000000;
r39t7_look_rom[985]=69'h000000000000000000;
r39t7_look_rom[986]=69'h000000000000000000;
r39t7_look_rom[987]=69'h000000000000000000;
r39t7_look_rom[988]=69'h000000000000000000;
r39t7_look_rom[989]=69'h000000000000000000;
r39t7_look_rom[990]=69'h000000000000000000;
r39t7_look_rom[991]=69'h000000000000000000;
r39t7_look_rom[992]=69'h000000000000000000;
r39t7_look_rom[993]=69'h000000000000000000;
r39t7_look_rom[994]=69'h000000000000000000;
r39t7_look_rom[995]=69'h000000000000000000;
r39t7_look_rom[996]=69'h000000000000000000;
r39t7_look_rom[997]=69'h000000000000000000;
r39t7_look_rom[998]=69'h000000000000000000;
r39t7_look_rom[999]=69'h000000000000000000;
r39t7_look_rom[1000]=69'h000000000000000000;
r39t7_look_rom[1001]=69'h000000000000000000;
r39t7_look_rom[1002]=69'h000000000000000000;
r39t7_look_rom[1003]=69'h000000000000000000;
r39t7_look_rom[1004]=69'h000000000000000000;
r39t7_look_rom[1005]=69'h000000000000000000;
r39t7_look_rom[1006]=69'h000000000000000000;
r39t7_look_rom[1007]=69'h000000000000000000;
r39t7_look_rom[1008]=69'h000000000000000000;
r39t7_look_rom[1009]=69'h000000000000000000;
r39t7_look_rom[1010]=69'h000000000000000000;
r39t7_look_rom[1011]=69'h000000000000000000;
r39t7_look_rom[1012]=69'h000000000000000000;
r39t7_look_rom[1013]=69'h000000000000000000;
r39t7_look_rom[1014]=69'h000000000000000000;
r39t7_look_rom[1015]=69'h000000000000000000;
r39t7_look_rom[1016]=69'h000000000000000000;
r39t7_look_rom[1017]=69'h000000000000000000;
r39t7_look_rom[1018]=69'h000000000000000000;
r39t7_look_rom[1019]=69'h000000000000000000;
r39t7_look_rom[1020]=69'h000000000000000000;
r39t7_look_rom[1021]=69'h000000000000000000;
r39t7_look_rom[1022]=69'h000000000000000000;
r39t7_look_rom[1023]=69'h000000000000000000;
end
always @(posedge clk)begin
 if(busy)r39t7_look_q<=r39t7_look_rom[{r39_mode,state}];
 if(!rst_n||clear)r39t7_override_q<=1;
 else if(!busy)begin
  if(start&&!fault)r39t7_override_q<=(ext2_warm===1'b1)?1:2;
 end else begin
  r39t7_override_q<=0;
  r39t7_wait_q<=multiply_wait;r39t7_added_q<=r39_matrix_added;
  r39t7_shape_q<=shape_enabled||sine_ff_enabled;r39t7_auto_q<=automatic_s;
  r39t7_return_q<=r39t7_return_op;
 end
end
wire[31:0]r39t7_return_word={3'b0,r39t7_return_q,26'b0}|
 ((r39t7_return_q==7)?32'h00090000:32'd0);
wire[2:0]r39t7_tag=r39t7_look_q[34:32];
// Parallel mutually exclusive word selection; do not infer a priority chain.
wire r39t7_run=(r39t7_override_q==0);
wire r39t7_use_base=r39t7_run&&((r39t7_tag==0)||
 ((r39t7_tag==1)&&r39t7_wait_q)||((r39t7_tag==2)&&r39t7_added_q)||
 ((r39t7_tag==3)&&!r39t7_shape_q)||((r39t7_tag==5)&&r39t7_auto_q));
wire[31:0]r39t4_control_q=(r39t7_look_q[31:0]&{32{r39t7_use_base}})|
 (32'h00010000&{32{r39t7_override_q==2}})|
 (32'h00008000&{32{r39t7_run&&(r39t7_tag==1)&&!r39t7_wait_q}})|
 (32'h00001462&{32{r39t7_run&&(r39t7_tag==2)&&!r39t7_added_q}})|
 (r39t7_return_word&{32{r39t7_run&&(r39t7_tag==4)}});


// R40T1: ROM stores decoded selects, avoiding a broadcast binary opcode.
wire r40t1_ma_special=r39t7_run&&(r39t7_tag==2)&&!r39t7_added_q;
wire[33:0] r40t1_ma_select=(r39t7_look_q[68:35]&~(34'h4000000&{34{r40t1_ma_special}})) | (34'h200000000&{34{r40t1_ma_special}});
wire r40t1_ma_write=(r39t7_use_base&&(|r39t7_look_q[68:35]))||r40t1_ma_special;
wire signed[31:0] r40t1_ma_value_1=(DISTURBANCE_OBSERVER&&!1'b0) ? z7 : 32'sd0;
wire signed[31:0] r40t1_ma_value_2=(DISTURBANCE_OBSERVER&&DISTURBANCE_REGULATOR&&!1'b0) ? n7 : 32'sd0;
wire signed[31:0] r40t1_ma_value_3=(MANUAL_ENCODER_FEEDBACK&&MANUAL_LOW_GAIN&&!1'b0) ? feedback_velocity : n3;
wire signed[31:0] r40t1_ma_value_4=(MOTION_POSITION_BOOST&&motion_s) ? boosted_position : position_error;
wire signed[31:0] r40t1_ma_value_5=(MOTION_VELOCITY_TRIM&&motion_s&&!(MANUAL_LOW_GAIN&&!1'b0)) ? velocity_s-(velocity_clip>>>3) : velocity_s;
wire signed[31:0] r40t1_ma_value_6=delivered;
wire signed[31:0] r40t1_ma_value_7=delivered-fallback_s;
wire signed[31:0] r40t1_ma_value_8=effective;
wire signed[31:0] r40t1_ma_value_9=integral;
wire signed[31:0] r40t1_ma_value_10=integral_lp;
wire signed[31:0] r40t1_ma_value_11=n0;
wire signed[31:0] r40t1_ma_value_12=n1;
wire signed[31:0] r40t1_ma_value_13=n4;
wire signed[31:0] r40t1_ma_value_14=n5;
wire signed[31:0] r40t1_ma_value_15=n6;
wire signed[31:0] r40t1_ma_value_16=phi_s;
wire signed[31:0] r40t1_ma_value_17=phi_s-reference_s-integral_lp;
wire signed[31:0] r40t1_ma_value_18=position_error;
wire signed[31:0] r40t1_ma_value_19=r36_feedback_velocity;
wire signed[31:0] r40t1_ma_value_20=r36_phi_error_s-integral_lp;
wire signed[31:0] r40t1_ma_value_21=r39t7_ma21_prepared;
wire signed[31:0] r40t1_ma_value_22=request;
wire signed[31:0] r40t1_ma_value_23=tf_new;
wire signed[31:0] r40t1_ma_value_24=tf_old;
wire signed[31:0] r40t1_ma_value_25=theta_s;
wire signed[31:0] r40t1_ma_value_26=velocity_s;
wire signed[31:0] r40t1_ma_value_27=z0;
wire signed[31:0] r40t1_ma_value_28=z1;
wire signed[31:0] r40t1_ma_value_29=z2;
wire signed[31:0] r40t1_ma_value_30=z3;
wire signed[31:0] r40t1_ma_value_31=z4;
wire signed[31:0] r40t1_ma_value_32=z5;
wire signed[31:0] r40t1_ma_value_33=z6;
wire signed[31:0] r40t1_ma_value_34=z7;
wire signed[31:0] r40t1_ma_data=
 (r40t1_ma_value_1&{32{r40t1_ma_select[0]}}) |
 (r40t1_ma_value_2&{32{r40t1_ma_select[1]}}) |
 (r40t1_ma_value_3&{32{r40t1_ma_select[2]}}) |
 (r40t1_ma_value_4&{32{r40t1_ma_select[3]}}) |
 (r40t1_ma_value_5&{32{r40t1_ma_select[4]}}) |
 (r40t1_ma_value_6&{32{r40t1_ma_select[5]}}) |
 (r40t1_ma_value_7&{32{r40t1_ma_select[6]}}) |
 (r40t1_ma_value_8&{32{r40t1_ma_select[7]}}) |
 (r40t1_ma_value_9&{32{r40t1_ma_select[8]}}) |
 (r40t1_ma_value_10&{32{r40t1_ma_select[9]}}) |
 (r40t1_ma_value_11&{32{r40t1_ma_select[10]}}) |
 (r40t1_ma_value_12&{32{r40t1_ma_select[11]}}) |
 (r40t1_ma_value_13&{32{r40t1_ma_select[12]}}) |
 (r40t1_ma_value_14&{32{r40t1_ma_select[13]}}) |
 (r40t1_ma_value_15&{32{r40t1_ma_select[14]}}) |
 (r40t1_ma_value_16&{32{r40t1_ma_select[15]}}) |
 (r40t1_ma_value_17&{32{r40t1_ma_select[16]}}) |
 (r40t1_ma_value_18&{32{r40t1_ma_select[17]}}) |
 (r40t1_ma_value_19&{32{r40t1_ma_select[18]}}) |
 (r40t1_ma_value_20&{32{r40t1_ma_select[19]}}) |
 (r40t1_ma_value_21&{32{r40t1_ma_select[20]}}) |
 (r40t1_ma_value_22&{32{r40t1_ma_select[21]}}) |
 (r40t1_ma_value_23&{32{r40t1_ma_select[22]}}) |
 (r40t1_ma_value_24&{32{r40t1_ma_select[23]}}) |
 (r40t1_ma_value_25&{32{r40t1_ma_select[24]}}) |
 (r40t1_ma_value_26&{32{r40t1_ma_select[25]}}) |
 (r40t1_ma_value_27&{32{r40t1_ma_select[26]}}) |
 (r40t1_ma_value_28&{32{r40t1_ma_select[27]}}) |
 (r40t1_ma_value_29&{32{r40t1_ma_select[28]}}) |
 (r40t1_ma_value_30&{32{r40t1_ma_select[29]}}) |
 (r40t1_ma_value_31&{32{r40t1_ma_select[30]}}) |
 (r40t1_ma_value_32&{32{r40t1_ma_select[31]}}) |
 (r40t1_ma_value_33&{32{r40t1_ma_select[32]}}) |
 (r40t1_ma_value_34&{32{r40t1_ma_select[33]}});

wire[5:0]r39t4_ma_op=r39t4_control_q[5:0];
wire[4:0]r39t4_mb_op=r39t4_control_q[10:6];
wire[3:0]r39t4_coef_op=r39t4_control_q[14:11];
wire[0:0]r39t4_mul_op=r39t4_control_q[15:15];
wire[3:0]r39t4_acc_op=r39t4_control_q[19:16];
wire[2:0]r39t4_lut_op=r39t4_control_q[22:20];
wire[1:0]r39t4_tfold_op=r39t4_control_q[24:23];
wire[0:0]r39t4_tfnew_op=r39t4_control_q[25:25];
wire[2:0]r39t4_ncommit_op=r39t4_control_q[28:26];
wire[0:0]r39t4_prodacc_op=r39t4_control_q[29:29];
wire[1:0]r39t4_outerr_op=r39t4_control_q[31:30];
reg signed[31:0]r39t4_matrix_b;
always @*begin case(r39_matrix_return)
9'd46:r39t4_matrix_b=-32'sd83392;
9'd81:r39t4_matrix_b=-32'sd521939;
9'd116:r39t4_matrix_b=-32'sd1182;
9'd151:r39t4_matrix_b=32'sd5878;
9'd186:r39t4_matrix_b=-32'sd385769;
9'd221:r39t4_matrix_b=32'sd461937;
9'd256:r39t4_matrix_b=32'sd57482;
default:r39t4_matrix_b=0;endcase end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
if(r40t1_ma_write)ma<=r40t1_ma_data;
end
end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
case(r39t4_mb_op)
5'd1:mb<=1'b0 ? (brake_s ? -32'sd3056810 : -32'sd2104864) : -32'sd2631080;
5'd2:mb<=1'b0 ? (brake_s ? -32'sd3363586 : -32'sd2148228) : -32'sd2685285;
5'd3:mb<=1'b0 ? (brake_s ? -32'sd388103 : -32'sd268429) : -32'sd335536;
5'd4:mb<=1'b0 ? (brake_s ? 32'sd45579777 : 32'sd30194271) : 32'sd37742839;
5'd5:mb<=1'b0 ? (brake_s ? 32'sd5555072 : 32'sd3568142) : 32'sd4460178;
5'd6:mb<=sine_ff_enabled ? 32'sd58130819 : 32'sd48442349;
5'd7:mb<=r40_position_unwind ? 32'sd8389 : 32'sd1048;
5'd8:mb<=32'sd133682;
5'd9:mb<=blend;
5'd10:mb<=brake_s ? -32'sd2104864 : motion_gain_5(motion_phase);
5'd11:mb<=brake_s ? -32'sd2148228 : motion_gain_6(motion_phase);
5'd12:mb<=brake_s ? -32'sd268429 : motion_gain_4(motion_phase);
5'd13:mb<=brake_s ? 32'sd30194271 : motion_gain_3(motion_phase);
5'd14:mb<=brake_s ? 32'sd3568142 : motion_gain_2(motion_phase);
5'd15:mb<=brake_s ? 32'sd38753879 : ((mature_static_s&&!motion_latched&&cap==150) ? damping_gain_1(damping_weight) : motion_gain_1(motion_phase));
5'd16:mb<=effective<0 ? 32'sd20906080 : 32'sd16777216;
5'd17:mb<=r39t4_matrix_b;
5'd18:mb<=motion_gain_7(motion_phase);
5'd19:mb<=request<0 ? 32'sd13463786 : 32'sd16777216;
5'd20:mb<=sine_ff_enabled ? -32'sd2554777 : -32'sd3523830;
5'd21:mb<=theta_gain;
default:begin end
endcase
end
end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
case(r39t4_coef_op)
4'd1:coefficient_dynamic<=0;
4'd2:coefficient_dynamic<=1;
4'd3:coefficient_dynamic<=1'b0&&drift_s;
4'd4:coefficient_dynamic<=1'b0||MANUAL_LOW_GAIN;
4'd5:coefficient_dynamic<=MANUAL_LOW_GAIN&&!1'b0;
4'd6:coefficient_dynamic<=automatic_s;
4'd7:coefficient_dynamic<=automatic_s&&drift_s;
4'd8:coefficient_dynamic<=r40_position_unwind || r41_moving_lead_i || (automatic_s&&drift_s&&!r38_endpoint_i_fast);
4'd9:coefficient_dynamic<=automatic_s&&motion_latched&&!brake_s;
4'd10:coefficient_dynamic<=r38_rate_boost;
4'd11:coefficient_dynamic<=sine_ff_enabled;
default:begin end
endcase
end
end
always @(posedge clk or negedge rst_n)begin
if(!rst_n)begin multiply_wait<=0;multiplier_b<=0;end
else if(clear)begin multiply_wait<=0;multiplier_b<=0;end
else if(busy&&r39t4_mul_op)begin
if(!multiply_wait)begin multiplier_b<=coefficient_operand;multiply_wait<=1;end
else begin product<=ma*multiplier_b;multiply_wait<=0;end
end else if(busy&&r39t4_prodacc_op)product<=acc;
end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
case(r39t4_acc_op)
4'd1:acc<=$signed({{32{phi_s[31]}},phi_s})-$signed({{32{reference_s[31]}},reference_s});
4'd2:acc<=$signed({{32{r36_omega_s[31]}},r36_omega_s})-acc;
4'd3:acc<=0;
4'd4:acc<=acc+(product>>>24);
4'd5:acc<=coefficient_dynamic ? (r40_position_unwind_s ? (acc>>>2) : (acc<<<2)) : (acc>>>4); // R40: narrow ramp only without positioning permission.
4'd6:acc<=coefficient_dynamic && r40_position_unwind_s ? r38_strong_shaped : r36_rate_shaped; // R40: original strong correction during explicit positioning.
4'd7:acc<=r36_fast_enabled_s ? $signed({{32{effective[31]}},effective}) : 64'sd0;
4'd8:acc<=r36_observer_omega;
4'd9:acc<=r39_mode&&r39_matrix_added ? 64'sd0 : acc;
default:begin end
endcase
end
end
always @(posedge clk)begin if(rst_n&&!clear&&busy)begin case(r39t4_lut_op)
3'd1:begin lut_abs<=n3[31]?-$unsigned(n3):$unsigned(n3);lut_negative<=n3[31];end
3'd2:begin lut_abs<=z3[31]?-$unsigned(z3):$unsigned(z3);lut_negative<=z3[31];end
3'd3:begin lut_abs<=r36_feedback_velocity[31]?-$unsigned(r36_feedback_velocity):$unsigned(r36_feedback_velocity);lut_negative<=r36_feedback_velocity[31];end
3'd4:begin lut_scaled<=({3'b0,lut_abs}<<2)+{3'b0,lut_abs};end
3'd5:begin lut_rounded<=lut_scaled+35'd4096;end
3'd6:begin lut_address<=lut_rounded[22:13];lut_overflow<=|lut_rounded[34:23];end
3'd7:begin lut_value<=tanh_word;end
default:begin end
endcase end end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
case(r39t4_tfold_op)
2'd1:tf_old<=lut_negative ? -$signed(lut_overflow ? 32'd16777216 : {7'd0,lut_value}) : $signed(lut_overflow ? 32'd16777216 : {7'd0,lut_value});
2'd2:tf_old<=r38_explicit_endpoint_s&&r38_current_s&&motion_latched&&motion_phase==3 ? r38_rate_replace_delta : 32'sd0;
default:begin end
endcase
end
end
always @(posedge clk)begin
if(rst_n&&!clear&&busy)begin
case(r39t4_tfnew_op)
2'd1:tf_new<=lut_negative ? -$signed(lut_overflow ? 32'd16777216 : {7'd0,lut_value}) : $signed(lut_overflow ? 32'd16777216 : {7'd0,lut_value});
default:begin end
endcase
end
end
always @(posedge clk or negedge rst_n)begin
if(!rst_n)begin n0<=0;n1<=0;n2<=0;n3<=0;n4<=0;n5<=0;n6<=0;end
else if(clear)begin n0<=0;n1<=0;n2<=0;n3<=0;n4<=0;n5<=0;n6<=0;end
else if(busy&&!(r39_mode&&!r39_matrix_added))begin case(r39t4_ncommit_op)
3'd1:n0<=sat(acc);
3'd2:n1<=sat(acc);
3'd3:n2<=sat(acc);
3'd4:n3<=sat(acc);
3'd5:n4<=sat(acc);
3'd6:n5<=sat(acc);
3'd7:n6<=sat(acc);
default:begin end
endcase end end
always @(posedge clk)if(busy&&((r39t4_outerr_op==1&&!r39_mode)||r39t4_outerr_op==2))output_error<=request-product[31:0];

endmodule


module r34_hold_damping_gate(input wire clk,rst_n,clear,tick,eligible,postmove,input wire signed[31:0]theta,output reg[3:0]weight);
 reg[9:0]quiet;reg signed[1:0]sign;reg[8:0]elapsed;reg[2:0]halves;reg active;reg[4:0]ramp;reg mode;
 wire[8:0] upper=postmove?9'd500:9'd180;
 wire[8:0] lower=postmove?9'd60:9'd80;
 wire signed[1:0]hit=theta>=1144?2'sd1:theta<=-1144?-2'sd1:2'sd0;
 wire[8:0]next_elapsed=sign==0 ? 9'd0:(elapsed<(upper+9'd1) ? elapsed+9'd1:upper+9'd1);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin quiet<=0;sign<=0;elapsed<=0;halves<=0;active<=0;weight<=0;ramp<=0;mode<=0;end
  else if(clear)begin quiet<=0;sign<=0;elapsed<=0;halves<=0;active<=0;weight<=0;ramp<=0;mode<=0;end
  else if(tick)begin
   mode<=postmove;
   if(!eligible || mode!=postmove)begin quiet<=0;sign<=0;elapsed<=0;halves<=0;active<=0;weight<=0;ramp<=0;end
   else if(quiet<1000)quiet<=quiet+1'b1;
   else begin
    if(!active)begin
     if(sign!=0)elapsed<=next_elapsed;
     if(hit!=0&&hit!=sign)begin
      if(sign!=0&&next_elapsed>=lower&&next_elapsed<=upper)begin
       if(halves<4)halves<=halves+1'b1;
       if(halves>=3)active<=1;
      end else halves<=0;
      sign<=hit;elapsed<=0;
     end else if(next_elapsed>upper)begin sign<=0;elapsed<=0;halves<=0;end
    end
    if(active&&weight<3)begin
     if(ramp>=19)begin ramp<=0;weight<=weight+1'b1;end else ramp<=ramp+1'b1;
    end
   end
  end
 end
endmodule
