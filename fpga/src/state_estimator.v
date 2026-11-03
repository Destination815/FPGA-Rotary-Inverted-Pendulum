`include "j280_config.vh"
`default_nettype none
// F03: fractional ADC mean preserved through angle/velocity; R02 alpha-beta unchanged.
// R01: T03normalAB preserved; measured-only180msblindsectorbridgeadded.
// Accept tick only while idle; commit all four outputs and step_valid together
// 15 clk periods later (300 ns at 50 MHz, inside the unchanged 1 ms period).
// reset/calibrate abort an in-flight sample. No timing exceptions are needed.
module state_estimator(input wire clk,rst_n,tick,adc_ready,calibrate,gap_allow,restart_observation,
 input wire [9:0] adc,zero_raw, input wire [13:0] adc_q4,zero_q4,input wire adc_q4_valid, input wire signed [31:0] encoder,
 output reg step_valid,measurement_ok,
 output reg gap_active,output wire[7:0] gap_ms,
 output reg signed [31:0] theta,dtheta,phi,dphi,theta_raw_q16,
 output reg signed [31:0] filtered_theta_q18);
 localparam signed [31:0] PI=205887,TWO_PI=411775;
 reg signed [31:0] enc_zero,last_phi,last_theta;
 reg initialized,previous_good;
 reg signed [31:0] phi_history[0:15];
 reg [3:0] history_index;reg [4:0] history_fill;integer i;
 // Alpha-beta observer in Q24 rad and Q24 rad/ms, as in T01.
 reg signed [31:0] ab_position,ab_velocity;
 reg ab_initialized;
 reg [4:0] stage;
 // Snapshot inputs AND previous estimator state. Nothing below reads live
 // ADC/encoder inputs until the next tick is accepted.
 reg [9:0] adc_s,zero_s;
 reg [13:0] fine_adc_s,fine_zero_s;
 reg signed [31:0] fine_delta,fine_angle,fine_wrapped;
 reg signed [31:0] encoder_s,enc_zero_s,last_theta_s,last_phi_s;
 reg signed [31:0] history_s,dphi_s,ab_position_s,ab_velocity_s;
 reg initialized_s,previous_good_s,ab_initialized_s,raw_good_s,full_history_s;
 reg [3:0] history_index_s;
 reg [4:0] history_fill_s;
 reg signed [31:0] adc_delta,encoder_delta,raw_angle,wrapped,phi_now;
 reg signed [63:0] enc_product;
 reg signed [31:0] predicted_sum,ab_predicted,delta_raw,delta;
 reg signed [31:0] error_raw,ab_error,error_abs,round_sum,ab_correction;
 reg signed [31:0] phi_delta,phi_rate_product,phi_rate_div,rate_phi;
 reg signed [31:0] phi_rate_delta,dphi_next,correction_15,position_next;
 reg signed [63:0] velocity_sum;
 reg signed [31:0] velocity_next,velocity_abs,rate_product,rate_rounded,rate_next;
 reg jump_good;
 reg bridge_start,gap_allow_s;
 wire bridge_done,bridge_use,bridge_active,bridge_ok,bridge_busy;
 wire signed[31:0] bridge_position,bridge_velocity;
 reg signed[31:0] gap_rate_abs,gap_rate_product,gap_rate_rounded;
 r01_gap_bridge gap_bridge(.clk(clk),.rst_n(rst_n),.clear(calibrate || restart_observation),.start(bridge_start),
 .gap_allow(gap_allow_s),.ab_initialized(ab_initialized_s),.ab_position(ab_position_s),
 .ab_velocity(ab_velocity_s),.current_arm_velocity(dphi_next),.previous_arm_velocity(dphi_s),
 .raw_wrapped(fine_wrapped>>>8),.adc_raw(adc_s),.zero_q4(fine_zero_s),.busy(bridge_busy),.done(bridge_done),
 .use_gap(bridge_use),.gap_active(bridge_active),.measurement_ok(bridge_ok),
 .position_out(bridge_position),.velocity_out(bridge_velocity),.gap_ms(gap_ms));
 // Constant shift/add expansion of the canonical legacy scale; no DSP added.
 function signed[63:0] legacy_encoder_product;
  input signed[31:0] value;reg signed[63:0] result,wide;integer bit_no;
  begin result=0;wide={{32{value[31]}},value};
   for(bit_no=0;bit_no<32;bit_no=bit_no+1)
    if(((`J280_C_LEGACY_Q16_PER_COUNT>>bit_no)&32'd1)!=0)result=result+(wide<<<bit_no);
   legacy_encoder_product=result;
  end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   enc_zero<=0;last_phi<=0;last_theta<=0;initialized<=0;previous_good<=0;
   history_index<=0;history_fill<=0;
   for(i=0;i<16;i=i+1) phi_history[i]<=0;
   ab_position<=0;ab_velocity<=0;ab_initialized<=0;
   stage<=0;step_valid<=0;measurement_ok<=0;bridge_start<=0;gap_allow_s<=0;gap_active<=0;
   theta<=0;theta_raw_q16<=0;dtheta<=0;phi<=0;dphi<=0;filtered_theta_q18<=0;
  end else begin
   step_valid<=0;bridge_start<=0;
   if(calibrate) begin
    gap_active<=0;enc_zero<=encoder;initialized<=0;previous_good<=0;dtheta<=0;dphi<=0;
    history_index<=0;history_fill<=0;
    ab_position<=0;ab_velocity<=0;ab_initialized<=0;stage<=0;filtered_theta_q18<=0;
   end else if(restart_observation) begin
    // Invalid blind prediction is discarded only after automatic output is zero.
    // Encoder zero/history and arm velocity are deliberately retained.
    ab_initialized<=0;previous_good<=0;ab_velocity<=0;dtheta<=0;
    measurement_ok<=0;gap_active<=0;stage<=0;bridge_start<=0;
   end else case(stage)
    0: if(tick) begin
     gap_allow_s<=gap_allow && adc_ready;adc_s<=adc;zero_s<=zero_raw;
     fine_adc_s<=adc_q4_valid?adc_q4:{adc,4'd0};fine_zero_s<=zero_q4;encoder_s<=encoder;enc_zero_s<=enc_zero;
     last_theta_s<=last_theta;last_phi_s<=last_phi;
     history_s<=phi_history[history_index];dphi_s<=dphi;
     ab_position_s<=ab_position;ab_velocity_s<=ab_velocity;
     initialized_s<=initialized;previous_good_s<=previous_good;
     ab_initialized_s<=ab_initialized;
     raw_good_s<=adc_ready && adc>3 && adc<1020;
     full_history_s<=history_fill>=16;
     history_index_s<=history_index;history_fill_s<=history_fill;stage<=1;
    end
    1: begin
     adc_delta<=$signed({22'd0,adc_s})-$signed({22'd0,zero_s});
     fine_delta<=$signed({18'd0,fine_adc_s})-$signed({18'd0,fine_zero_s});
     // The original $signed(encoder-enc_zero) subtracts at 32 bits.
     encoder_delta<=encoder_s-enc_zero_s;stage<=2;
    end
    2: begin
     raw_angle<=((adc_delta<<<8)+(adc_delta<<<7)+(adc_delta<<<1));
     fine_angle<=(((fine_delta<<<8)+(fine_delta<<<7)+(fine_delta<<<1)))<<<4;
     enc_product<=legacy_encoder_product(encoder_delta);
     predicted_sum<=ab_position_s+ab_velocity_s;stage<=3;
    end
    3: begin
     if(fine_angle>PI*256) fine_wrapped<=fine_angle-TWO_PI*256;
     else if(fine_angle< -PI*256) fine_wrapped<=fine_angle+TWO_PI*256;
     else fine_wrapped<=fine_angle;
     if(raw_angle>PI) wrapped<=raw_angle-TWO_PI;
     else if(raw_angle< -PI) wrapped<=raw_angle+TWO_PI;
     else wrapped<=raw_angle;
     if(enc_product>64'sd2000000) phi_now<=32'sd2000000;
     else if(enc_product< -64'sd2000000) phi_now<=-32'sd2000000;
     else phi_now<=enc_product[31:0];
     if(predicted_sum>PI*256) ab_predicted<=predicted_sum-TWO_PI*256;
     else if(predicted_sum< -PI*256) ab_predicted<=predicted_sum+TWO_PI*256;
     else ab_predicted<=predicted_sum;stage<=4;
    end
    4: begin
     delta_raw<=wrapped-last_theta_s;
     error_raw<=fine_wrapped-ab_predicted;
     phi_delta<=phi_now-(full_history_s?history_s:last_phi_s);stage<=5;
    end
    5: begin
     if(delta_raw>PI) delta<=delta_raw-TWO_PI;
     else if(delta_raw< -PI) delta<=delta_raw+TWO_PI;
     else delta<=delta_raw;
     if(error_raw>PI*256) ab_error<=error_raw-TWO_PI*256;
     else if(error_raw< -PI*256) ab_error<=error_raw+TWO_PI*256;
     else ab_error<=error_raw;
     // Intentionally 32-bit product, including original overflow semantics.
     phi_rate_product<=full_history_s?((phi_delta<<<7)-(phi_delta<<<1)-phi_delta):((phi_delta<<<10)-(phi_delta<<<4)-(phi_delta<<<3));stage<=6;
    end
    6: begin
     error_abs<=ab_error<0?-ab_error:ab_error;
     // Signed /2 truncates toward zero, unlike arithmetic right shift.
     phi_rate_div<=full_history_s?phi_rate_product/32'sd2:phi_rate_product;
     stage<=7;
    end
    7: begin
     round_sum<=error_abs+32'sd32;
     if(phi_rate_div>32'sd1966080) rate_phi<=32'sd1966080;
     else if(phi_rate_div< -32'sd1966080) rate_phi<=-32'sd1966080;
     else rate_phi<=phi_rate_div;
     jump_good<=delta<9830 && delta> -9830;stage<=8;
    end
    8: begin
     ab_correction<=ab_error<0?-(round_sum>>>6):(round_sum>>>6);
     phi_rate_delta<=rate_phi-dphi_s;stage<=9;
    end
    9: begin
     velocity_sum<={{32{ab_velocity_s[31]}},ab_velocity_s}+{{32{ab_correction[31]}},ab_correction};
     correction_15<=((ab_correction<<<4)-ab_correction);
     dphi_next<=dphi_s+(phi_rate_delta>>>2);stage<=10;
    end
    10: begin
     if(velocity_sum>64'sd503316) velocity_next<=32'sd503316;
     else if(velocity_sum< -64'sd503316) velocity_next<=-32'sd503316;
     else velocity_next<=velocity_sum[31:0];
     position_next<=ab_predicted+correction_15;stage<=11;
    end
    11: begin
     velocity_abs<=velocity_next<0?-velocity_next:velocity_next;stage<=12;
    end
    12: begin
     rate_product<=((velocity_abs<<<10)-(velocity_abs<<<4)-(velocity_abs<<<3));stage<=13;
    end
    13: begin
     rate_rounded<=rate_product+32'sd128;stage<=14;
    end
    14: begin
     rate_next<=velocity_next<0?-(rate_rounded>>>8):(rate_rounded>>>8);
     stage<=15;
    end
    15:begin bridge_start<=1;stage<=16;end
    16:if(bridge_done)begin gap_rate_abs<=bridge_velocity<0 ? -bridge_velocity:bridge_velocity;stage<=17;end
    17:begin gap_rate_product<=((gap_rate_abs<<<10)-(gap_rate_abs<<<4)-(gap_rate_abs<<<3));stage<=18;end
    18:begin gap_rate_rounded<=gap_rate_product+32'sd128;stage<=19;end
    19: begin
     gap_active<=bridge_active;
     history_index<=history_index_s+1'b1;
     if(history_fill_s<16) history_fill<=history_fill_s+1'b1;
     phi_history[history_index_s]<=phi_now;
     if(bridge_use)begin
      ab_position<=bridge_position;ab_velocity<=bridge_velocity;ab_initialized<=bridge_ok;
      filtered_theta_q18<=bridge_position>>>6;
     end else if(!ab_initialized_s || !raw_good_s || !jump_good) begin
      ab_position<=fine_wrapped;ab_velocity<=0;ab_initialized<=raw_good_s;filtered_theta_q18<=fine_wrapped>>>6;
     end else begin
      ab_position<=position_next;ab_velocity<=velocity_next;filtered_theta_q18<=position_next>>>6;
     end
     step_valid<=1;theta<=bridge_use ? bridge_position>>>8:fine_wrapped>>>8;theta_raw_q16<=wrapped;phi<=phi_now;
     last_theta<=bridge_use ? bridge_position>>>8:wrapped;last_phi<=phi_now;
     initialized<=1;previous_good<=bridge_use ? bridge_ok:(raw_good_s && (jump_good || !initialized_s));
     measurement_ok<=bridge_use ? bridge_ok:(raw_good_s && initialized_s && jump_good);
     if(!initialized_s) begin dtheta<=0;dphi<=0;end
     else begin
      if(bridge_use)dtheta<=bridge_velocity<0 ? -(gap_rate_rounded>>>8):(gap_rate_rounded>>>8);
      else if(raw_good_s && previous_good_s && jump_good && ab_initialized_s) dtheta<=rate_next;
      else dtheta<=0;
      dphi<=dphi_next;
     end
     stage<=0;
    end
    default:stage<=0;
   endcase
  end
 end
endmodule
`default_nettype wire
