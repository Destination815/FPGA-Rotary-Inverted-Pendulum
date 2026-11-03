from pathlib import Path
P=Path(__file__).resolve().parent
source=P.parents[1]/'fpga/src/state_estimator.v'
s=source.read_text()
s=s.replace('tick,adc_ready,calibrate,','tick,adc_ready,calibrate,gap_allow,')
s=s.replace('output reg step_valid,measurement_ok,','output reg step_valid,measurement_ok,\n output reg gap_active,output wire[7:0] gap_ms,')
s=s.replace('reg [3:0] stage;','reg [4:0] stage;')
pos=s.index(' always @(posedge clk')
s=s[:pos]+''' reg bridge_start,gap_allow_s;
 wire bridge_done,bridge_use,bridge_active,bridge_ok,bridge_busy;
 wire signed[31:0] bridge_position,bridge_velocity;
 reg signed[31:0] gap_rate_abs,gap_rate_product,gap_rate_rounded;
 r01_gap_bridge gap_bridge(.clk(clk),.rst_n(rst_n),.clear(calibrate),.start(bridge_start),
 .gap_allow(gap_allow_s),.ab_initialized(ab_initialized_s),.ab_position(ab_position_s),
 .ab_velocity(ab_velocity_s),.current_arm_velocity(dphi_next),.previous_arm_velocity(dphi_s),
 .raw_wrapped(wrapped),.adc_raw(adc_s),.zero_raw(zero_s),.busy(bridge_busy),.done(bridge_done),
 .use_gap(bridge_use),.gap_active(bridge_active),.measurement_ok(bridge_ok),
 .position_out(bridge_position),.velocity_out(bridge_velocity),.gap_ms(gap_ms));
'''+s[pos:]
s=s.replace('stage<=0;step_valid<=0;measurement_ok<=0;','stage<=0;step_valid<=0;measurement_ok<=0;bridge_start<=0;gap_allow_s<=0;gap_active<=0;')
s=s.replace('step_valid<=0;\n   if(calibrate)', 'step_valid<=0;bridge_start<=0;\n   if(calibrate)')
s=s.replace('enc_zero<=encoder;initialized<=0;', 'gap_active<=0;enc_zero<=encoder;initialized<=0;')
s=s.replace('adc_s<=adc;zero_s<=zero_raw;', 'gap_allow_s<=gap_allow && adc_ready;adc_s<=adc;zero_s<=zero_raw;')
s=s.replace('    15: begin\n     history_index', '''    15:begin bridge_start<=1;stage<=16;end
    16:if(bridge_done)begin gap_rate_abs<=bridge_velocity<0 ? -bridge_velocity:bridge_velocity;stage<=17;end
    17:begin gap_rate_product<=gap_rate_abs*32'sd1000;stage<=18;end
    18:begin gap_rate_rounded<=gap_rate_product+32'sd128;stage<=19;end
    19: begin
     gap_active<=bridge_active;
     history_index''')
s=s.replace('if(!ab_initialized_s || !raw_good_s || !jump_good) begin\n      ab_position', '''if(bridge_use)begin
      ab_position<=bridge_position;ab_velocity<=bridge_velocity;ab_initialized<=bridge_ok;
      filtered_theta_q18<=bridge_position>>>6;
     end else if(!ab_initialized_s || !raw_good_s || !jump_good) begin
      ab_position''')
s=s.replace('step_valid<=1;theta<=wrapped;phi<=phi_now;last_theta<=wrapped;last_phi<=phi_now;', '''step_valid<=1;theta<=bridge_use ? bridge_position>>>8:wrapped;phi<=phi_now;
     last_theta<=bridge_use ? bridge_position>>>8:wrapped;last_phi<=phi_now;''')
s=s.replace('previous_good<=raw_good_s && (jump_good || !initialized_s);', 'previous_good<=bridge_use ? bridge_ok:(raw_good_s && (jump_good || !initialized_s));')
s=s.replace('measurement_ok<=raw_good_s && initialized_s && jump_good;', 'measurement_ok<=bridge_use ? bridge_ok:(raw_good_s && initialized_s && jump_good);')
s=s.replace('if(raw_good_s && previous_good_s && jump_good && ab_initialized_s) dtheta<=rate_next;', "if(bridge_use)dtheta<=bridge_velocity<0 ? -(gap_rate_rounded>>>8):(gap_rate_rounded>>>8);\n      else if(raw_good_s && previous_good_s && jump_good && ab_initialized_s) dtheta<=rate_next;")
s=s.replace('// T02: integer-equivalent estimator with registered arithmetic stages.', '// R01: T03normalAB preserved; measured-only180msblindsectorbridgeadded.')
(P/'state_estimator_gap.v').write_text(s)
