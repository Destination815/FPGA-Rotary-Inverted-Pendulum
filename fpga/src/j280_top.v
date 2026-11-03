`include "j280_config.vh"
`include "j280_coordinates.vh"
`default_nettype none
module j280_top #(parameter integer MOTOR_CHANNEL=`J280_CHANNEL,parameter integer UART_DIV=`J280_CLK_HZ/`J280_BAUD)(input wire clk_50m,rst_n,input wire [3:0] key_sw,
 input wire uart_rx,output wire uart_tx,
 output wire adc_clk,adc_oe_n,input wire [9:0] adc_data_in,
 input wire enc1_a,enc1_b,enc2_a,enc2_b,
 output wire AN1,AN2,PWMA,BN1,BN2,PWMB,output wire [3:0] led_n);
 reg [1:0] reset_sync;
 wire reset_n=reset_sync[1];
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) reset_sync<=0;else reset_sync<={reset_sync[0],1'b1};
 end
 reg [15:0] divider;reg tick;reg [31:0] millis;
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) begin divider<=0;tick<=0;millis<=0;end
  else begin
   tick<=0;
   if(divider==49999) begin divider<=0;tick<=1;millis<=millis+1'b1;end
   else divider<=divider+1'b1;
  end
 end
 reg [13:0] control_adc_q4;
 reg [9:0] control_adc_raw;reg control_adc_fine_valid;
 always @(posedge clk_50m or negedge reset_n) if(!reset_n)begin control_adc_raw<=0;control_adc_fine_valid<=0;end else if(tick)begin control_adc_raw<=adc;control_adc_fine_valid<=valid_q4;end
 always @(posedge clk_50m or negedge reset_n) if(!reset_n) control_adc_q4<=0;else if(tick)control_adc_q4<=valid_q4?adc_q4:{adc,4'd0};
 wire [9:0] adc;wire adc_ready;wire [13:0] adc_q4;wire ready_q4,valid_q4;wire signed [31:0] enc;wire [15:0] illegal;
 adc_parallel adc_inst(.clk(clk_50m),.rst_n(reset_n),.data(adc_data_in),.adc_clk(adc_clk),.oe_n(adc_oe_n),.average(adc),.ready(adc_ready),.average_q4(adc_q4),.ready_q4(ready_q4),.valid_q4(valid_q4));
 // Balance_Ctrl_20260618 sheet 1: XH1 = bridge B + encoder 1; XH2 = bridge A + encoder 2.
 encoder_quad enc_inst(.clk(clk_50m),.rst_n(reset_n),
 .a(MOTOR_CHANNEL==1?enc1_a:enc2_a),.b(MOTOR_CHANNEL==1?enc1_b:enc2_b),.count(enc),.illegal(illegal));
 wire [3:0] key_press;wire key_stop;wire link_stop;
 wire stop_request=key_stop || link_stop;
 key_input keys(.clk(clk_50m),.rst_n(reset_n),.keys_n(key_sw),.pressed(key_press),.stop_level(key_stop));
 wire rx_valid,rx_error;wire [7:0] rx_byte;
 uart_rx #(.DIV(UART_DIV)) rx_inst(.clk(clk_50m),.rst_n(reset_n),.rx(uart_rx),.valid(rx_valid),.error(rx_error),.data(rx_byte));
 wire cmd_valid,staged_cmd_valid,wire_cmd_valid,target_overflow,mapped_overflow;wire [7:0] cmd_seq,opcode,wire_cmd_seq,wire_opcode;wire signed [31:0] cmd_value,wire_cmd_value;wire [15:0] cmd_errors;
 wire mapped_valid;wire[7:0]mapped_seq,mapped_opcode;wire signed[31:0]mapped_value;
 wire[31:0]cfg_payload;wire[1:0]cfg_admission;
 // Ordinary commands cross two FF boundaries. Only CRC-valid STOP has a
 // separate fast path; its raw opcode never selects the ordinary payload.
 wire serial_stop=wire_cmd_valid && wire_opcode==8'h01;
 wire command_cancel=stop_request || serial_stop;
 // A newer raw command cancels every older uncommitted command, including
 // a result already in the last stage. One bounded transaction, no queue.
 assign cmd_valid=staged_cmd_valid && !wire_cmd_valid && !stop_request;
 reg[1:0] command_admission_q;
 always @(posedge clk_50m or negedge reset_n)begin
  if(!reset_n)command_admission_q<=2'd3;
  else if(command_cancel)command_admission_q<=2'd3;
  else if(wire_cmd_valid)begin
   if(diag_active || dump_active || diag_begin)command_admission_q<=2'd2;
   else if(wire_opcode==8'h38)command_admission_q<=
    armed && (state==3 || state==4) &&
    ((wire_cmd_value==1 && !ext2_prepared && calibrated && measurement_ok && reference_ready && ext2_center_ok) ||
     (wire_cmd_value==0 && ext2_prepared)) ? 2'd0:2'd2;
   else command_admission_q<=(state==0 && !armed)?2'd0:2'd3;
  end
 end
 calibrated_command coordinates_cmd(.clk(clk_50m),.rst_n(reset_n),.stop_level(command_cancel),.in_valid(wire_cmd_valid),.in_seq(wire_cmd_seq),.in_opcode(wire_opcode),.in_value(wire_cmd_value),.out_valid(mapped_valid),.out_seq(mapped_seq),.out_opcode(mapped_opcode),.out_value(mapped_value),.overflow(mapped_overflow));
 // The second registered stage aligns opcode/sequence/value/overflow and
 // uses admission captured when the original CRC-valid command arrived.
 motion_config_stage config_stage(.clk(clk_50m),.rst_n(reset_n),.stop_level(command_cancel),.new_command(wire_cmd_valid),
 .in_valid(mapped_valid),.in_seq(mapped_seq),.in_opcode(mapped_opcode),.in_value(mapped_value),.in_overflow(mapped_overflow),
 .admission(command_admission_q),
 .out_valid(staged_cmd_valid),.out_seq(cmd_seq),.out_opcode(opcode),.out_value(cmd_value),.out_overflow(target_overflow),
 .configuration(cfg_payload),.saved_admission(cfg_admission));
 command_rx parser(.clk(clk_50m),.rst_n(reset_n),.byte_valid(rx_valid),.byte_error(rx_error),.byte_data(rx_byte),.valid(wire_cmd_valid),.sequence_id(wire_cmd_seq),.opcode(wire_opcode),.value(wire_cmd_value),.errors(cmd_errors));
 localparam [31:0] D02_BUILD=`J280_C_BUILD; // R01 integrated fixed-point swing/balance/trajectory
 wire legacy_ready,diag_ready,diag_active,diag_drive;wire [2:0] diag_phase,legacy_phase;wire [7:0] diag_reason,legacy_reason;
 wire [13:0] diag_count;wire [10:0] legacy_count;wire [31:0] diag_id,diag_start,legacy_id,legacy_start;wire signed [15:0] diag_amplitude,legacy_amplitude;
 wire signed [31:0] diag_command,applied_debug;reg diag_begin;
 reg [12:0] dump_index,read_index;reg dump_active;reg [1:0] read_wait;
 reg [175:0] dump_row0,dump_row1;reg [1:0] dump_rows;reg [2:0] dump_stage;
 wire [319:0] shared_legacy_data;wire legacy_read_valid,legacy_overrun;
 wire [319:0] diag_read_data,legacy_read_data;reg [319:0] diag_sample;
 wire signed[31:0] capture_origin;
 wire diag_stop=stop_request || serial_stop;
 reg [2:0] previous_state;
 reg[31:0] motion_configuration;reg signed[31:0] workspace_limit,shuttle_amplitude,motion_speed_limit;reg[1:0] motion_speed,speed_admission;reg[7:0] shuttle_period;
 // R40 runtime profile. Keep ordinary configuration and capture origin.
 // ext2_consumed records reference start only; repeated runs are allowed.
 reg ext2_prepared,ext2_consumed;
 reg signed[31:0] ext2_amplitude;
 reg[31:0] ext2_phase_step;
 reg signed[31:0] ext2_travel,ext2_center_bound,ext2_center_lower;
 // These parameter-only bounds retire long before the next CRC command.
 // No wide subtract/negate cascade on the entry acceptance path.
 always @(posedge clk_50m)begin
  ext2_travel<=workspace_limit-32'sd11438;
  ext2_center_bound<=ext2_travel-ext2_amplitude;
  ext2_center_lower<=-ext2_center_bound;
 end
 wire ext2_configuration=ext2_prepared;
 // R41T1: balanced byte comparisons, exact signed32 domain including wrap.
 // This is combinational: both original command-edge checks remain current.
 function r41t1_gt32;
  input signed[31:0] a,b;
  reg[3:0] gt,eq;
  begin
   gt={a[30:24]>b[30:24],a[23:16]>b[23:16],a[15:8]>b[15:8],a[7:0]>b[7:0]};
   eq={a[30:24]==b[30:24],a[23:16]==b[23:16],a[15:8]==b[15:8],1'b1};
   r41t1_gt32=(a[31]!=b[31]) ? !a[31] :
    gt[3] || (eq[3]&&gt[2]) || (eq[3]&&eq[2]&&gt[1]) ||
    (eq[3]&&eq[2]&&eq[1]&&gt[0]);
  end
 endfunction
 // R43: admission and reference generation use the same retained center.
 // Keep all Q18 bits; truncating a fallback center can admit an invalid edge.
 wire signed[31:0] ext2_entry_center_q18;
 wire ext2_center_ok=ext2_center_bound>=0 &&
   !r41t1_gt32(ext2_center_lower<<<2,ext2_entry_center_q18) &&
   !r41t1_gt32(ext2_entry_center_q18,ext2_center_bound<<<2);
 always @(posedge clk_50m or negedge reset_n)begin
  if(!reset_n)ext2_consumed<=0;
  else if(!armed || !ext2_prepared)ext2_consumed<=0;
  else if(tick && trajectory_running)ext2_consumed<=1;
 end
 wire[8:0] cfg_range={cfg_payload[29],cfg_payload[7:0]};wire[10:0] cfg_amp=cfg_payload[18:8];wire[1:0] cfg_speed=cfg_payload[20:19];wire[6:0] cfg_period=cfg_payload[27:21];
 wire period_ok=cfg_period==8 || cfg_period==10 || cfg_period==12 || cfg_period==16 || cfg_period==20 || cfg_period==30 || cfg_period==40 || cfg_period==60 || cfg_period==90 || cfg_period==120;
 wire[15:0] cfg_period_wide={9'd0,cfg_period};
 wire[15:0] cfg_ten_period=(cfg_period_wide<<3)+(cfg_period_wide<<1);
 wire[15:0] cfg_velocity=(cfg_speed==3 ? ((cfg_ten_period<<2)+(cfg_ten_period<<1)):cfg_speed==2 ? ((cfg_ten_period<<1)+cfg_ten_period):cfg_speed==1 ? (cfg_ten_period<<1):cfg_ten_period);
 wire[15:0] cfg_amp_wide={5'd0,cfg_amp};
 wire[15:0] cfg_range_margin={7'd0,cfg_range}-16'd10;
 wire[15:0] cfg_range_tenths=(cfg_range_margin<<3)+(cfg_range_margin<<1);
 wire[15:0] cfg_nine_amp=(cfg_amp_wide<<3)+cfg_amp_wide;
 wire[15:0] cfg_ten_velocity=(cfg_velocity<<3)+(cfg_velocity<<1);
 wire[21:0] cfg_range_q=({13'd0,cfg_range}<<10)+({13'd0,cfg_range}<<7)-({13'd0,cfg_range}<<3);
 wire[20:0] amp_hi=amp_hi_table(cfg_amp[10:5]);
 wire[14:0] amp_lo=amp_lo_table(cfg_amp[4:0]);
 wire[3:0] amp_rem={1'b0,amp_hi[2:0]}+{1'b0,amp_lo[2:0]};
 wire[17:0] cfg_amp_rad=amp_hi[20:3]+{6'd0,amp_lo[14:3]}+{17'd0,(amp_rem>=5)};

 function [20:0] amp_hi_table;input[5:0] a;begin case(a)
 0:amp_hi_table=21'd0;
 1:amp_hi_table=21'd29284;
 2:amp_hi_table=21'd58571;
 3:amp_hi_table=21'd87858;
 4:amp_hi_table=21'd117145;
 5:amp_hi_table=21'd146432;
 6:amp_hi_table=21'd175716;
 7:amp_hi_table=21'd205003;
 8:amp_hi_table=21'd234290;
 9:amp_hi_table=21'd263577;
 10:amp_hi_table=21'd292864;
 11:amp_hi_table=21'd322148;
 12:amp_hi_table=21'd351435;
 13:amp_hi_table=21'd380722;
 14:amp_hi_table=21'd410009;
 15:amp_hi_table=21'd439296;
 16:amp_hi_table=21'd468580;
 17:amp_hi_table=21'd497867;
 18:amp_hi_table=21'd527154;
 19:amp_hi_table=21'd556441;
 20:amp_hi_table=21'd585728;
 21:amp_hi_table=21'd615012;
 22:amp_hi_table=21'd644299;
 23:amp_hi_table=21'd673586;
 24:amp_hi_table=21'd702873;
 25:amp_hi_table=21'd732160;
 26:amp_hi_table=21'd761444;
 27:amp_hi_table=21'd790731;
 28:amp_hi_table=21'd820018;
 29:amp_hi_table=21'd849305;
 30:amp_hi_table=21'd878592;
 31:amp_hi_table=21'd907876;
 32:amp_hi_table=21'd937163;
 33:amp_hi_table=21'd966450;
 34:amp_hi_table=21'd995737;
 35:amp_hi_table=21'd1025024;
 36:amp_hi_table=21'd1054308;
 37:amp_hi_table=21'd1083595;
 38:amp_hi_table=21'd1112882;
 39:amp_hi_table=21'd1142169;
 40:amp_hi_table=21'd1171456;
 41:amp_hi_table=21'd1200740;
 42:amp_hi_table=21'd1230027;
 43:amp_hi_table=21'd1259314;
 44:amp_hi_table=21'd1288601;
 45:amp_hi_table=21'd1317888;
 46:amp_hi_table=21'd1347172;
 47:amp_hi_table=21'd1376459;
 48:amp_hi_table=21'd1405746;
 49:amp_hi_table=21'd1435033;
 50:amp_hi_table=21'd1464320;
 51:amp_hi_table=21'd1493604;
 52:amp_hi_table=21'd1522891;
 53:amp_hi_table=21'd1552178;
 54:amp_hi_table=21'd1581465;
 55:amp_hi_table=21'd1610752;
 56:amp_hi_table=21'd1640036;
 57:amp_hi_table=21'd1669323;
 58:amp_hi_table=21'd1698610;
 59:amp_hi_table=21'd1727897;
 60:amp_hi_table=21'd1757184;
 61:amp_hi_table=21'd1786468;
 62:amp_hi_table=21'd1815755;
 63:amp_hi_table=21'd1845042;
 default:amp_hi_table=0;endcase end endfunction
 function [14:0] amp_lo_table;input[4:0] a;begin case(a)
 0:amp_lo_table=15'd0;
 1:amp_lo_table=15'd914;
 2:amp_lo_table=15'd1828;
 3:amp_lo_table=15'd2745;
 4:amp_lo_table=15'd3659;
 5:amp_lo_table=15'd4576;
 6:amp_lo_table=15'd5490;
 7:amp_lo_table=15'd6404;
 8:amp_lo_table=15'd7321;
 9:amp_lo_table=15'd8235;
 10:amp_lo_table=15'd9152;
 11:amp_lo_table=15'd10066;
 12:amp_lo_table=15'd10980;
 13:amp_lo_table=15'd11897;
 14:amp_lo_table=15'd12811;
 15:amp_lo_table=15'd13728;
 16:amp_lo_table=15'd14642;
 17:amp_lo_table=15'd15556;
 18:amp_lo_table=15'd16473;
 19:amp_lo_table=15'd17387;
 20:amp_lo_table=15'd18304;
 21:amp_lo_table=15'd19218;
 22:amp_lo_table=15'd20132;
 23:amp_lo_table=15'd21049;
 24:amp_lo_table=15'd21963;
 25:amp_lo_table=15'd22880;
 26:amp_lo_table=15'd23794;
 27:amp_lo_table=15'd24708;
 28:amp_lo_table=15'd25625;
 29:amp_lo_table=15'd26539;
 30:amp_lo_table=15'd27456;
 31:amp_lo_table=15'd28370;
 default:amp_lo_table=0;endcase end endfunction
 wire cfg_ok=(cfg_payload[31:30]==0 && cfg_range>=20 && cfg_range<=360 && period_ok && cfg_amp>=1 && cfg_amp_wide<=cfg_range_tenths && cfg_nine_amp<=cfg_ten_velocity);
 reg armed,calibrated,calibrate,start,capture,clear,jog,shuttle,target_request,shuttle_request;
 motion_link_watchdog link_watchdog(.clk(clk_50m),.rst_n(reset_n),.tick(tick),.armed(armed),
  .heartbeat(wire_cmd_valid && wire_opcode==8'h30),.expired(link_stop));
 reg [9:0] zero_raw;reg [13:0] zero_q4;reg [7:0] last_seq,cmd_status;
 reg signed [31:0] target,jog_value,ktheta,kdtheta,kphi,kdphi,pwm_limit,swing_gain;
 wire step,measurement_ok,gap_active;wire [7:0] gap_ms;wire signed [31:0] theta,dtheta,phi,dphi,filtered_theta_q18,theta_raw_q16;
 wire signed[31:0] observer_integral,observer_lp;wire[9:0] observer_age;wire[223:0] observer_state;wire[7:0] observer_samples;wire observer_ready;
 wire reference_ready,trajectory_running,recovery_boosting,recovery_used,settled,settle_timeout;wire signed [31:0] active_limit;
 wire [2:0] state;wire [7:0] fault;wire signed [31:0] command,reference,ref_velocity;wire result_valid;
 wire idle=(state==0);wire configure=idle && !armed;
 always @(posedge clk_50m or negedge reset_n)begin
  if(!reset_n)speed_admission<=3;
  else if(stop_request)speed_admission<=3;
  else if(wire_cmd_valid)speed_admission<=wire_opcode==8'h37 ? ((diag_active || dump_active || diag_begin)?2'd2:(configure?2'd0:2'd3)):2'd3;
 end
 wire calibration_ready;reg calibration_ready_q,reference_center_q;reg[9:0] calibration_adc_q;reg signed[31:0]calibration_encoder_q;
 always @(posedge clk_50m or negedge reset_n)if(!reset_n)begin calibration_adc_q<=0;calibration_encoder_q<=0;calibration_ready_q<=0;reference_center_q<=0;end else begin calibration_adc_q<=adc;calibration_encoder_q<=enc;calibration_ready_q<=calibration_ready;reference_center_q<=reference>= -1167 && reference<=1167;end
 calibration_guard calibration_check(.clk(clk_50m),.rst_n(reset_n),.step(step),.enabled(configure && ready_q4 && valid_q4),
 .measurement_ok(measurement_ok),.adc(adc),.encoder(enc),.ready(calibration_ready));
 // R27: register COAST-entry decode before wide estimator cancellation.
 // Output is already zero on COAST entry; restart is 20ns later, before next tick.
 reg estimator_restart_q;
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n)estimator_restart_q<=0;
  else estimator_restart_q<=state==7 && previous_state!=7;
 state_estimator estimator(.clk(clk_50m),.rst_n(reset_n),.tick(tick),.adc_ready(adc_ready && ready_q4),.calibrate(calibrate),.restart_observation(estimator_restart_q),.adc(adc),.zero_raw(zero_raw),.adc_q4(adc_q4),.zero_q4(zero_q4),.adc_q4_valid(valid_q4),.encoder(enc),
 .gap_allow(state==1 || state==6),.gap_active(gap_active),.gap_ms(gap_ms),.step_valid(step),.measurement_ok(measurement_ok),.theta(theta),.dtheta(dtheta),.phi(phi),.dphi(dphi),.filtered_theta_q18(filtered_theta_q18),.theta_raw_q16(theta_raw_q16));
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) begin
   ext2_amplitude<=32'sd48620;ext2_phase_step<=32'd1227134;motion_configuration<=32'd41981630;workspace_limit<=32'sd217360;shuttle_amplitude<=32'sd17157;motion_speed<=0;motion_speed_limit<=0;shuttle_period<=20;previous_state<=0;armed<=0;calibrated<=0;calibrate<=0;start<=0;capture<=0;clear<=0;jog<=0;shuttle<=0;
   diag_begin<=0;target_request<=0;shuttle_request<=0;zero_raw<=512;zero_q4<=14'd8192;last_seq<=0;cmd_status<=0;target<=0;jog_value<=0;ext2_prepared<=0;
   ktheta<=`J280_K_THETA_Q16;kdtheta<=`J280_K_DTHETA_Q16;kphi<=`J280_K_PHI_Q16;kdphi<=`J280_K_DPHI_Q16;
   pwm_limit<=`J280_PWM_LIMIT;swing_gain<=`J280_SWING_GAIN;
  end else begin
   calibrate<=0;start<=0;capture<=0;clear<=0;jog<=0;diag_begin<=0;target_request<=0;shuttle_request<=0;
   previous_state<=state;
   // Clear the profile on recovery/fault or after a completed fade exit.
   // Do not clear it between entry ACK and the first reference transaction.
   if(ext2_prepared && ((!armed || (state!=3 && state!=4)) ||
       (ext2_consumed && !trajectory_running && reference_ready)))begin
    ext2_prepared<=0;shuttle<=0;
   end
   if(drive_tripped || state==5 || (state==0 && (previous_state==2 || previous_state==3 || previous_state==4))) armed<=0;
   if(stop_request)begin armed<=0;ext2_prepared<=0;end
   else if(serial_stop)begin last_seq<=wire_cmd_seq;cmd_status<=0;armed<=0;shuttle<=0;ext2_prepared<=0;end
   else if(cmd_valid) begin
    last_seq<=cmd_seq;cmd_status<=0;
    if((diag_active || dump_active || diag_begin) && opcode!=1 && opcode!=8'h30 && opcode!=8'h32) cmd_status<=2;
    else case(opcode)
     8'h01:begin armed<=0;shuttle<=0;ext2_prepared<=0;end
     8'h02:if(calibration_ready_q && configure && ready_q4 && valid_q4 && measurement_ok && adc==calibration_adc_q && enc==calibration_encoder_q) begin
      zero_raw<=adc_q4[13:4];zero_q4<=adc_q4;calibrated<=1;calibrate<=1;target<=0;shuttle<=0;ext2_prepared<=0;
      end else cmd_status<=4;
     8'h03:if(idle && !drive_tripped && calibrated && cmd_value==`J280_C_TOKEN) armed<=1;else cmd_status<=4;
     8'h04:if(idle && armed && calibrated && measurement_ok && (theta>163840 || theta< -163840) && dtheta<131072 && dtheta> -131072 && dphi<32768 && dphi> -32768)start<=1;else cmd_status<=2;
     8'h05:if(idle && armed && calibrated && measurement_ok && dphi<65536 && dphi> -65536 && theta<1144 && theta> -1144 && dtheta<13108 && dtheta> -13108) capture<=1;else cmd_status<=2;
     8'h26:if(!ext2_prepared && !target_overflow && armed && reference_ready && cmd_value>= -(workspace_limit-32'sd11438) && cmd_value<=(workspace_limit-32'sd11438)) begin target<=cmd_value;target_request<=1;end else cmd_status<=2; // Ordinary targets remain available after Extension2 fades out.
     8'h07:if(armed && (state==3 || state==4) && ((cmd_value==0) || (cmd_value==1 && !ext2_prepared && reference_ready && reference_center_q))) begin shuttle<=cmd_value!=0;shuttle_request<=1;end else cmd_status<=2;
     8'h36:if(cfg_admission!=0)cmd_status<={6'd0,cfg_admission};else if(idle && !armed && cfg_ok)begin motion_configuration<=cfg_payload;workspace_limit<=$signed({10'd0,cfg_range_q});shuttle_amplitude<=$signed({14'd0,cfg_amp_rad});motion_speed<=cfg_speed;motion_speed_limit<=0;shuttle_period<={1'b0,cfg_period};ext2_prepared<=0;end else cmd_status<=3;
     8'h37:if(speed_admission!=0)cmd_status<={6'd0,speed_admission};else if(configure && !target_overflow && cmd_value>=32'sd1186 && cmd_value<=32'sd71165)begin motion_speed_limit<=cmd_value;ext2_prepared<=0;end else cmd_status<=3;
     8'h39:if(!target_overflow && !ext2_prepared && !drive_tripped &&
       (idle || state==2 || state==3 || state==4) && cmd_value>0 && !r41t1_gt32(cmd_value,ext2_travel)) ext2_amplitude<=cmd_value;else cmd_status<=3;
     8'h3a:if(!ext2_prepared && !drive_tripped && (idle || state==2 || state==3 || state==4) &&
       !r41t1_gt32(32'sd1193,cmd_value) && !r41t1_gt32(cmd_value,32'sd4294967))ext2_phase_step<=cmd_value;else cmd_status<=3;
     8'h38:if(cfg_admission!=0)cmd_status<={6'd0,cfg_admission};else if(cmd_value==1 && !ext2_prepared && armed && calibrated && measurement_ok &&
       (state==3 || state==4) && reference_ready && ext2_center_ok)begin
       ext2_prepared<=1;shuttle<=1;shuttle_request<=1;
      end else if(cmd_value==0 && ext2_prepared && armed && (state==3 || state==4))begin
       shuttle<=0;shuttle_request<=1;
      end else cmd_status<=2;
     8'h08:if(!armed)begin clear<=1;ext2_prepared<=0;end else cmd_status<=2;
     // v0.1.3: bounded drive diagnosis up to 25%; still 100 ms and PWM-limited.
     8'h09:cmd_status<=2; // no repeated drive diagnosis
     8'h10,8'h11,8'h12,8'h13,8'h15:cmd_status<=2; // frozen R01 algorithms: legacy gain writes rejected
     8'h14:if(!(configure && cmd_value==150)) cmd_status<=3; // mode limits are 150/350, not user editable
     8'h30:begin end // D02 capability/info, no movement
     8'h31:cmd_status<=2; // D03 acquisitions complete
     8'h32:if(!diag_active && !armed && idle && diag_count!=0 && cmd_value==0) begin end else cmd_status<=2;
     default:cmd_status<=1;
    endcase
   end else if(!diag_active && !dump_active && !diag_begin) begin
    // Movement is explicit serial only for this candidate; SW4 stop always active.

    if(key_press[2] && calibration_ready_q && configure && ready_q4 && valid_q4 && measurement_ok && adc==calibration_adc_q && enc==calibration_encoder_q) begin zero_raw<=adc_q4[13:4];zero_q4<=adc_q4;calibrated<=1;calibrate<=1;target<=0;shuttle<=0;ext2_prepared<=0;end
   end
  end
 end
 // Physical export and capture-bound zero are separate from the old state estimator.
 wire signed[31:0] encoder_zero,phi_physical;wire physical_saturated;
 reg signed[31:0] recording_encoder_zero;
 calibrated_position physical_position(.clk(clk_50m),.rst_n(reset_n),.tick(tick),.calibrate(calibrate),.encoder(enc),.encoder_zero(encoder_zero),.phi_physical(phi_physical),.saturated(physical_saturated));
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n)recording_encoder_zero<=0;else if(r41_passive_begin)recording_encoder_zero<=encoder_zero;
 wire priority_active;
 wire handoff_recovery;wire[511:0] observer_ext2;
 controller control(.clk(clk_50m),.rst_n(reset_n),.step(step),.frame_tick(tick),.measurement_ok(measurement_ok),.armed(armed),.sensor_gap_active(gap_active),.sensor_gap_ms(gap_ms),
 .ext2_enable(ext2_prepared),.request_start(start),.request_capture(capture),.request_clear(clear),.request_stop(stop_request),.request_jog(jog),.jog_value(jog_value),
 .theta(theta),.dtheta(dtheta),.phi(phi),.dphi(dphi),.filtered_theta_q18(filtered_theta_q18),
 .ext2_amplitude(ext2_amplitude),.ext2_phase_step(ext2_phase_step),
 .workspace_limit(workspace_limit),.full_turn_recovery(motion_configuration[28]),.shuttle_amplitude(shuttle_amplitude),.motion_speed(motion_speed),.motion_speed_limit(motion_speed_limit),.shuttle_period(shuttle_period),.target(target),.shuttle(shuttle),.request_target(target_request),.request_shuttle(shuttle_request),
 .ktheta(ktheta),.kdtheta(kdtheta),.kphi(kphi),.kdphi(kdphi),.pwm_limit(pwm_limit),.swing_gain(swing_gain),
 .state(state),.fault(fault),.command(command),.reference(reference),.ref_velocity(ref_velocity),.ext2_entry_center_q18(ext2_entry_center_q18),.result_valid(result_valid),.priority_active(priority_active),.reference_ready(reference_ready),.trajectory_running(trajectory_running),.active_limit(active_limit),.recovery_boosting(recovery_boosting),.recovery_used(recovery_used),.settled(settled),.settle_timeout(settle_timeout),.handoff_recovery(handoff_recovery),.observer_ext2(observer_ext2),.observer_state(observer_state),.observer_samples(observer_samples),.observer_ready_out(observer_ready),.observer_integral(observer_integral),.observer_lp(observer_lp),.observer_origin(capture_origin),.observer_age(observer_age));
 wire requested_enable=reset_n && !stop_request && (diag_active?diag_drive:(armed && state!=0 && state!=5 && state!=7));
 wire signed [31:0] output_command=diag_active?diag_command:command;
 wire motor_enable,drive_tripped;wire signed [31:0] safe_drive;
 drive_guard output_guard(.clk(clk_50m),.rst_n(reset_n),.enable(requested_enable),.clear(clear),
 .requested(output_command),.limit(active_limit),.allowed(motor_enable),.safe_command(safe_drive),.tripped(drive_tripped));
 wire [7:0] reported_fault=drive_tripped?8'd7:fault;
 wire [2:0] output_state=drive_tripped?3'd5:diag_drive?3'd6:state;
 wire [15:0] output_flags={physical_saturated,handoff_recovery,settle_timeout,(settled && motor_enable),recovery_used,(recovery_boosting && motor_enable),gap_active,trajectory_running,reference_ready,drive_tripped,(priority_active && motor_enable && !diag_active),motor_enable,calibration_ready,measurement_ok,calibrated,(armed || diag_drive)};
 wire m1,m2,mp;wire raw_m1,raw_m2,raw_mp;wire signed [31:0] raw_applied;
 motor_pwm motor(.clk(clk_50m),.rst_n(reset_n),.enable(motor_enable),.command(safe_drive*`J280_MOTOR_SIGN),.in1(raw_m1),.in2(raw_m2),.pwm(raw_mp),.applied_debug(raw_applied));
 // End the guard/PWM combinational chain at FFs, before the output buffers.
 // All bridge pins and applied telemetry advance together by one 20 ns clock.
 motor_output_register physical_drive(.clk(clk_50m),.rst_n(reset_n),
  .in1(raw_m1),.in2(raw_m2),.pwm(raw_mp),.applied(raw_applied),
  .out1(m1),.out2(m2),.out_pwm(mp),.out_applied(applied_debug));
 assign AN1=(MOTOR_CHANNEL==0)?m1:1'b0;assign AN2=(MOTOR_CHANNEL==0)?m2:1'b0;assign PWMA=(MOTOR_CHANNEL==0)?mp:1'b1;
 assign BN1=(MOTOR_CHANNEL==1)?m1:1'b0;assign BN2=(MOTOR_CHANNEL==1)?m2:1'b0;assign PWMB=(MOTOR_CHANNEL==1)?mp:1'b1;
 reg [3:0] led_registered;
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) led_registered<=4'b1111;
  else led_registered<=~{(state==5),(state==1),(state==2 || state==3 || state==4),armed};
 end
 assign led_n=led_registered;
 reg [4:0] log_div;reg send,sample_complete;
 // Snapshot after coherent controller commit, never at an assumed fixed delay.
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) begin log_div<=0;send<=0;sample_complete<=0;end
  else begin
   send<=0;sample_complete<=result_valid;
   if(result_valid) begin
    if(log_div==19) begin log_div<=0;send<=1;end else log_div<=log_div+1'b1;
   end
  end
 end
 wire [15:0] dropped;wire tx_ready,tx_valid;wire [7:0] tx_byte;
 reg [511:0] snapshot;
 always @* begin
  snapshot=0;
  snapshot[7:0]=8'ha5;snapshot[15:8]=8'h5a;snapshot[23:16]=5;snapshot[31:24]=64;
  snapshot[63:32]=millis;snapshot[95:64]=theta;snapshot[127:96]=dtheta;snapshot[159:128]=phi_physical;snapshot[191:160]=dphi;
  snapshot[223:192]=reference;snapshot[255:224]=ref_velocity;snapshot[287:256]=output_command;
  snapshot[303:288]={6'd0,sampled_adc};snapshot[319:304]={6'd0,zero_raw};snapshot[327:320]={5'd0,output_state};snapshot[335:328]=reported_fault;
  snapshot[351:336]=output_flags;
  snapshot[359:352]=last_seq;snapshot[367:360]=cmd_status;snapshot[383:368]=cmd_errors;
  snapshot[415:384]=sampled_enc;snapshot[447:416]=target;snapshot[463:448]=active_limit[15:0];snapshot[479:464]=illegal;snapshot[495:480]=dropped;
 end
 // Raw channels latched on estimator tick, then joined with that update's state.
 reg [9:0] sampled_adc;reg signed [31:0] sampled_enc;
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) begin sampled_adc<=0;sampled_enc<=0;end
  else if(tick) begin sampled_adc<=adc;sampled_enc<=enc;end
 end
 always @* begin
  diag_sample=0;diag_sample[31:0]=millis;diag_sample[55:48]={5'd0,output_state};diag_sample[63:56]=reported_fault;
  diag_sample[79:64]={6'd0,sampled_adc};diag_sample[95:80]={6'd0,zero_raw};diag_sample[127:96]=sampled_enc;
  diag_sample[159:128]=theta;diag_sample[191:160]=dtheta;diag_sample[223:192]=phi;diag_sample[255:224]=dphi;
  diag_sample[271:256]=output_command[15:0];diag_sample[287:272]=applied_debug[15:0];
  diag_sample[303:288]=output_flags;diag_sample[319:304]=illegal;
 end
 wire legacy_ram_write;wire[9:0]legacy_ram_address;wire[319:0]legacy_ram_data;
 d02_capture diagnosis(.clk(clk_50m),.rst_n(reset_n),.sample(sample_complete),.stop(diag_stop),.begin_capture(diag_begin),
  .requested(cmd_value),.theta(theta),.phi(phi),.encoder(sampled_enc),.adc(sampled_adc),.measurement_ok(measurement_ok),
  .eligible(idle && !armed && calibrated && pwm_limit==150 && !dump_active && !stop_request),.millis(millis),.sample_data(diag_sample),
  .read_index(dump_index[9:0]),.read_data(legacy_read_data),.shared_read(shared_legacy_data),.ram_write(legacy_ram_write),.ram_address(legacy_ram_address),.ram_data(legacy_ram_data),.ready(legacy_ready),.active(diag_active),.drive(diag_drive),
  .command(diag_command),.phase(legacy_phase),.reason(legacy_reason),.count(legacy_count),.capture_id(legacy_id),.start_ms(legacy_start),.amplitude(legacy_amplitude));

 // Passive recorder is isolated from drive selection and all controller inputs.
 // R41 reuse the same head/tail RAM for the latest automatic recovery.
 // previous_state already follows the controller every clock; this edge pulse
 // cannot be caused by ordinary HOLD/TRACK changes or a manual session.
 wire r41_passive_recovery_restart=passive_selected && passive_busy && passive_automatic &&
  armed && !diag_stop && reported_fault==0 &&
  (previous_state==2 || previous_state==3 || previous_state==4) &&
  (state==1 || state==6 || state==7);
 wire r41_passive_begin=capture || start || r41_passive_recovery_restart;
 reg passive_selected;
 wire passive_busy,passive_valid,passive_automatic,passive_read_valid;wire [13:0] passive_count;
 wire [12:0] passive_head,passive_tail;wire[11:0] passive_tail_base;wire[31:0] passive_id,passive_start,passive_total,passive_event,passive_last,passive_first_gap,passive_trigger_ms;
 wire[15:0] passive_gaps,passive_trigger;wire[7:0] passive_reason,passive_fault;wire[175:0] passive_data;
 wire[495:0] compact_info_payload,compact_data_payload;
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n)passive_selected<=0;
  else if(capture || start)passive_selected<=1;
  else if(diag_begin)passive_selected<=0;
 end
 passive_capture recorder(.legacy_select(!passive_selected),.legacy_write(legacy_ram_write),.legacy_address(legacy_ram_address),.legacy_data(legacy_ram_data),
  .legacy_read_index(read_index[9:0]),.legacy_read_data(shared_legacy_data),.legacy_read_valid(legacy_read_valid),.legacy_overrun(legacy_overrun),
  .clk(clk_50m),.rst_n(reset_n),.begin_record(r41_passive_begin),.automatic_start(start || r41_passive_recovery_restart),.sample(result_valid),.stop(diag_stop),
  .adc_q4(control_adc_q4),.adc_fine_valid(control_adc_fine_valid),.encoder(sampled_enc),.command_pwm(output_command),.applied_pwm(applied_debug),
  .flags(output_flags),.state({1'b0,output_state}),.limit(active_limit),.fault(reported_fault),.theta_q16(theta),.dtheta_q16(dtheta),
  .millis(millis),.read_index(read_index),.read_data(passive_data),.read_valid(passive_read_valid),
  .busy(passive_busy),.valid(passive_valid),.automatic_session(passive_automatic),.count(passive_count),.capture_id(passive_id),
  .head_count(passive_head),.tail_count(passive_tail),.tail_base(passive_tail_base),.total_samples(passive_total),.begin_ms(passive_start),
  .event_ms(passive_event),.last_ms(passive_last),.first_gap_ms(passive_first_gap),.trigger_ms(passive_trigger_ms),.gaps(passive_gaps),
  .trigger_index(passive_trigger),.reason(passive_reason),.freeze_fault(passive_fault));
 compact_frames compact_wire(.build(D02_BUILD),.capture_id(diag_id),.total_samples(passive_total),.begin_ms(passive_start),.event_ms(passive_event),
  .last_ms(passive_last),.first_gap_ms(passive_first_gap),.trigger_ms(passive_trigger_ms),.count(passive_count),.head_count(passive_head),.tail_count(passive_tail),
  .tail_base(passive_tail_base),.gaps(passive_gaps),.trigger_index(passive_trigger),.reason(passive_reason),.freeze_fault(passive_fault),
  .busy(passive_busy),.valid(passive_valid),.automatic_session(passive_automatic),.first_index(dump_index),.row_count(dump_rows),.row0(dump_row0),.row1(dump_row1),
  .info_payload(compact_info_payload),.data_payload(compact_data_payload));
 assign diag_read_data=legacy_read_data;
 assign diag_count=passive_selected?(passive_valid?passive_count:14'd0):{3'd0,legacy_count};
 assign diag_id=passive_selected?(passive_id | 32'h80000000):legacy_id;
 assign diag_start=passive_selected?passive_start:legacy_start;
 assign diag_amplitude=passive_selected?16'sd32767:legacy_amplitude;
 assign diag_phase=passive_selected?(passive_busy?3'd6:passive_valid?3'd4:3'd0):legacy_phase;
 assign diag_reason=passive_selected?passive_reason:legacy_reason;
 assign diag_ready=legacy_ready && !passive_busy;

 // Independent packet types: live=1, capability/status=2, frozen sample=3.
 reg [511:0] info_frame,data_frame,observer_frame,observer_saved;
 reg[511:0] ext2_b_frame,ext2_c_frame,ext2_debug_saved;
 reg observer_pending,ext2_b_pending,ext2_c_pending,ext2_checkpoint_phase;reg logger_send;
 wire logger_busy;reg live_pending,info_pending,compact_info_pending,coordinates_pending;reg[511:0] coordinates_frame,sine_settings_frame;reg sine_settings_pending;reg [3:0] info_div;
 always @* begin
  observer_frame=0;observer_frame[31:0]=32'h40045aa5;
  observer_frame[63:32]=D02_BUILD;observer_frame[95:64]=millis;observer_frame[127:96]=diag_id;
  observer_frame[351:128]=observer_state;observer_frame[367:352]=output_command[15:0];observer_frame[383:368]={6'd0,observer_age};
  observer_frame[415:384]=capture_origin;observer_frame[447:416]=observer_integral;observer_frame[479:448]=observer_lp;observer_frame[495:480]=16'h1301;
  // Type6 remains C101; types7/8 remain C831. New types9/10 are read-only
  // companions, locked with type4 into one cohort at25Hz during Extension2.
  ext2_b_frame=0;ext2_b_frame[31:0]=32'h40095aa5;ext2_b_frame[63:32]=D02_BUILD;
  ext2_b_frame[95:64]=observer_saved[95:64];ext2_b_frame[127:96]=observer_saved[127:96];
  ext2_b_frame[159:128]=ext2_debug_saved[31:0];ext2_b_frame[175:160]=observer_saved[367:352];
  ext2_b_frame[191:176]=observer_saved[383:368];ext2_b_frame[223:192]=observer_saved[415:384];
  ext2_b_frame[255:224]=ext2_debug_saved[191:160];ext2_b_frame[287:256]=ext2_debug_saved[223:192];
  ext2_b_frame[319:288]=ext2_debug_saved[255:224];ext2_b_frame[351:320]=ext2_debug_saved[159:128];
  ext2_b_frame[383:352]=ext2_debug_saved[63:32];ext2_b_frame[399:384]=16'h1901;
  ext2_b_frame[415:400]=16'he239;ext2_b_frame[447:416]=ext2_debug_saved[95:64];
  ext2_b_frame[479:448]=ext2_debug_saved[127:96];ext2_b_frame[495:480]=16'h1701;
  ext2_c_frame=0;ext2_c_frame[31:0]=32'h400a5aa5;ext2_c_frame[63:32]=D02_BUILD;
  ext2_c_frame[95:64]=observer_saved[95:64];ext2_c_frame[127:96]=observer_saved[127:96];
  ext2_c_frame[143:128]=observer_saved[367:352];ext2_c_frame[159:144]={10'd0,ext2_debug_saved[133:128]};
  ext2_c_frame[191:160]=ext2_debug_saved[287:256];ext2_c_frame[223:192]=ext2_debug_saved[319:288];
  ext2_c_frame[255:224]=ext2_debug_saved[383:352];ext2_c_frame[287:256]=ext2_debug_saved[415:384];
  ext2_c_frame[319:288]=ext2_debug_saved[351:320];ext2_c_frame[351:320]=ext2_debug_saved[447:416];
  ext2_c_frame[383:352]=ext2_debug_saved[479:448];ext2_c_frame[399:384]=16'h1a01;
  ext2_c_frame[415:400]=16'he239;ext2_c_frame[447:416]=ext2_debug_saved[511:480];ext2_c_frame[495:480]=16'h1801;
  info_frame=0;info_frame[31:0]=32'h40025aa5;info_frame[63:32]=D02_BUILD;
  info_frame[95:64]=diag_id;info_frame[103:96]={5'd0,diag_phase};info_frame[111:104]=diag_reason;
  info_frame[127:112]={2'd0,diag_count};info_frame[143:128]=passive_selected?passive_trigger:16'd50;info_frame[159:144]=16'd1;
  info_frame[175:160]=diag_amplitude;info_frame[191:176]=16'd350;info_frame[223:192]=diag_start;info_frame[255:224]=millis;
  info_frame[271:256]=`J280_C_CPR_ROUNDED;info_frame[279:272]=8'hff;info_frame[287:280]=(MOTOR_CHANNEL==1?8'd1:8'd0);
  info_frame[303:288]=passive_selected?16'd22:16'd40;info_frame[319:304]=passive_selected?16'd255:16'd159;info_frame[335:320]=passive_selected?16'd8192:16'd1024;
  // Capability bit4: ready-byte bit1 explicitly identifies a START window.
  info_frame[343:336]={6'd0,(passive_selected && passive_automatic),diag_ready};info_frame[351:344]={7'd0,dump_active};
  // F03 asynchronous diagnostic extension; time is info_frame millis, not the live sample.
  info_frame[447:416]=motion_configuration;info_frame[463:448]=16'h1401;
  info_frame[367:352]={2'd0,adc_q4};info_frame[383:368]={2'd0,zero_q4};
  info_frame[399:384]=16'hf401;info_frame[407:400]={7'd0,valid_q4};info_frame[415:408]={5'd0,ext2_consumed,ext2_prepared,ready_q4};
  info_frame[495:464]=`J280_C_PHYSICAL_RPC_Q24;
  sine_settings_frame=0;sine_settings_frame[31:0]=32'h400b5aa5;
  sine_settings_frame[63:32]=D02_BUILD;sine_settings_frame[95:64]=millis;
  sine_settings_frame[127:96]=ext2_amplitude;sine_settings_frame[159:128]=ext2_phase_step;
  sine_settings_frame[167:160]={5'd0,(shuttle&&!ext2_prepared),ext2_consumed,ext2_prepared};sine_settings_frame[495:480]=16'h5501;
  coordinates_frame=0;coordinates_frame[31:0]=32'h40065aa5;
  coordinates_frame[63:32]=D02_BUILD;coordinates_frame[95:64]=`J280_C_TOKEN;
  coordinates_frame[111:96]=`J280_C_SCHEMA;coordinates_frame[127:112]=16'd1;
  coordinates_frame[159:128]=`J280_C_CPR_NUM;coordinates_frame[191:160]=`J280_C_CPR_DEN;
  coordinates_frame[223:192]=`J280_C_LEGACY_Q16_PER_COUNT;coordinates_frame[255:224]=`J280_C_PHYSICAL_RPC_Q24;
  coordinates_frame[287:256]=`J280_C_TARGET_K_Q24;coordinates_frame[319:288]=encoder_zero;
  coordinates_frame[351:320]=recording_encoder_zero;coordinates_frame[383:352]=diag_id;
  coordinates_frame[415:384]=millis;coordinates_frame[447:416]=motion_speed_limit;coordinates_frame[479:448]=32'h3601;coordinates_frame[495:480]=16'hc101;
  data_frame=0;data_frame[31:0]=32'h40035aa5;data_frame[63:32]=diag_id;data_frame[79:64]={3'd0,dump_index};
  data_frame[415:96]=diag_read_data;
  data_frame[447:416]=D02_BUILD;
  data_frame[463:448]={2'd0,diag_count};data_frame[471:464]=diag_reason;data_frame[479:472]={5'd0,diag_phase};
  if(passive_selected)data_frame={16'd0,compact_data_payload};
 end
 // R39T1 equivalent priority encoding before a parallel payload mux.
 // The capture edge and original pending/handshake updates remain unchanged.
 wire[8:0] tx_req={(sine_settings_pending && !dump_active),dump_active && read_wait==0 && (passive_selected ? dump_stage==2 : legacy_read_valid),ext2_c_pending,ext2_b_pending,observer_pending,coordinates_pending,compact_info_pending,info_pending,(live_pending||send)};
 wire[8:0] tx_pick;
 assign tx_pick[0]=tx_req[0];
 genvar tx_i;
 generate for(tx_i=1;tx_i<9;tx_i=tx_i+1)begin:tx_priority
  assign tx_pick[tx_i]=tx_req[tx_i] && !(|tx_req[tx_i-1:0]);
 end endgenerate
 wire logger_capture=!(cmd_valid && opcode==8'h32 && !diag_active && !armed && idle && diag_count!=0 && cmd_value==0) && !logger_busy && !logger_send && (|tx_req);
 wire[511:0] tx_selected=({512{tx_pick[0]}} & snapshot) |
 ({512{tx_pick[1]}} & info_frame) | ({512{tx_pick[2]}} & {16'd0,compact_info_payload}) |
 ({512{tx_pick[3]}} & coordinates_frame) | ({512{tx_pick[4]}} & observer_saved) |
 ({512{tx_pick[5]}} & ext2_b_frame) | ({512{tx_pick[6]}} & ext2_c_frame) |
 ({512{tx_pick[7]}} & data_frame) | ({512{tx_pick[8]}} & sine_settings_frame);
 always @(posedge clk_50m or negedge reset_n) begin
  if(!reset_n) begin sine_settings_pending<=0;observer_pending<=0;observer_saved<=0;ext2_b_pending<=0;ext2_c_pending<=0;ext2_debug_saved<=0;ext2_checkpoint_phase<=0;logger_send<=0;live_pending<=0;info_pending<=0;compact_info_pending<=0;coordinates_pending<=0;info_div<=0;dump_index<=0;read_index<=0;dump_active<=0;read_wait<=0;dump_stage<=0;dump_rows<=0;dump_row0<=0;dump_row1<=0;end
  else begin
   logger_send<=0;
   if(read_wait!=0) read_wait<=read_wait-1'b1;
   if(dump_active && read_wait==0 && passive_selected)begin
    if(dump_stage==0 && passive_read_valid)begin
     dump_row0<=passive_data;
     if({1'b0,dump_index}+14'd1>=diag_count)begin dump_rows<=1;dump_row1<=0;dump_stage<=2;end
     else begin read_index<=dump_index+13'd1;read_wait<=2;dump_stage<=1;end
    end else if(dump_stage==1 && passive_read_valid)begin dump_row1<=passive_data;dump_rows<=2;dump_stage<=2;end
   end
   if(!armed)ext2_checkpoint_phase<=0;
   if(send && armed && (state==2||state==3||state==4||state==6))begin
    if(ext2_prepared)begin
     ext2_checkpoint_phase<=~ext2_checkpoint_phase;
     // Preserve an entire cohort until sent. Never replace only one member.
     if(!ext2_checkpoint_phase && !observer_pending && !ext2_b_pending && !ext2_c_pending)begin
      observer_saved<=observer_frame;observer_pending<=1;
      ext2_debug_saved<={observer_ext2[511:160],16'd0,(fault!=0),observer_ready,observer_samples,observer_ext2[133:128],observer_ext2[127:0]};ext2_b_pending<=1;ext2_c_pending<=1;
     end
    end else if(!ext2_b_pending && !ext2_c_pending)begin observer_saved<=observer_frame;observer_pending<=1;end
   end
   if(send) begin live_pending<=1;if(info_div==9) begin info_div<=0;info_pending<=1;compact_info_pending<=passive_selected;coordinates_pending<=1;sine_settings_pending<=1;end else info_div<=info_div+1'b1;end
   if(cmd_valid && opcode==8'h30)begin info_pending<=1;compact_info_pending<=passive_selected;coordinates_pending<=1;sine_settings_pending<=1;end
   if(cmd_valid && opcode==8'h32 && !diag_active && !armed && idle && diag_count!=0 && cmd_value==0) begin
    dump_index<=0;read_index<=0;dump_active<=1;read_wait<=2;dump_stage<=0;dump_rows<=0;info_pending<=1;compact_info_pending<=passive_selected;
   end else if(!logger_busy && !logger_send) begin
    if(|tx_req)begin logger_send<=1;end
    if(live_pending || send) begin live_pending<=0;end
    else if(info_pending) begin info_pending<=0;end
    else if(compact_info_pending)begin compact_info_pending<=0;end
    else if(coordinates_pending)begin coordinates_pending<=0;end
    else if(observer_pending)begin observer_pending<=0;end
    else if(ext2_b_pending)begin ext2_b_pending<=0;end
    else if(ext2_c_pending)begin ext2_c_pending<=0;end
    else if(!dump_active && sine_settings_pending)begin sine_settings_pending<=0;end
    else if(dump_active && read_wait==0 && (passive_selected ? dump_stage==2 : legacy_read_valid)) begin
     
     if({1'b0,dump_index}+(passive_selected?{12'd0,dump_rows}:14'd1)>=diag_count) begin dump_active<=0;info_pending<=1;compact_info_pending<=passive_selected;end
     else begin dump_index<=dump_index+(passive_selected?{11'd0,dump_rows}:13'd1);read_index<=dump_index+(passive_selected?{11'd0,dump_rows}:13'd1);read_wait<=2;dump_stage<=0;end
    end
   end
  end
 end
 telemetry #(.PRECAPTURE(1)) logger(.capture(logger_capture),.clk(clk_50m),.rst_n(reset_n),.send(logger_send),.tx_ready(tx_ready),.snapshot(tx_selected),.busy(logger_busy),.tx_valid(tx_valid),.tx_data(tx_byte),.dropped(dropped));
 uart_tx #(.DIV(UART_DIV)) tx_inst(.clk(clk_50m),.rst_n(reset_n),.valid(tx_valid),.data(tx_byte),.ready(tx_ready),.tx(uart_tx));
endmodule

// One bounded command stage, not a queue. A newer command or SW4 cancels
// an uncommitted configuration; no cancelled configuration can appear later.
// Admission is snapshotted so a command originally rejected while active
// cannot become accepted merely because the board became idle one clock later.
module motion_config_stage(input wire clk,rst_n,stop_level,new_command,in_valid,
 input wire[7:0]in_seq,in_opcode,input wire signed[31:0]in_value,
 input wire in_overflow,input wire[1:0]admission,
 output reg out_valid,output reg[7:0]out_seq,out_opcode,
 output reg signed[31:0]out_value,output reg out_overflow,
 output reg[31:0]configuration,output reg[1:0]saved_admission);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin out_valid<=0;out_seq<=0;out_opcode<=0;out_value<=0;out_overflow<=0;configuration<=0;saved_admission<=3;end
  else begin
   out_valid<=0;
   if(!stop_level && !new_command && in_valid)begin
    out_valid<=1;out_seq<=in_seq;out_opcode<=in_opcode;out_value<=in_value;out_overflow<=in_overflow;
    if(in_opcode==8'h36 || in_opcode==8'h38)begin configuration<=in_value;saved_admission<=admission;end
   end
  end
 end
endmodule
`default_nettype wire
