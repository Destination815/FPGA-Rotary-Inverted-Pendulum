`default_nettype none
// A single 50 MHz domain; ADC clock is an OUTPUT, never an internal fabric clock.
module adc_parallel(input wire clk, rst_n, input wire [9:0] data,
 output reg adc_clk, output wire oe_n, output reg [9:0] average, output reg ready,
 output reg [13:0] average_q4, output reg ready_q4,valid_q4);
 reg [4:0] phase; reg [4:0] warmup; reg [3:0] count;
 reg [13:0] sum;
 // Anti-alias before 1 kHz sampling: 512 conversions at 2.5 MHz.
 // Retain four fractional ADC-code bits; do not re-quantize the mean to 10 bits.
 reg [8:0] fine_count;
 reg [18:0] fine_sum;
 reg [9:0] fine_low,fine_high;
 reg fine_good;
 wire [19:0] fine_total={1'b0,fine_sum}+{10'd0,sample_data};
 wire [9:0] fine_next_low=(sample_data<fine_low)?sample_data:fine_low;
 wire [9:0] fine_next_high=(sample_data>fine_high)?sample_data:fine_high;
 wire [10:0] fine_span={1'b0,fine_next_high}-{1'b0,fine_next_low};
 wire fine_sample_good=sample_data>10'd3 && sample_data<10'd1020;
 // The external bus has exactly one sequential destination. Keep this name:
 // the SDC ADC-only timing budget terminates at these 10 input registers.
 (* syn_keep=1, syn_preserve=1 *) reg [9:0] sample_data;
 wire [14:0] total = {1'b0,sum}+{5'd0,sample_data};
 // Registered 2.5 MHz output: no combinational clock decoding glitches.
 assign oe_n=1'b0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin adc_clk<=0;phase<=0;warmup<=0;count<=0;sum<=0;average<=0;ready<=0;sample_data<=0;average_q4<=0;ready_q4<=0;valid_q4<=0;fine_count<=0;fine_sum<=0;fine_low<=0;fine_high<=0;fine_good<=0;end
  else begin
   phase <= (phase==19)?5'd0:phase+1'b1;
   if(phase==19) adc_clk<=1;else if(phase==9) adc_clk<=0;
   // 3PA1030 Figure 22: outputs update after the ADC FALLING edge.
   // Phase 15: lock the bus 120 ns after that falling edge; the next output
   // change is another 280 ns away (before external propagation delays).
   // T_OD=25 ns is typical only; see the explicit compile-only SDC budget.
   if(phase==15) sample_data<=data;
   // Accumulation uses the registered sample on the NEXT 50 MHz edge.
   // This preserves every sampled word and the 16-point averaging arithmetic;
   // average/ready become visible exactly one 20 ns cycle later than T01.
   if(phase==16) begin
    if(warmup>=16) begin
     if(fine_count==0) begin fine_low<=sample_data;fine_high<=sample_data;fine_good<=fine_sample_good;end
     else begin fine_low<=fine_next_low;fine_high<=fine_next_high;fine_good<=fine_good && fine_sample_good;end
     if(fine_count==9'd511) begin
      average_q4<=fine_total[18:5];ready_q4<=1;
      valid_q4<=fine_good && fine_sample_good && fine_span<=11'd32;
      fine_count<=0;fine_sum<=0;
     end else begin fine_sum<=fine_total[18:0];fine_count<=fine_count+1'b1;end
    end
    if(warmup<16) warmup<=warmup+1'b1;
    else if(count==15) begin average<=total[13:4];sum<=0;count<=0;ready<=1;end
    else begin sum<=total[13:0];count<=count+1'b1;end
   end
  end
 end
endmodule

module encoder_quad(input wire clk,rst_n, input wire a,b,
 output reg signed [31:0] count, output reg [15:0] illegal);
 (* syn_keep=1 *) reg [1:0] sync_a,sync_b;
 reg [1:0] candidate,stable,previous; reg [3:0] filter_count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync_a<=0;sync_b<=0;candidate<=0;stable<=0;previous<=0;filter_count<=0;count<=0;illegal<=0;end
  else begin
   sync_a<={sync_a[0],a};sync_b<={sync_b[0],b};
   if({sync_a[1],sync_b[1]}!=candidate) begin candidate<={sync_a[1],sync_b[1]};filter_count<=0;end
   else if(filter_count<7) filter_count<=filter_count+1'b1;
   else stable<=candidate;
   previous<=stable;
   case({previous,stable})
    4'b0010,4'b1011,4'b1101,4'b0100:count<=count+32'sd1;
    4'b0001,4'b0111,4'b1110,4'b1000:count<=count-32'sd1;
    4'b0011,4'b1100,4'b0110,4'b1001:if(illegal!=16'hffff) illegal<=illegal+1'b1;
    default:begin end
   endcase
  end
 end
endmodule

// Calibration only: qualify a bounded position envelope at the 1 kHz sample rate.
// Unlike differentiated velocity, a 1-code ADC toggle does not restart settling.
// The envelope covers ALL samples since its last restart, not just endpoints.
module calibration_guard(input wire clk,rst_n,step,enabled,measurement_ok,
 input wire [9:0] adc,input wire signed [31:0] encoder,output wire ready);
 reg [7:0] samples;
 reg [9:0] adc_low,adc_high;
 reg signed [31:0] enc_low,enc_high;
 wire [9:0] next_adc_low=(adc<adc_low)?adc:adc_low;
 wire [9:0] next_adc_high=(adc>adc_high)?adc:adc_high;
 wire signed [31:0] next_enc_low=(encoder<enc_low)?encoder:enc_low;
 wire signed [31:0] next_enc_high=(encoder>enc_high)?encoder:enc_high;
 wire [10:0] adc_span={1'b0,next_adc_high}-{1'b0,next_adc_low};
 wire signed [32:0] enc_span=$signed({next_enc_high[31],next_enc_high})-$signed({next_enc_low[31],next_enc_low});
 wire valid=enabled && measurement_ok && adc>=10'd128 && adc<=10'd895;
 wire within_span=adc_span<=11'd4 && enc_span<=33'sd2;
 // Current inputs also gate ready so an out-of-envelope sample cannot use a
 // stale ready flag between the sample edge and a serial/key calibration.
 assign ready=valid && samples>=8'd200 && within_span;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin samples<=0;adc_low<=0;adc_high<=0;enc_low<=0;enc_high<=0;end
  else if(!valid) samples<=0;
  else if(step) begin
   if(samples==0 || !within_span) begin
    samples<=1;adc_low<=adc;adc_high<=adc;enc_low<=encoder;enc_high<=encoder;
   end else begin
    if(samples<200) samples<=samples+1'b1;
    adc_low<=next_adc_low;adc_high<=next_adc_high;enc_low<=next_enc_low;enc_high<=next_enc_high;
   end
  end
 end
endmodule

module key_input #(parameter integer DEBOUNCE=500000)(input wire clk,rst_n,
 input wire [3:0] keys_n, output reg [3:0] pressed, output wire stop_level);
 reg [3:0] sync1,sync2,stable; reg [19:0] counts[0:3]; integer i;
 assign stop_level=!sync2[3]; // Stop is synchronized but deliberately NOT debounced.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync1<=15;sync2<=15;stable<=15;pressed<=0;for(i=0;i<4;i=i+1) counts[i]<=0;end
  else begin
   sync1<=keys_n;sync2<=sync1;pressed<=0;
   for(i=0;i<4;i=i+1) begin
    if(sync2[i]==stable[i]) counts[i]<=0;
    else if(counts[i]==DEBOUNCE[19:0]-20'd1) begin
     counts[i]<=0;stable[i]<=sync2[i];pressed[i]<=!sync2[i];
    end else counts[i]<=counts[i]+1'b1;
   end
  end
 end
endmodule

module motor_pwm #(parameter integer PERIOD=2500)(input wire clk,rst_n,enable,
 input wire signed [31:0] command, output wire in1,in2,pwm,output wire signed [31:0] applied_debug);
 reg [11:0] counter;reg signed [31:0] applied;
 assign applied_debug=applied;
 wire signed [31:0] bounded=command>350?32'sd350:command< -350?-32'sd350:command;
 wire [31:0] magnitude=applied<0 ? $unsigned(-applied):$unsigned(applied);
 // At the deployed PERIOD=2500, floor(|u|*2500/1000)=floor(|u|*5/2).
 // applied is clamped to +/-350; phase-specific guard limits balance to200. Keep the exact waveform while avoiding a
 // 32-bit divide on the combinational path to the physical PWM output.
 wire [10:0] five_magnitude={2'b00,magnitude[8:0]}+{magnitude[8:0],2'b00};
 wire [31:0] threshold;
 generate if(PERIOD==2500) begin: fixed_period
  assign threshold={22'd0,five_magnitude[10:1]};
 end else begin: parameterized_period
  assign threshold=(magnitude*PERIOD)/1000;
 end endgenerate
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin counter<=0;applied<=0;end
  else begin
   counter<=(counter==PERIOD[11:0]-12'd1)?12'd0:counter+1'b1;
   if(!enable) applied<=0;
   else if(counter==PERIOD[11:0]-12'd1) begin
    // One complete PWM period coasting between opposite polarities.
    if((applied>0 && bounded<0)||(applied<0 && bounded>0)) applied<=0;
    else applied<=bounded;
   end
  end
 end
 // TB6612: IN1=IN2=0,PWM=1 -> high impedance STOP (STBY is hardwired high).
 assign in1=enable && (applied>0);
 assign in2=enable && (applied<0);
 assign pwm=(!enable || applied==0)?1'b1:({20'd0,counter}<threshold);
endmodule
// T03: one coherent output boundary. Async reset still coasts immediately.
// A synchronous disable/guard trip reaches the physical bridge on the next
// 50 MHz rising edge, not at the next PWM period or 1 ms control tick.
module motor_output_register(input wire clk,rst_n,in1,in2,pwm,
 input wire signed [31:0] applied,
 output reg out1,out2,out_pwm,output reg signed [31:0] out_applied);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin out1<=0;out2<=0;out_pwm<=1;out_applied<=0;end
  else begin out1<=in1;out2<=in2;out_pwm<=pwm;out_applied<=applied;end
 end
endmodule
`default_nettype wire
