`default_nettype none
// D03: fixed 0/+/-100/+/-150 permille; 1 degree start / 10 degree travel.
// Single-shot, autonomous 1 kHz diagnosis. Never runs the balancing controller.
module d02_capture(input wire clk,rst_n,sample,stop,begin_capture,
 input wire signed [31:0] requested,theta,phi,encoder,
 input wire [9:0] adc,input wire measurement_ok,eligible,
 input wire [31:0] millis,input wire [319:0] sample_data,
 input wire [9:0] read_index,output wire [319:0] read_data,input wire [319:0] shared_read,output wire ram_write,output wire[9:0] ram_address,output wire[319:0] ram_data,
 output wire ready,active,drive,output reg signed [31:0] command,
 output reg [2:0] phase,output reg [7:0] reason,
 output reg [10:0] count,output reg [31:0] capture_id,start_ms,
 output reg signed [15:0] amplitude);

 reg [8:0] settled;reg [9:0] low_adc,high_adc,base_adc;
 reg signed [31:0] low_enc,high_enc,origin_enc,origin_phi;
 reg [6:0] pulse_age;
 reg [16:0] watchdog;
 wire signed [31:0] amplitude_wide={{16{amplitude[15]}},amplitude};
 wire posture=(theta>200168 || theta< -200168) && phi<=1143 && phi>= -1143;
 wire [9:0] next_lo=adc<low_adc?adc:low_adc;
 wire [9:0] next_hi=adc>high_adc?adc:high_adc;
 wire signed [31:0] next_el=encoder<low_enc?encoder:low_enc;
 wire signed [31:0] next_eh=encoder>high_enc?encoder:high_enc;
 wire signed [32:0] enc_span=$signed({next_eh[31],next_eh})-$signed({next_el[31],next_el});
 wire span_ok=({1'b0,next_hi}-{1'b0,next_lo}<=11'd4) && enc_span<=33'sd1;
 assign active=(phase==1 || phase==2 || phase==3);
 assign ready=eligible && !active && measurement_ok && posture && settled>=300 && span_ok;
 assign drive=(phase==2) && amplitude!=0 && !stop && measurement_ok;
 wire signed [32:0] travel=$signed({phi[31],phi})-$signed({origin_phi[31],origin_phi});
 wire signed [32:0] enc_move=$signed({encoder[31],encoder})-$signed({origin_enc[31],origin_enc});
 wire pre_moved=({1'b0,adc}>{1'b0,base_adc}+11'd4 || {1'b0,adc}+11'd4<{1'b0,base_adc} || enc_move>1 || enc_move< -1);
 wire abort_now=stop || !measurement_ok || watchdog>=99999 || travel>11438 || travel< -11438 || (phase==1 && pre_moved);
 // Freeze retained memory on completion/abort. It can be downloaded repeatedly.
 assign read_data=shared_read;
 assign ram_write=rst_n && sample && active && !abort_now && count<1024;
 assign ram_address=count[9:0];
 assign ram_data={sample_data[319:48],5'd0,count,sample_data[31:0]};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   phase<=0;reason<=0;count<=0;command<=0;capture_id<=0;start_ms<=0;amplitude<=0;
   settled<=0;low_adc<=0;high_adc<=0;low_enc<=0;high_enc<=0;origin_enc<=0;origin_phi<=0;base_adc<=0;pulse_age<=0;watchdog<=0;
  end else begin
   if(!active || sample) watchdog<=0;else if(watchdog<100000) watchdog<=watchdog+1'b1;
   if(!eligible || active || !measurement_ok || !posture) settled<=0;
   else if(sample) begin
    if(settled==0 || !span_ok) begin settled<=1;low_adc<=adc;high_adc<=adc;low_enc<=encoder;high_enc<=encoder;end
    else begin
     if(settled<300) settled<=settled+1'b1;
     low_adc<=next_lo;high_adc<=next_hi;low_enc<=next_el;high_enc<=next_eh;
    end
   end
   if(active && abort_now) begin
    phase<=5;command<=0;
    reason<=stop?8'd1:!measurement_ok?8'd2:watchdog>=99999?8'd5:(phase==1 && pre_moved)?8'd4:8'd3;
   end else if(begin_capture && ready && !stop && (requested==100 || requested== -100 || requested==150 || requested== -150 || requested==0)) begin
    phase<=1;reason<=0;count<=0;capture_id<=capture_id+1'b1;command<=0;
    amplitude<=requested[15:0];origin_enc<=encoder;origin_phi<=phi;base_adc<=adc;pulse_age<=0;start_ms<=millis;
   end else if(sample && active) begin
    if(count==0) start_ms<=millis;
    count<=count+1'b1;
    if(phase==1 && count==49) begin
     phase<=2;pulse_age<=0;command<=amplitude>0?32'sd20:amplitude<0?-32'sd20:32'sd0;
    end else if(phase==2) begin
     if(pulse_age==99) begin phase<=3;command<=0;end
     else begin
      pulse_age<=pulse_age+1'b1;
      if(amplitude>0 && command<amplitude_wide)
       command<=(command+32'sd20>amplitude_wide) ? amplitude_wide : command+32'sd20;
      if(amplitude<0 && command>amplitude_wide)
       command<=(command-32'sd20<amplitude_wide) ? amplitude_wide : command-32'sd20;
     end
    end
    if(count==1023) begin phase<=4;command<=0;end
   end
  end
 end
endmodule
`default_nettype wire
