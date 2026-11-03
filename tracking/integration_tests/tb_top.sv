`include "j280_config.vh"
`timescale 1ns/1ps
module tb_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;reg [3:0] keys=15;reg [9:0] adc=512;
 wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire [3:0] leds;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(adc),
 .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(leds));
 wire byte_valid,byte_error;wire [7:0] data;
 uart_rx monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(byte_valid),.error(byte_error),.data(data));
 reg [351:0] saved_memory [0:4095]; reg [12:0] saved_count;reg [31:0] saved_id;
 integer fd,bytes=0,checks=0,heartbeat_before=0;
 always @(posedge clk) if(byte_valid) begin $fwrite(fd,"%c",data);bytes<=bytes+1;end
 always @(posedge clk) if(rst && (an1 || an2 || !pa)) $fatal(1,"unused A channel is not coasting");
 always @(negedge clk) if(rst && dut.sample_complete) begin
  if(dut.estimator.stage!=0 || dut.control.phase!=0)$fatal(1,"telemetry before completed estimator/controller");
  if(dut.snapshot[303:288]!={6'd0,dut.sampled_adc} || $signed(dut.snapshot[415:384])!=dut.sampled_enc)$fatal(1,"telemetry incoherent raw channels");
 end
 task clocks(input integer n);repeat(n) @(negedge clk);endtask
 task uart_byte(input [7:0] d);integer j;begin
 rx=0;clocks(434);for(j=0;j<8;j=j+1)begin rx=d[j];clocks(434);end rx=1;clocks(434);end endtask
 function automatic [15:0] crcbyte(input [15:0] c0,input [7:0] d);
 reg [15:0] c;integer j;begin c=c0^{d,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;return c;end endfunction
 task send(input [7:0] seq,input [7:0] op,input [31:0] v,input bit bad);
 reg [15:0] crc;integer j;begin
 crc=crcbyte(crcbyte(16'hffff,seq),op);uart_byte(8'haa);uart_byte(8'h55);uart_byte(seq);uart_byte(op);
 for(j=0;j<4;j=j+1)begin uart_byte(v[j*8+:8]);crc=crcbyte(crc,v[j*8+:8]);end
 uart_byte(crc[7:0]);uart_byte(crc[15:8]^(bad?8'h01:8'h00));clocks(100);
 end endtask
 initial begin
  fd=$fopen("top_telemetry.bin","wb");if(!fd)$fatal(1,"cannot open telemetry output");
  clocks(3);rst=1;clocks(5000);
  if(an1||an2||!pa||bn1||bn2||!pb||dut.armed)$fatal(1,"boot drive");checks++;
  if(dut.D02_BUILD!=32'h4a070228)$fatal(1,"R13 r3 version");checks++;
  send(1,3,32'h4a323830,0);if(dut.armed||dut.cmd_status!=4)$fatal(1,"arm before calibration");checks++;
  clocks(10200000);if(bytes<256)$fatal(1,"disarmed IDLE heartbeat missing");checks++;send(2,2,0,0);if(!dut.calibrated||dut.zero_raw!=512)$fatal(1,"calibration");checks++;
  send(3,3,32'h4a323830,1);if(dut.armed||dut.cmd_errors!=1)$fatal(1,"bad CRC armed system");checks++;
  for(integer op=16;op<=19;op++)begin send(op[7:0],op[7:0],0,0);if(dut.cmd_status!=2)$fatal(1,"old gain accepted");checks++;end
  send(20,21,800,0);if(dut.cmd_status!=2)$fatal(1,"old swing gain accepted");checks++;
  send(21,20,350,0);if(dut.cmd_status!=3||dut.pwm_limit!=150)$fatal(1,"user limit accepted");checks++;
  send(23,20,200,0);if(dut.cmd_status!=3||dut.pwm_limit!=150)$fatal(1,"stale R08 limit accepted");checks++;
  send(22,20,150,0);if(dut.cmd_status!=0)$fatal(1,"fixed mode limit rejected");checks++;
  send(23,9,100,0);if(dut.cmd_status!=2||dut.jog)$fatal(1,"old jog enabled");checks++;
  send(24,49,100,0);if(dut.cmd_status!=2||dut.diag_begin)$fatal(1,"D03 diagnosis enabled");checks++;
  send(25,6,5719,0);if(dut.cmd_status!=2)$fatal(1,"position accepted beforehold");checks++;
  send(26,7,1,0);if(dut.cmd_status!=2||dut.shuttle)$fatal(1,"trajectory accepted beforehold");checks++;
  send(70,22,32'd41981630,0);if(dut.cmd_status!=0 || dut.workspace_limit!=217360)$fatal(1,"config valid");checks++;
  send(71,22,32'hffffffff,0);if(dut.cmd_status!=3 || dut.workspace_limit!=217360)$fatal(1,"config invalid mutated");checks++;
  send(27,3,32'h4a323830,0);if(!dut.armed)$fatal(1,"valid arm");checks++;
  send(72,22,32'd41981500,0);if(dut.cmd_status!=3 || dut.workspace_limit!=217360)$fatal(1,"armed config mutated");checks++;
  send(28,4,0,0);if(dut.cmd_status!=2)$fatal(1,"auto start fromupright");checks++;
  force dut.theta=196608;send(28,4,0,0);if(dut.cmd_status!=0 || (dut.state!=1 && dut.state!=6))$fatal(1,"auto start from down failed");send(28,1,0,0);release dut.theta;send(28,3,32'h4a323830,0);checks++;
  send(29,5,0,0);clocks(100000);if(dut.state!=2)$fatal(1,"manualcapture");checks++;
  // Timer acceleration only: real 2000-tick handoff is tested in tb_supervisor.
  force dut.control.hold_ticks=1999;clocks(1);release dut.control.hold_ticks;clocks(100000);
  if(dut.state!=3||!dut.reference_ready||dut.active_limit!=150)$fatal(1,"tracking handoff");checks++;
  send(30,6,210000,0);if(dut.cmd_status!=2||dut.target!=0)$fatal(1,"invalid target accepted");checks++;
  send(31,6,5719,0);if(dut.cmd_status!=0||dut.target!=5719)$fatal(1,"bounded target rejected");clocks(100000);if(dut.state!=4)$fatal(1,"position did not enter TRACK");checks++;
  send(32,7,1,0);if(dut.cmd_status!=2)$fatal(1,"trajectory accepted during position");checks++;
  clocks(11000000);
  // Actual repeatable policy -> observer -> dynamic guard -> UART.
  force dut.theta=7864;force dut.filtered_theta_q18=31457;force dut.dtheta=32768;
  force dut.phi=0;force dut.dphi=0;
  clocks(2500000);
  if(dut.active_limit!=350 || !dut.recovery_boosting || !dut.recovery_used || dut.drive_tripped || !dut.motor_enable)$fatal(1,"integrated recovery pulse missing");checks++;
  force dut.theta=0;force dut.filtered_theta_q18=0;force dut.dtheta=0;
  clocks(8000000);
  if(dut.active_limit!=150 || dut.recovery_boosting || !dut.recovery_used || dut.drive_tripped)$fatal(1,"recovered cap failed to return");checks++;
  release dut.theta;release dut.filtered_theta_q18;release dut.dtheta;release dut.phi;release dut.dphi;
  send(33,1,0,0);clocks(10);if(dut.armed||bn1||bn2||!pb||dut.command)$fatal(1,"serialSTOP failed");checks++;
  send(34,3,32'h4a323830,0);send(35,5,0,0);clocks(100000);
  force dut.control.hold_ticks=1999;clocks(1);release dut.control.hold_ticks;clocks(100000);
  send(50,7,1,0);clocks(100000);if(dut.cmd_status!=0 || !dut.trajectory_running || dut.state!=4)$fatal(1,"valid shuttle failed");checks++;
  send(51,7,0,0);clocks(100000);if(dut.cmd_status!=0 || dut.trajectory_running)$fatal(1,"smooth shuttle exit failed");checks++;
  adc=515;clocks(500000);
  if(dut.command==0||!dut.motor_enable||(!bn1&&!bn2))$fatal(1,"mapped XH1 bridge neverdriven");checks++;
  force dut.control.command=-32'sd2097058;clocks(2);release dut.control.command;clocks(4);
  if(dut.motor_enable||!dut.drive_tripped||dut.reported_fault!=7||bn1||bn2||!pb)$fatal(1,"independent guard failed");checks++;
  heartbeat_before=bytes;clocks(1500000);if(bytes<=heartbeat_before+16)$fatal(1,"fault/disarmed heartbeat missing");checks++;
  if(!dut.passive_valid || dut.passive_reason!=3)$fatal(1,"fault tail did not freeze");
  saved_count=dut.passive_count;saved_id=dut.passive_id;
  for(integer z=0;z<4096;z++)saved_memory[z]=dut.recorder.memory[z];
  send(36,1,0,0);send(37,8,0,0);clocks(100);if(dut.drive_tripped||dut.fault!=0)$fatal(1,"guard clear");checks++;
  if(saved_count!=dut.passive_count || saved_id!=dut.passive_id)$fatal(1,"clear changed tail metadata");
  for(integer z=0;z<4096;z++)if(saved_memory[z]!==dut.recorder.memory[z])$fatal(1,"clear changed tail RAM");checks++;
  adc=512;clocks(3000000);send(38,3,32'h4a323830,0);send(39,5,0,0);clocks(100000);adc=515;clocks(500000);
  keys[3]=0;clocks(4);if(bn1||bn2||!pb||dut.armed)$fatal(1,"SW4");checks++;
  keys[3]=1;clocks(100);send(40,1,0,0);send(41,48,0,0);clocks(1500000);
  if(!dut.passive_valid || dut.diag_count==0 || dut.diag_id[31]!=1'b1)$fatal(1,"passive capture not frozen");checks++;
  send(42,50,0,0);if(dut.cmd_status!=0)$fatal(1,"passive read rejected");clocks(5000000);
  if(bytes<64*10)$fatal(1,"missing telemetry");
  $fclose(fd);$display("PASS R01 top: checks=%0d UART CRC/calibration/version/oldgainrejection/bounded-position/shuttle-and-auto-rejection/SW4/XH1/outputguard/telemetry bytes=%0d",checks,bytes);$finish;
 end
 initial begin #1500000000;$fatal(1,"top test timeout");end
endmodule
