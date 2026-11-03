`include "j280_config.vh"
`timescale 1ns/1ps
module tb_tail_uart;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;reg [3:0] keys=15;reg [9:0] adc=512;
 wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire [3:0] leds;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(adc),
 .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(leds));
 wire byte_valid,byte_error;wire [7:0] data;
 uart_rx #(.DIV(16)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(byte_valid),.error(byte_error),.data(data));
 integer fd,bytes=0,checks=0,heartbeat_before=0;
 always @(posedge clk) if(dut.tx_valid && dut.tx_ready) begin $fwrite(fd,"%c",dut.tx_byte);bytes<=bytes+1;end
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
  force dut.tx_ready=1;
  fd=$fopen("tail_uart.bin","wb");clocks(3);rst=1;clocks(5000);
  // Inject coherent synthetic samples into the production recorder through its top-level input.
  // Keep actual controller idle/disarmed; verify read bus, wrap, metadata and serial framing.
  force dut.capture=1;clocks(1);release dut.capture;clocks(1);
  for(integer n=0;n<4300;n++) begin
   force dut.control_adc_raw=n%1024;force dut.control_adc_fine_valid=n%2;force dut.filtered_theta_q18=(n-2200)*123;force dut.control_adc_q4=(n*37)%16384;force dut.ref_velocity=(n-2200)*16;force dut.active_limit=150+n%51;force dut.reference=n-2200;
   force dut.result_valid=1;force dut.millis=32'd10000+n;
   force dut.diag_sample={192'd0,32'(n),32'd0,8'd0,8'd3,16'd0,32'(10000+n)};
   clocks(1);force dut.result_valid=0;clocks(1);
  end
  force dut.control_adc_raw=1023;force dut.control_adc_fine_valid=1;force dut.filtered_theta_q18=-823549;force dut.control_adc_q4=16383;force dut.ref_velocity=-34315;force dut.reference=-12345;force dut.active_limit=200;force dut.result_valid=0;force dut.millis=32'd14300;
  force dut.diag_sample={192'd0,32'd4300,32'd0,8'd4,8'd5,16'd0,32'd14300};
  force dut.result_valid=1;clocks(1);force dut.result_valid=0;clocks(1);
  release dut.control_adc_raw;release dut.control_adc_fine_valid;release dut.filtered_theta_q18;release dut.control_adc_q4;release dut.ref_velocity;release dut.reference;release dut.diag_sample;release dut.millis;release dut.result_valid;release dut.active_limit;
  clocks(4); // timestamp finalization is three clocks; wait for valid
  if(!dut.passive_valid || dut.diag_count!=4096 || dut.passive_reason!=3) $fatal(1,"tail fault freeze/count");
  send(43,48,0,0);clocks(500000);
  send(44,50,0,0);if(dut.cmd_status!=0)$fatal(1,"tail READ rejected");
  wait(!dut.dump_active);clocks(500000);
  if(dut.armed||dut.command||bn1||bn2||!pb)$fatal(1,"reader drove motor");
  $fclose(fd);$display("PASS tail4096 UART wrap/fault/index/count/idle/no motion bytes=%0d",bytes);$finish;
 end
 initial begin #2000000000;$fatal(1,"tail test timeout");end
endmodule
