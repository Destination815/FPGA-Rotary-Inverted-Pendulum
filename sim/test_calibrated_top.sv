`timescale 1ns/1ps
`include "j280_coordinates.vh"
module test_calibrated_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;reg[3:0]keys=15;
 wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));
 wire bv,be;wire[7:0]bd;
 uart_rx #(.DIV(434)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(bv),.error(be),.data(bd));
 integer fd,bytes=0,checks=0,stop_edge_checks=0;reg[7:0] seq=0;
 integer sample_checks=0;reg signed[31:0] delta_check;reg signed[63:0] q_check;
 always @(negedge clk)begin
  if(rst && dut.sample_complete && !dut.calibrate)begin
   delta_check=dut.sampled_enc-dut.encoder_zero;
   q_check=$signed(delta_check)*64'sd97763;
   q_check=q_check<0?-(((-q_check)+128)>>>8):((q_check+128)>>>8);
   if(dut.phi_physical!==q_check[31:0])$fatal(1,"incoherent position/encoder snapshot ms%0d sampled%0d expected%0d phi%0d",dut.millis,dut.sampled_enc,q_check,dut.phi_physical);
   sample_checks++;
  end
 end
 localparam[31:0] CFG=32'd42505918|32'h10000000;
 always @(posedge clk)begin
  if(bv)begin $fwrite(fd,"%c",bd);bytes++;end
  if(rst && be)$fatal(1,"UART framing error");
 end
 always @(posedge clk)begin
  if(rst && dut.wire_cmd_valid && dut.wire_opcode==1 && !dut.stop_request)begin
   #1;if(dut.armed || dut.last_seq!==dut.wire_cmd_seq || dut.cmd_status!=0)$fatal(1,"validSTOP edge/ACK delayed");
   stop_edge_checks++;
  end
 end
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task stopped;begin if(bn1||bn2||!pb||an1||an2||!pa||dut.start||dut.capture)$fatal(1,"unexpected movement");end endtask
 task byte_in(input[7:0]v);integer j;begin rx=0;clocks(434);for(j=0;j<8;j++)begin rx=v[j];clocks(434);end rx=1;clocks(434);end endtask
 function automatic[15:0]crcbyte(input[15:0]old,input[7:0]v);reg[15:0]c;integer j;begin c=old^{v,8'd0};for(j=0;j<8;j++)c=c[15]?(c<<1)^16'h1021:c<<1;return c;end endfunction
 task command(input[7:0]op,input[31:0]v,input[7:0]status);reg[15:0]c;integer j;begin
  seq++;c=crcbyte(crcbyte(16'hffff,seq),op);byte_in(8'haa);byte_in(8'h55);byte_in(seq);byte_in(op);
  for(j=0;j<4;j++)begin byte_in(v[j*8+:8]);c=crcbyte(c,v[j*8+:8]);end byte_in(c[7:0]);byte_in(c[15:8]);clocks(100);
  if(dut.last_seq!=seq || dut.cmd_status!=status)$fatal(1,"op%h seq%0d status%0d expected%0d",op,seq,dut.cmd_status,status);checks++;
 end endtask
 task bad_stop;reg[15:0]c;reg[7:0]prev_seq;reg[15:0]prev_errors;begin
  prev_seq=dut.last_seq;prev_errors=dut.cmd_errors;seq++;
  c=crcbyte(crcbyte(16'hffff,seq),8'h01);repeat(4)c=crcbyte(c,8'd0);
  byte_in(8'haa);byte_in(8'h55);byte_in(seq);byte_in(8'h01);repeat(4)byte_in(8'd0);
  byte_in(c[7:0]^8'h01);byte_in(c[15:8]);clocks(100);
  if(!dut.armed || dut.last_seq!=prev_seq || dut.cmd_errors!=prev_errors+1)$fatal(1,"badCRC STOP entered fast path");checks++;
 end endtask
 initial begin
  fd=$fopen("top_wire.bin","wb");clocks(3);rst=1;clocks(10);stopped();
  command(8'h16,CFG,1); // old units/config opcode rejected
  command(8'h36,CFG,0);
  if(dut.motion_configuration!=CFG || dut.workspace_limit!=217360 || dut.shuttle_amplitude!=17160 || dut.motion_speed!=1)$fatal(1,"preserved configuration mismatch");stopped();
  command(8'h36,CFG|32'h20000000,3);
  force dut.calibrated=1;
  command(3,32'h4a323830,4);if(dut.armed)$fatal(1,"legacyARM accepted");
  command(3,`J280_C_TOKEN,0);if(!dut.armed)$fatal(1,"newARM rejected");bad_stop();
  force dut.reference_ready=1;
  command(6,100000,1);if(dut.target!=0)$fatal(1,"legacy target accepted");
  command(8'h26,102944,0);if(dut.target!=106748)$fatal(1,"physical90deg target conversion%0d",dut.target);
  command(8'h26,-102944,0);if(dut.target!=-106748)$fatal(1,"negative target conversion");
  command(8'h26,2147483647,2);if(dut.target!=-106748)$fatal(1,"overflow target committed");
  command(8'h26,210000,2);if(dut.target!=-106748)$fatal(1,"boundary target committed");
  command(1,0,0);if(dut.armed)$fatal(1,"STOP did not disarm");stopped();
  release dut.reference_ready;release dut.calibrated;
  // Zero uses the real shared CAL pulse; test injection avoids waiting for the
  // mechanical calibration guard. UART/packetization/measurement RTL are real.
  force dut.enc=100;force dut.calibrate=1;clocks(1);release dut.calibrate;clocks(5);
  force dut.enc=370;clocks(100000);command(8'h30,0,0);clocks(2200000);force dut.send=0;clocks(700000);
  if(dut.encoder_zero!=100 || dut.phi_physical!=103109)$fatal(1,"physical export/zero incorrect phi%0d zero%0d",dut.phi_physical,dut.encoder_zero);
  stopped();
  // Retained RAM must keep its original zero even after a later CAL.
  release dut.send;
  force dut.capture=1;clocks(1);release dut.capture;clocks(250000);
  command(1,0,0);clocks(20);
  if(dut.recording_encoder_zero!=100 || dut.diag_count==0)$fatal(1,"capture zero/count not latched");
  force dut.enc=1000;force dut.calibrate=1;clocks(1);release dut.calibrate;clocks(30);
  command(8'h30,0,0);clocks(700000);
  command(8'h32,0,0);clocks(5000000);
  force dut.send=0;clocks(700000);stopped();
  // Key stop path still forces registered bridge pins inactive.
  force dut.key_stop=1;clocks(3);stopped();release dut.key_stop;
  release dut.enc;
  for(integer i=1;i<=20;i++)begin force dut.enc=1000+i*3;clocks(50000);end
  force dut.send=0;clocks(700000);
  if(stop_edge_checks!=2)$fatal(1,"missing exactSTOP edge checks");
  $fclose(fd);$display("PASS actual top UART checks=%0d bytes=%0d coherent_commits=%0d",checks,bytes,sample_checks);$finish;
 end
 initial begin #500000000;$fatal(1,"top timeout");end
endmodule
