`timescale 1ns/1ps
module test_r26_recovery_uart;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;reg[3:0]keys=15;
 wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));
 wire bv,be;wire[7:0]bd;
 uart_rx #(.DIV(434)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(bv),.error(be),.data(bd));
 integer fd,bytes=0,checks=0;reg[7:0] seq=0;
 localparam[31:0] BASE=32'd42505918,WIDE=BASE|32'h10000000;
 always @(posedge clk)begin
  if(bv)begin $fwrite(fd,"%c",bd);bytes++;end
  if(rst && be)$fatal(1,"actual UART framing error");
 end
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task stopped;begin if(bn1||bn2||!pb||an1||an2||!pa||dut.start||dut.capture||dut.calibrate)$fatal(1,"configuration caused movement/calibration");end endtask
 task byte_in(input [7:0]v);integer j;begin rx=0;clocks(434);for(j=0;j<8;j++)begin rx=v[j];clocks(434);end rx=1;clocks(434);end endtask
 function automatic [15:0]crcbyte(input[15:0]old,input[7:0]v);reg[15:0]c;integer j;begin c=old^{v,8'd0};for(j=0;j<8;j++)c=c[15]?(c<<1)^16'h1021:c<<1;return c;end endfunction
 task command(input[7:0]op,input[31:0]v,input[7:0]status);reg[15:0]c;integer j;begin
  seq++;c=crcbyte(crcbyte(16'hffff,seq),op);byte_in(8'haa);byte_in(8'h55);byte_in(seq);byte_in(op);
  for(j=0;j<4;j++)begin byte_in(v[j*8+:8]);c=crcbyte(c,v[j*8+:8]);end byte_in(c[7:0]);byte_in(c[15:8]);clocks(100);
  if(dut.last_seq!=seq||dut.cmd_status!=status)$fatal(1,"command op=%h status=%0d expected=%0d",op,dut.cmd_status,status);
 end endtask
 task verify(input[31:0]v);begin
  if(dut.motion_configuration!=v||dut.control.full_turn_recovery!=v[28])$fatal(1,"configuration or controller flag mismatch");
  if(dut.workspace_limit!=217360||dut.shuttle_amplitude!=17160||dut.motion_speed!=1||dut.shuttle_period!=20)$fatal(1,"motion fields corrupted by recovery option: workspace=%0d amplitude=%0d speed=%0d period=%0d",dut.workspace_limit,dut.shuttle_amplitude,dut.motion_speed,dut.shuttle_period);
  stopped();checks++;
 end endtask
 task readback;begin command(8'h30,0,0);clocks(300000);end endtask
 initial begin
  fd=$fopen("recovery_uart.bin","wb");clocks(3);rst=1;clocks(10);force dut.send=0;stopped();
  if(dut.motion_configuration[28]||dut.armed)$fatal(1,"option must reset disabled/unarmed");
  command(8'h16,BASE,0);verify(BASE);readback();
  command(8'h16,WIDE,0);verify(WIDE);readback();
  command(8'h16,WIDE|32'h20000000,3);verify(WIDE);
  command(8'h16,WIDE|32'h40000000,3);verify(WIDE);
  command(8'h16,WIDE|32'h80000000,3);verify(WIDE);
  // Test-only supervisor state/armed injection; UART parser/config registers real.
  force dut.armed=1;command(8'h16,BASE,3);verify(WIDE);force dut.armed=0;
  force dut.control.state=3;command(8'h16,BASE,3);verify(WIDE);force dut.control.state=0;
  command(8'h16,WIDE^32'd191,3);verify(WIDE);
  command(8'h16,BASE,0);verify(BASE);readback();
  command(8'h01,0,0);stopped();if(dut.armed)$fatal(1,"STOP armed");
  release dut.armed;release dut.control.state;
  rst=0;clocks(3);rst=1;clocks(10);stopped();
  if(dut.motion_configuration[28]||dut.armed)$fatal(1,"reset retained recovery authorization");checks++;
  $fclose(fd);$display("PASS R26 recovery actual UART groups=%0d bytes=%0d",checks,bytes);$finish;
 end
 initial begin #100000000;$fatal(1,"UART config timeout");end
endmodule
