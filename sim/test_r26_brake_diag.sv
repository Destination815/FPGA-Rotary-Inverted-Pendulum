`timescale 1ns/1ps
module test_r26_brake_diag;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;reg[3:0]keys=15;
 wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(keys),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));
 wire bv,be;wire[7:0]bd;
 uart_rx #(.DIV(434)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(bv),.error(be),.data(bd));
 integer fd,bytes=0,checks=0,cycles=0,stop_cycle=-1,stop_checks=0;
 always @(posedge clk)begin
  cycles++;
  if(bv)begin $fwrite(fd,"%c",bd);bytes++;end
  if(rst && be)$fatal(1,"actual UART framing error");
  if(rst && dut.cmd_valid && dut.opcode==1)stop_cycle=cycles;
  if(stop_cycle>=0 && cycles==stop_cycle+4)begin
   if(dut.armed||dut.requested_enable||bn1||bn2||!pb||dut.applied_debug)$fatal(1,"UART STOP did not clear physical output by4clks");
   stop_checks++;
  end
 end
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task stopped;begin if(bn1||bn2||!pb||an1||an2||!pa)$fatal(1,"physical bridge STOP mismatch");end endtask
 task arm_drive;begin force dut.armed=1;clocks(1);release dut.armed;clocks(3000);if(!dut.motor_enable||dut.raw_applied==0)$fatal(1,"injected BRAKE motor setup failed");end endtask
 task pulse(input integer ms);begin force dut.millis=ms;force dut.send=1;clocks(1);force dut.send=0;end endtask
 task frame(input integer ms);begin pulse(ms);clocks(999999);end endtask
 task byte_in(input [7:0]v);integer j;begin rx=0;clocks(434);for(j=0;j<8;j++)begin rx=v[j];clocks(434);end rx=1;clocks(434);end endtask
 function automatic [15:0]crcbyte(input[15:0]old,input[7:0]v);reg[15:0]c;integer j;begin c=old^{v,8'd0};for(j=0;j<8;j++)c=c[15]?(c<<1)^16'h1021:c<<1;return c;end endfunction
 task serial_stop;reg[15:0]c;integer j;begin
  c=crcbyte(crcbyte(16'hffff,8'd77),8'd1);byte_in(8'haa);byte_in(8'h55);byte_in(77);byte_in(1);
  for(j=0;j<4;j++)begin byte_in(0);c=crcbyte(c,0);end byte_in(c[7:0]);byte_in(c[15:8]);clocks(100);
 end endtask
 initial begin
  fd=$fopen("brake_diag_uart.bin","wb");clocks(3);rst=1;clocks(10);stopped();
  // Test-only coherent supervisor/observer injection. Production packetizer,
  // UART TX and RX at115200, command parser, STOP logic, guard and pins remain real.
  force dut.send=0;force dut.control.state=6;force dut.control.command=120;force dut.control.fault=0;force dut.control.watchdog=0;force dut.control.trip_code=0;
  force dut.observer_state={32'sd77,-32'sd66,32'sd55,-32'sd44,32'sd33,-32'sd22,32'sd11};
  force dut.observer_age=321;force dut.observer_integral=-765432;force dut.observer_lp=654321;force dut.capture_origin=-123456;force dut.diag_id=32'h80000003;
  arm_drive();frame(1000);frame(1020);frame(1040);checks++;
  pulse(1060);clocks(350000);
  if(!dut.logger_busy)$fatal(1,"STOP was not sent during active diagnostic transmission");
  serial_stop();clocks(100);stopped();if(dut.armed||dut.last_seq!=77||dut.cmd_status!=0||stop_checks!=1)$fatal(1,"serial STOP ACK/armed/priority failed");checks++;
  clocks(650000);force dut.control.state=0;force dut.control.command=0;frame(1080);frame(1100);stopped();checks++;
  force dut.control.state=6;force dut.control.command=120;arm_drive();pulse(1120);clocks(350000);
  if(!dut.logger_busy)$fatal(1,"SW4 was not asserted during diagnostic transmission");
  keys[3]=0;clocks(4);stopped();if(dut.armed||dut.requested_enable||dut.applied_debug)$fatal(1,"SW4 failed during BRAKE serial transmission");checks++;
  keys[3]=1;clocks(650000);force dut.control.state=0;force dut.control.command=0;frame(1140);stopped();
  if(dut.armed||dut.dropped||dut.cmd_errors)$fatal(1,"post-STOP rearm/drop/error");checks++;
  $fclose(fd);$display("PASS R26 BRAKE diagnostic top UART groups=%0d bytes=%0d stop_checks=%0d build=%08h",checks,bytes,stop_checks,dut.D02_BUILD);$finish;
 end
 initial begin #200000000;$fatal(1,"BRAKE diagnostic UART timeout");end
endmodule
