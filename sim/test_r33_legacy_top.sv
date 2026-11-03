`timescale 1ns/1ps
`include "j280_coordinates.vh"
module test_r33_legacy_top;
 reg clk=0;always #10 clk=~clk;reg rst=0,rx=1;wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top #(.UART_DIV(8)) dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));

 wire bv,be;wire[7:0]bd;uart_rx #(.DIV(8)) monitor(.clk(clk),.rst_n(rst),.rx(tx),.valid(bv),.error(be),.data(bd));
 integer fd,bytes=0;integer i;reg[7:0]seq=0;reg[31:0]m;reg[13:0]aq;reg signed[31:0]en,cu,ap,th,om;reg[15:0]fl;reg[2:0]st;reg[31:0]li;reg[7:0]fa;reg fine;
 always @(posedge clk)begin if(bv)begin $fwrite(fd,"%c",bd);bytes++;end if(rst&&be)$fatal(1,"UART framing error");end
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task stopped;begin if(bn1||bn2||!pb||an1||an2||!pa||dut.start||dut.capture)$fatal(1,"unexpected drive/start/capture");end endtask
 task byte_in(input[7:0]v);integer j;begin rx=0;clocks(8);for(j=0;j<8;j++)begin rx=v[j];clocks(8);end rx=1;clocks(8);end endtask
 function automatic[15:0]crcbyte(input[15:0]old,input[7:0]v);reg[15:0]c;integer j;begin c=old^{v,8'd0};for(j=0;j<8;j++)c=c[15]?(c<<1)^16'h1021:c<<1;return c;end endfunction
 task command(input[7:0]op);reg[15:0]c;integer j;begin seq++;c=crcbyte(crcbyte(16'hffff,seq),op);byte_in(8'haa);byte_in(8'h55);byte_in(seq);byte_in(op);for(j=0;j<4;j++)begin byte_in(0);c=crcbyte(c,0);end byte_in(c[7:0]);byte_in(c[15:8]);clocks(100);if(dut.last_seq!=seq||dut.cmd_status!=0)$fatal(1,"command failure op%h status%0d",op,dut.cmd_status);end endtask
 task drain;integer k;begin
  k=0;while(dut.dump_active||dut.logger_busy||dut.logger_send||dut.info_pending||dut.compact_info_pending||dut.coordinates_pending||dut.live_pending)begin clocks(50);k++;if(k>700000)$fatal(1,"dump timeout");end
  clocks(1000);stopped();
 end endtask
 task begin_record(input integer n,input integer base,input bit automatic_mode);begin
  m=base;force dut.millis=m;force dut.sampled_enc=en;force dut.theta=th;force dut.dtheta=om;force dut.control_adc_q4=aq;force dut.control_adc_fine_valid=fine;force dut.output_flags=fl;force dut.output_state=st;force dut.active_limit=li;force dut.reported_fault=fa;force dut.output_command=cu;force dut.applied_debug=ap;
  force dut.start=automatic_mode;force dut.capture=!automatic_mode;clocks(1);force dut.start=0;force dut.capture=0;
  for(i=0;i<n;i++)begin
   m=base+i;aq=(i*13+17)%16384;en=i*7-17000;cu=(i%301)-150;ap=-cu;th=(i%3000)-1500;om=(i%10000)-5000;fl=i;st=i<2350?1:2;li=150;fa=0;fine=i%5!=0;
   force dut.result_valid=1;clocks(1);force dut.result_valid=0;clocks(2);stopped();
  end
  if(!dut.passive_busy||dut.passive_valid)$fatal(1,"window fill incorrectly froze recorder");
  cu=0;ap=0;fl=0;st=0;m=base+n;command(1);clocks(10);
  if(dut.passive_busy||!dut.passive_valid||dut.diag_count!=(n<8192?n:8192))$fatal(1,"freeze/count failure count%0d",dut.diag_count);
  command(8'h30);drain();
  // Three actual live frames for host stopped-state gating.
  repeat(3)begin m=m+20;force dut.send=1;clocks(1);force dut.send=0;drain();end
  command(8'h32);drain();
 end endtask
 reg[319:0]ld;reg[9:0]la;
 initial begin
  fd=$fopen("legacy_top_wire.bin","wb");clocks(3);rst=1;clocks(20);
  force dut.tick=0;force dut.send=0;force dut.result_valid=0;force dut.armed=0;force dut.idle=1;force dut.state=0;
  force dut.passive_selected=0;force dut.legacy_ram_address=la;force dut.legacy_ram_data=ld;force dut.legacy_ram_write=0;
  force dut.legacy_count=5;force dut.legacy_phase=4;force dut.legacy_id=17;
  for(i=0;i<5;i++)begin
   la=i;ld=0;ld[31:0]=500+i;ld[47:32]=i;ld[79:64]=745;ld[95:80]=745;ld[127:96]=-27+i;ld[159:128]=-30+i;ld[191:160]=-40+i;ld[223:192]=-50+i;ld[255:224]=-60+i;ld[271:256]=-70+i;ld[287:272]=70-i;
   force dut.legacy_ram_write=1;clocks(1);force dut.legacy_ram_write=0;clocks(4);stopped();
  end
  command(8'h30);drain();command(8'h32);drain();stopped();
  if(dut.legacy_overrun)$fatal(1,"legacy RAM overrun");
  $fclose(fd);$display("PASS actual top D02 two-word sharedRAM UART5rows; stopped output");$finish;
 end
 initial begin #2000000000;$fatal(1,"timeout");end
endmodule
