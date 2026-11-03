`timescale 1ns/1ps
module test_passive_restart;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;wire tx,ac,oe,a1,a2,ap,b1,b2,bp;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd742),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(a1),.AN2(a2),.PWMA(ap),.BN1(b1),.BN2(b2),.PWMB(bp),.led_n(led));
 reg[2:0]st=0;reg[7:0]fault_in=0;reg sp=0,cp=0,sample_in=0,stop_in=0,arm=1;reg[31:0]ms=0;reg signed[31:0]zero=100;reg[12:0]ri=0;
 integer checks=0,a,b,n;integer id0,base_ms;
 task clocks(input integer count);repeat(count)begin @(negedge clk);#1;end endtask
 task need(input bit ok,input string msg);checks++;if(!ok)$fatal(1,"%s id=%0d state=%0d prev=%0d",msg,dut.passive_id,st,dut.previous_state);endtask
 task reset_case;begin rst=0;st=0;fault_in=0;sp=0;cp=0;sample_in=0;stop_in=0;arm=1;ms=100;zero=100;clocks(4);rst=1;clocks(10);end endtask
 task start_auto;begin sp=1;clocks(1);sp=0;st=1;clocks(2);need(dut.passive_busy&&dut.passive_automatic&&dut.passive_id==1,"auto start metadata");end endtask
 task row;begin ms++;sample_in=1;clocks(1);sample_in=0;clocks(2);end endtask
 task readms(input integer index,input integer value);begin ri=index;clocks(3);need(dut.passive_read_valid&&dut.passive_data[31:0]==value,$sformatf("RAM[%0d] time got%0d want%0d",index,dut.passive_data[31:0],value));end endtask
 initial begin
  force dut.tick=0;force dut.state=st;force dut.fault=fault_in;force dut.drive_tripped=0;force dut.start=sp;force dut.capture=cp;force dut.result_valid=sample_in;force dut.stop_request=stop_in;force dut.serial_stop=0;force dut.armed=arm;force dut.millis=ms;force dut.encoder_zero=zero;force dut.read_index=ri;
  force dut.output_command=32'sd10;force dut.applied_debug=32'sd9;force dut.active_limit=32'd150;force dut.theta=32'sd100;force dut.dtheta=32'sd200;force dut.sampled_enc=32'sd123;force dut.control_adc_q4=14'd11872;force dut.control_adc_fine_valid=1;
  // All nine valid state edges restart exactly once; intra-recovery states do not.
  for(a=2;a<=4;a++)for(b=0;b<3;b++)begin
   reset_case();start_auto();st=a;clocks(2);row();zero=222;st=b==0?1:b==1?6:7;clocks(1);
   need(dut.passive_id==2&&dut.passive_busy&&dut.passive_automatic&&dut.passive_total==0,"recovery edge did not restart");need(dut.recording_encoder_zero==222,"encoder origin not captured with new id");
   clocks(10);need(dut.passive_id==2,"edge repeated");st=1;clocks(2);st=6;clocks(2);st=7;clocks(2);need(dut.passive_id==2,"intra recovery retriggered");
  end
  reset_case();start_auto();st=2;clocks(2);st=3;clocks(2);st=4;clocks(2);st=3;clocks(2);need(dut.passive_id==1,"normal capture/hold/track restarted");
  // A manual capture is never reclassified as automatic.
  reset_case();cp=1;clocks(1);cp=0;st=2;clocks(2);row();st=1;clocks(2);need(dut.passive_id==1&&!dut.passive_automatic&&dut.passive_total==1,"manual capture restarted");
  // STOP wins on recovery transition and preserves collected data/id/origin.
  reset_case();start_auto();st=3;clocks(2);row();id0=dut.passive_id;zero=333;st=1;stop_in=1;clocks(1);stop_in=0;clocks(4);
  need(dut.passive_id==id0&&dut.passive_valid&&dut.passive_count==1&&dut.passive_reason==1&&!dut.passive_busy,"STOP lost to restart");need(dut.recording_encoder_zero==100,"STOP updated origin");
  // A coincident fault sample freezes original capture, including fault row.
  reset_case();start_auto();st=4;clocks(2);row();zero=444;st=6;fault_in=10;row();clocks(3);
  need(dut.passive_id==1&&dut.passive_valid&&dut.passive_count==2&&dut.passive_reason==3&&dut.passive_fault==10,"fault lost to restart");need(dut.recording_encoder_zero==100,"fault updated origin");
  // Stopped/no active capture and disarmed sessions cannot restart.
  st=3;fault_in=0;clocks(2);st=1;clocks(2);need(dut.passive_id==1,"frozen capture restarted");
  reset_case();start_auto();st=3;clocks(2);arm=0;st=1;clocks(2);need(dut.passive_id==1,"disarmed transition restarted");
  // The same 8192-row RAM now contains latest recovery head + latest tail.
  reset_case();start_auto();st=3;clocks(2);repeat(100)row();st=6;clocks(1);base_ms=ms;need(dut.passive_id==2,"last recovery not selected");
  for(n=0;n<10000;n++)begin if(n==20)st=2;if(n==2020)st=3;row();end
  stop_in=1;clocks(1);stop_in=0;clocks(4);
  need(dut.passive_id==2&&dut.passive_automatic&&dut.passive_total==10000&&dut.passive_head==4096&&dut.passive_tail==4096&&dut.passive_count==8192,"latest recovery head/tail counts");
  need(dut.passive_start==base_ms&&dut.passive_gaps==0&&dut.passive_trigger==20&&dut.passive_trigger_ms==base_ms+21,"latest recovery metadata");
  readms(0,base_ms+1);readms(4095,base_ms+4096);readms(4096,base_ms+5905);readms(8191,base_ms+10000);
  need(dut.coordinates_frame[351:320]==100&&dut.coordinates_frame[383:352]==(32'h80000000|2),"wire capture identity/origin mismatch");
  $display("PASS passive restart checks=%0d",checks);$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
