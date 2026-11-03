`timescale 1ns/1ps
`include "j280_coordinates.vh"
module test_r39_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;wire tx,ac,oe,a1,a2,ap,b1,b2,bp;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(a1),.AN2(a2),.PWMA(ap),.BN1(b1),.BN2(b2),.PWMB(bp),.led_n(led));
 wire bv,be;wire[7:0]bd;
 uart_rx #(.DIV(434)) mon(.clk(clk),.rst_n(rst),.rx(tx),.valid(bv),.error(be),.data(bd));
 integer fd,checks=0,bytes=0,i,phase;reg valid=0;reg[7:0]op=0,seq=0;reg signed[31:0]value=0;
 reg[511:0]dbg=0;reg changing_debug=0;
 always @* if(changing_debug)begin dbg[63:32]=dut.millis;dbg[95:64]=dut.millis*2;dbg[127:96]=dut.millis*3;dbg[287:256]=dut.millis*100;end
 always @(posedge clk)if(bv)begin $fwrite(fd,"%c",bd);bytes++;end
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task need(input bit b,input string why);checks++;if(!b)$fatal(1,"%s",why);endtask
 task emit(input[7:0]o,input[31:0]v);seq++;op=o;value=v;valid=1;#1;endtask
 task cmd(input[7:0]o,input[31:0]v,input[7:0]status);begin emit(o,v);clocks(1);valid=0;clocks(35);need(dut.last_seq==seq&&dut.cmd_status==status,$sformatf("op%h ACK/status got%0d want%0d",o,dut.cmd_status,status));end endtask
 task stop;begin cmd(1,0,0);need(!dut.armed&&!dut.ext2_prepared,"STOP must clear armed/prepared");end endtask
 initial begin
  fd=$fopen("r39_top_wire.bin","wb");
  force dut.wire_cmd_valid=valid;force dut.wire_opcode=op;force dut.wire_cmd_seq=seq;force dut.wire_cmd_value=value;
  clocks(3);rst=1;clocks(10);force dut.tick=0;
  cmd('h38,1,3);need(!dut.ext2_prepared,"normal config gained permission");
  cmd('h36,75082175,3);cmd('h36,75082174,0);
  need(dut.shuttle_amplitude==48620&&dut.shuttle_period==35&&dut.motion_speed==3,"special config units/default profile");
  cmd('h38,2,3);cmd('h38,1,0);need(dut.ext2_prepared&&!dut.armed,"prepare armed or failed");
  need(dut.info_frame[409]&&dut.info_frame[311],"new capability/readback missing");
  // Permission is sampled at receive as well as commit; becoming idle cannot launder it.
  cmd('h38,0,0);force dut.armed=1;emit('h38,1);clocks(1);valid=0;force dut.armed=0;clocks(35);
  need(dut.cmd_status==3&&!dut.ext2_prepared,"active raw admission laundered");release dut.armed;
  for(phase=0;phase<2;phase++)begin
   emit('h38,1);clocks(1);valid=0;clocks(phase);emit(1,0);clocks(1);valid=0;clocks(6);
   need(!dut.ext2_prepared&&!dut.armed,"STOP cancelled permission reappeared");
   emit('h38,1);clocks(1);valid=0;clocks(phase);emit('h30,0);clocks(1);valid=0;clocks(6);
   need(!dut.ext2_prepared,"INFO cancelled permission reappeared");
  end
  force dut.state=3;force dut.reference_ready=1;force dut.reference_center_q=1;force dut.armed=1;
  cmd(7,1,2);need(!dut.shuttle,"unprepared special shuttle admitted");
  force dut.armed=0;force dut.state=0;cmd('h38,1,0);
  force dut.armed=1;cmd(4,0,2);cmd('h26,0,2);cmd('h38,0,3);
  force dut.state=3;cmd(7,1,0);
  // Match A39H1: one trajectory per capture; host owns11s elapsed STOP.
  force dut.trajectory_running=0;force dut.tick=1;clocks(5);need(!dut.ext2_consumed,"command consumed before actual reference");
  force dut.trajectory_running=1;clocks(1);need(dut.ext2_consumed,"actual trajectory not marked");
  cmd(7,1,2);
  // Feed the unchanged242 link watchdog while accelerating logical1ms ticks.
  for(i=0;i<22;i++)begin clocks(500);cmd('h30,0,0);end
  need(!dut.stop_request&&dut.ext2_prepared,"unapproved board11s stop remains");
  force dut.capture=1;clocks(1);need(!dut.ext2_consumed,"new accepted capture did not reset permission");
  release dut.capture;force dut.trajectory_running=0;clocks(1);
  release dut.armed;stop();
  release dut.trajectory_running;release dut.state;release dut.reference_ready;release dut.reference_center_q;
  force dut.tick=0;clocks(3);cmd('h38,1,0);cmd('h37,22875,3);need(dut.ext2_prepared&&dut.motion_speed_limit==0,"special numeric speed bypass");
  cmd('h36,75082174,0);cmd('h38,1,0);cmd('h36,41981630,0);need(!dut.ext2_prepared,"new config retained permission");
  cmd('h36,75082174,0);cmd('h38,1,0);cmd(8,0,0);need(!dut.ext2_prepared,"CLEAR retained permission");
  cmd('h38,1,0);force dut.key_stop=1;clocks(3);need(!dut.ext2_prepared,"SW4 retained permission");release dut.key_stop;
  // Real 115200baud transmitter/receiver, stable simulated observer words. No plant.
  cmd('h38,1,0);force dut.armed=1;force dut.state=3;force dut.fault=0;force dut.command=0;
  force dut.observer_age=201;force dut.observer_samples=200;force dut.observer_ready=1;force dut.observer_state=224'd0;force dut.observer_integral=0;force dut.observer_lp=0;force dut.capture_origin=0;
  dbg[159:128]=32'd45;dbg[287:256]=32'd123456;dbg[319:288]=100;dbg[351:320]=8192;dbg[383:352]=10000;dbg[415:384]=10000;dbg[447:416]=167772;
  changing_debug=1;force dut.observer_ext2=dbg;release dut.tick;clocks(3000000);force dut.send=0;clocks(1500000);
  need(!be,"UART framing error");release dut.armed;release dut.state;release dut.fault;release dut.command;clocks(3);stop();
  $fclose(fd);$display("PASS r39_top checks=%0d uart_bytes=%0d",checks,bytes);$finish;
 end
 initial begin #500000000;$fatal(1,"timeout");end
endmodule
