`timescale 1ns/1ps
module test_r40_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;wire tx,ac,oe,a1,a2,ap,b1,b2,bp;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(rx),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd742),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(a1),.AN2(a2),.PWMA(ap),.BN1(b1),.BN2(b2),.PWMB(bp),.led_n(led));
 integer checks=0,n;reg valid=0;reg[7:0]op=0,seq=0;reg signed[31:0]value=0;
 reg[31:0]cfg;reg signed[31:0]speed,origin,center=102944;
 integer capture_pulses=0,start_pulses=0;
 always @(posedge clk)begin
  if(dut.capture)capture_pulses++;
  if(dut.start)start_pulses++;
 end
 task clocks(input integer count);repeat(count)begin @(negedge clk);#1;end endtask
 task need(input bit b,input string why);checks++;if(!b)$fatal(1,"%s",why);endtask
 task emit(input[7:0]o,input[31:0]v);seq++;op=o;value=v;valid=1;#1;endtask
 task cmd(input[7:0]o,input[31:0]v,input[7:0]status);begin
  emit(o,v);clocks(1);valid=0;clocks(35);
  need(dut.last_seq==seq&&dut.cmd_status==status,$sformatf("op%h ACK/status got%0d want%0d",o,dut.cmd_status,status));
 end endtask
 task same_config;
  need(dut.motion_configuration==cfg,"ext2 changed ordinary CONFIG");
  need(dut.motion_speed_limit==speed,"ext2 changed ordinary numeric speed");
  need(dut.capture_origin==origin,"ext2 changed capture origin");
  need(capture_pulses==0&&start_pulses==0,"ext2 inserted automatic capture/start");
 endtask
 initial begin
  force dut.wire_cmd_valid=valid;force dut.wire_opcode=op;force dut.wire_cmd_seq=seq;force dut.wire_cmd_value=value;
  clocks(3);rst=1;clocks(10);force dut.tick=0;force dut.state=0;force dut.stop_request=0;
  cmd('h38,1,2);need(!dut.ext2_prepared,"IDLE gained ext2 profile");
  cmd('h36,848336488,0);cmd('h37,34315,0);
  cfg=dut.motion_configuration;speed=dut.motion_speed_limit;
  need(cfg==848336488&&speed!=0,"normal config/speed not set");
  force dut.armed=1;force dut.calibrated=1;force dut.measurement_ok=1;
  force dut.state=3;force dut.reference_ready=1;force dut.reference=center;
  force dut.capture_origin=32'sd4000;origin=dut.capture_origin;
  force dut.theta=32'sd3431;force dut.dtheta=32'sd65536;force dut.dphi=32'sd40000;
  cmd('h38,2,2);need(!dut.ext2_prepared,"invalid value accepted");
  cmd('h38,1,0);need(dut.ext2_prepared&&dut.shuttle,"shaky nonzero-center entry rejected");same_config();
  cmd('h38,1,2);same_config();
  force dut.trajectory_running=1;force dut.reference_ready=0;force dut.tick=1;clocks(2);force dut.tick=0;
  need(dut.ext2_consumed,"actual reference start not reflected");
  cmd('h26,0,2);same_config();
  cmd('h38,0,0);need(dut.ext2_prepared&&!dut.shuttle,"exit removed profile before fade done");same_config();
  force dut.trajectory_running=0;force dut.reference_ready=1;clocks(3);
  need(!dut.ext2_prepared,"completed fade did not restore ordinary mode");same_config();
  cmd('h26,0,0);same_config();
  cmd('h38,1,0);same_config();
  force dut.state=1;clocks(3);need(!dut.ext2_prepared,"automatic recovery retained ext2 profile");same_config();
  force dut.state=3;
  center=360*1144-60058;clocks(2);cmd('h38,1,0);
  force dut.trajectory_running=1;force dut.tick=1;clocks(2);force dut.tick=0;cmd('h38,0,0);
  force dut.trajectory_running=0;clocks(3);
  center=360*1144-60057;clocks(2);cmd('h38,1,2);need(!dut.ext2_prepared,"out-of-travel center accepted");
  center=-360*1144+60058;clocks(2);cmd('h38,1,0);same_config();
  // STOP has priority over a switch still in the command path.
  release dut.armed;cmd(1,0,0);need(!dut.armed&&!dut.ext2_prepared,"STOP not clearing profile");
  for(n=0;n<3;n++)begin
   force dut.armed=1;center=0;emit('h38,1);clocks(1);valid=0;clocks(n);release dut.armed;emit(1,0);clocks(1);valid=0;
   clocks(40);need(!dut.armed&&!dut.ext2_prepared,"cancelled entry appeared after STOP");
  end
  cmd(8,0,0);need(!dut.ext2_prepared,"CLEAR restored profile");
  need(capture_pulses==0&&start_pulses==0,"mode switch recaptured");
  $display("PASS r40_top checks=%0d",checks);$finish;
 end
 initial begin #50000000;$fatal(1,"timeout");end
endmodule
