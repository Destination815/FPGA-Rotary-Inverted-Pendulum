`timescale 1ns/1ps
`include "j280_coordinates.vh"
module test_motion_config_commit;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(1'b1),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));
 reg valid=0;reg[7:0]op=0,seq=0;reg signed[31:0]value=0;integer checks=0;
 localparam[31:0] CFG_A=32'd42505918|32'h10000000;
 localparam[31:0] CFG_B=32'd41981630;
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task need(input bit b,input string why);begin checks++;if(!b)$fatal(1,"%s",why);end endtask
 task emit(input[7:0]o,input[31:0]v);begin seq++;op=o;value=v;valid=1;#1;end endtask
 task command(input[7:0]o,input[31:0]v,input[7:0]status);begin
  emit(o,v);clocks(1);valid=0;clocks(3);
  need(dut.last_seq==seq&&dut.cmd_status==status,"wrong ACK/status");
 end endtask
 initial begin
  force dut.wire_cmd_valid=valid;force dut.wire_opcode=op;force dut.wire_cmd_seq=seq;force dut.wire_cmd_value=value;
  clocks(3);rst=1;clocks(10);force dut.tick=0;
  need(!dut.armed&&dut.idle,"initial board not idle");
  emit(8'h36,CFG_A);need(!dut.cmd_valid,"config exposed before snapshot");
  clocks(1);valid=0;#1;need(dut.motion_configuration==CFG_B,"config committed too early");
  value=32'hffffffff;clocks(2);
  need(dut.last_seq==seq&&dut.cmd_status==0&&dut.motion_configuration==CFG_A,"snapshot config not committed");
  need(dut.workspace_limit==217360&&dut.shuttle_amplitude==17160&&dut.motion_speed==1&&dut.shuttle_period==20,"config mapping changed");
  clocks(2);command(8'h36,CFG_A|32'h20000000,3);need(dut.motion_configuration==CFG_A,"invalid config committed");
  // Acceptance at arrival cannot become true just because live guards change.
  force dut.armed=1;emit(8'h36,CFG_B);clocks(1);valid=0;force dut.armed=0;clocks(2);
  need(dut.cmd_status==3&&dut.motion_configuration==CFG_A,"originally active command became accepted later");release dut.armed;
  emit(8'h36,CFG_B);clocks(1);valid=0;force dut.armed=1;clocks(2);
  need(dut.cmd_status==3&&dut.motion_configuration==CFG_A,"live armed guard not rechecked");force dut.armed=0;clocks(1);release dut.armed;
  force dut.dump_active=1;emit(8'h36,CFG_B);clocks(1);valid=0;force dut.dump_active=0;clocks(2);
  need(dut.cmd_status==2&&dut.motion_configuration==CFG_A,"original dump rejection not retained");release dut.dump_active;
  // STOP preempts immediately and no pending configuration may commit later.
  emit(8'h36,CFG_B);clocks(1);valid=0;emit(1,0);
  need(dut.serial_stop,"serialSTOP delayed");clocks(1);valid=0;clocks(3);
  need(dut.last_seq==seq&&!dut.armed&&dut.motion_configuration==CFG_A,"STOP cancelled config appeared later");
  emit(8'h36,CFG_B);clocks(1);valid=0;force dut.key_stop=1;clocks(1);release dut.key_stop;clocks(3);
  need(dut.motion_configuration==CFG_A&&!dut.armed,"SW4 cancelled config appeared later");
  command(8'h36,CFG_B,0);need(dut.motion_configuration==CFG_B,"new config failed afterSTOP");
  need(!bn1&&!bn2&&pb&&!an1&&!an2&&pa&&!dut.start&&!dut.capture,"config changed drive or started motion");
  $display("PASS integrated_config_commit_checks=%0d",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"configuration test timeout");end
endmodule
