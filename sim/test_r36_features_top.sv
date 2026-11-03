`timescale 1ns/1ps
module test_r36_features_top;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;wire tx,ac,oe,an1,an2,pa,bn1,bn2,pb;wire[3:0]led;
 j280_top dut(.clk_50m(clk),.rst_n(rst),.key_sw(4'b1111),.uart_rx(1'b1),.uart_tx(tx),.adc_clk(ac),.adc_oe_n(oe),.adc_data_in(10'd746),.enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),.AN1(an1),.AN2(an2),.PWMA(pa),.BN1(bn1),.BN2(bn2),.PWMB(pb),.led_n(led));
 reg valid=0;reg[7:0]op=0,seq=0;reg signed[31:0]value=0;integer checks=0;
 reg hb_tick=0,hb_armed=0,hb_heartbeat=0;wire hb_expired;integer i,phase;reg signed[31:0] prior_target;reg[31:0] prior_config;
 motion_link_watchdog hb(.clk(clk),.rst_n(rst),.tick(hb_tick),.armed(hb_armed),.heartbeat(hb_heartbeat),.expired(hb_expired));
 localparam[31:0] CFG_EXT=32'h30000068 | (150<<8) | (2<<19) | (20<<21);
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task need(input bit b,input string why);begin checks++;if(!b)$fatal(1,"%s",why);end endtask
 task emit(input[7:0]o,input[31:0]v);begin seq++;op=o;value=v;valid=1;#1;end endtask
 task command(input[7:0]o,input[31:0]v,input[7:0]status);begin
  emit(o,v);clocks(1);valid=0;clocks(35);need(dut.last_seq==seq&&dut.cmd_status==status,"wrong ACK/status");
 end endtask
 initial begin
  force dut.wire_cmd_valid=valid;force dut.wire_opcode=op;force dut.wire_cmd_seq=seq;force dut.wire_cmd_value=value;
  clocks(3);rst=1;clocks(10);force dut.tick=0;
  command(8'h36,CFG_EXT,0);need(dut.workspace_limit==411840,"extended workspace decode");need(dut.motion_speed==2&&dut.motion_speed_limit==0,"preset compatibility reset");
  command(8'h37,1144,0);need(dut.motion_speed_limit==1186,"minimum numeric speed conversion");
  need(dut.coordinates_frame[447:416]==1186&&dut.coordinates_frame[479:448]==32'h3601,"numeric speed descriptor readback");
  command(8'h37,68629,0);need(dut.motion_speed_limit==71165,"maximum numeric speed conversion");
  command(8'h37,0,3);need(dut.motion_speed_limit==71165,"zero speed admitted");
  command(8'h37,-1144,3);need(dut.motion_speed_limit==71165,"negative speed admitted");
  command(8'h37,80000,3);need(dut.motion_speed_limit==71165,"overspeed admitted");
  // Active admission stays rejected even if the board becomes idle during mapping.
  force dut.armed=1;emit(8'h37,22875);clocks(1);valid=0;force dut.armed=0;clocks(35);
  need(dut.cmd_status==3&&dut.motion_speed_limit==71165,"active speed request admitted after state change");release dut.armed;
  // Every newer command/SW4 cancels an in-flight mapped request.
  emit(8'h37,22875);clocks(1);valid=0;emit(8'h01,0);need(dut.serial_stop,"STOP delayed by numeric mapping");clocks(1);valid=0;clocks(35);
  need(!dut.armed&&dut.motion_speed_limit==71165,"cancelled numeric speed reappeared");
  emit(8'h37,22875);clocks(1);valid=0;force dut.key_stop=1;clocks(1);release dut.key_stop;clocks(35);need(dut.motion_speed_limit==71165,"SW4 cancellation failed");
  force dut.armed=1;force dut.reference_ready=1;
  command(8'h26,377471,0);need(dut.target>390000,"physical330 target still clipped by old180 guard");
  command(8'h26,388930,2);need(dut.target<400402,"out of actual range target changed state");
  force dut.armed=0;release dut.reference_ready;clocks(1);release dut.armed;
  need(!bn1&&!bn2&&pb&&!an1&&!an2&&pa,"configuration started motor");
  // Capture configuration admission on the original raw receive edge.
  prior_config=dut.motion_configuration;
  force dut.armed=1;emit(8'h36,CFG_EXT);clocks(1);valid=0;force dut.armed=0;clocks(4);
  need(dut.cmd_status==3&&dut.motion_configuration==prior_config,"config original armed admission lost");release dut.armed;
  force dut.dump_active=1;emit(8'h36,CFG_EXT);clocks(1);valid=0;force dut.dump_active=0;clocks(4);
  need(dut.cmd_status==2&&dut.motion_configuration==prior_config,"config original dump admission lost");release dut.dump_active;
  emit(8'h36,CFG_EXT);clocks(1);valid=0;force dut.armed=1;clocks(4);
  need(dut.cmd_status==3,"config live guard skipped");force dut.armed=0;clocks(1);release dut.armed;
  // All conversion and output-stage phases: STOP must ACK and disarm on
  // its original raw acceptance edge, and no delayed target may reappear.
  for(phase=0;phase<29;phase=phase+1)begin
   force dut.armed=1;force dut.reference_ready=1;prior_target=dut.target;
   emit(8'h26,1144);clocks(1);valid=0;clocks(phase);
   emit(1,0);release dut.armed;clocks(1);valid=0;
   need(!dut.armed&&dut.last_seq==seq&&dut.cmd_status==0,"fastSTOP acceptance edge/ACK changed");
   release dut.reference_ready;clocks(35);
   if(phase<28)need(dut.target==prior_target,"STOP-cancelled target reappeared");
  end
  // A raw newer INFO must cancel an unfinished conversion even while the
  // first and last pipeline stages contain older work.
  for(phase=0;phase<28;phase=phase+1)begin
   force dut.armed=1;force dut.reference_ready=1;prior_target=dut.target;
   emit(8'h26,22875);clocks(1);valid=0;clocks(phase);emit(8'h30,0);clocks(1);valid=0;clocks(35);
   need(dut.last_seq==seq&&dut.cmd_status==0&&dut.target==prior_target,"newINFO failed to cancel target");
   release dut.reference_ready;force dut.armed=0;clocks(1);release dut.armed;
  end
  // Each of the two ordinary stages must reject an old config when a
  // newer STOP, INFO or SW4 arrives before its commit edge.
  for(phase=0;phase<2;phase=phase+1)begin
   prior_config=dut.motion_configuration;
   emit(8'h36,32'd41981630);clocks(1);valid=0;clocks(phase);emit(1,0);clocks(1);valid=0;clocks(5);
   need(dut.last_seq==seq&&dut.motion_configuration==prior_config,"STOP-cancelled config reappeared");
   emit(8'h36,32'd41981630);clocks(1);valid=0;clocks(phase);emit(8'h30,0);clocks(1);valid=0;clocks(5);
   need(dut.last_seq==seq&&dut.motion_configuration==prior_config,"newINFO-cancelled config reappeared");
   emit(8'h36,32'd41981630);clocks(1);valid=0;clocks(phase);force dut.key_stop=1;clocks(1);release dut.key_stop;clocks(5);
   need(dut.motion_configuration==prior_config,"SW4-cancelled config reappeared");
  end
  // Accelerated logical1ms ticks:150 seconds with a valid INFO every200ms.
  hb_armed=1;hb_tick=1;
  for(i=0;i<150000;i=i+1)begin hb_heartbeat=(i%200)==0;clocks(1);need(!hb_expired,"heartbeat imposed total session duration");end
  hb_armed=0;hb_heartbeat=0;clocks(1);hb_armed=1;
  clocks(2999);need(!hb_expired,"link timeout before3000 ticks");clocks(1);need(hb_expired,"link did not timeout at3000 ticks");
  hb_armed=0;clocks(1);need(!hb_expired,"unarmed link timer not reset");clocks(4000);need(!hb_expired,"unarmed board counted link timeout");hb_tick=0;
  // The real top gives expiry priority over a same-clock ARM and cancels conversion.
  force dut.armed=1;force dut.link_watchdog.age_ms=12'd3000;release dut.armed;
  emit(8'h03,32'h4a433131);clocks(1);valid=0;need(!dut.armed,"link timeout lost priority to ARM");release dut.link_watchdog.age_ms;clocks(2);
  prior_target=dut.target;force dut.armed=1;force dut.reference_ready=1;force dut.link_watchdog.age_ms=12'd2999;
  emit(8'h26,1144);clocks(1);valid=0;force dut.link_watchdog.age_ms=12'd3000;release dut.armed;clocks(1);release dut.link_watchdog.age_ms;release dut.reference_ready;clocks(35);
  need(!dut.armed&&dut.target==prior_target,"link timeout left delayed target commit");
  command(8'h30,0,0);need(!dut.armed&&!dut.start&&!dut.capture,"reconnection automatically started motion");
  $display("PASS r36_top_feature_checks=%0d",checks);$finish;
 end
 initial begin #5000000;$fatal(1,"feature top timeout");end
endmodule
