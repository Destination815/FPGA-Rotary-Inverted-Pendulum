`timescale 1ns/1ps
module test_r41_motion_safety;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,step=0,stop=0,armed=1,guard=0,full=1;
 reg[2:0] state_in=3;
 reg signed[31:0] phi=0,velocity=0,ref_in=0,workspace=300000;
 wire[7:0]fault;wire signed[31:0]command;
 controller dut(.clk(clk),.rst_n(rst_n),.step(step),.frame_tick(step),.measurement_ok(1'b1),.armed(armed),.sensor_gap_active(1'b0),.sensor_gap_ms(8'd0),
 .request_start(1'b0),.request_capture(1'b0),.request_clear(1'b0),.request_stop(stop),.request_jog(1'b0),.request_target(1'b0),.request_shuttle(1'b0),
 .jog_value(32'd0),.theta(32'd0),.dtheta(32'd0),.phi(phi),.dphi(velocity),.target(32'd0),.filtered_theta_q18(32'd0),.shuttle(1'b0),.full_turn_recovery(full),.ext2_enable(1'b0),
 .workspace_limit(workspace),.shuttle_amplitude(32'd0),.motion_speed(2'd0),.motion_speed_limit(32'd0),.shuttle_period(8'd20),
 .ktheta(32'd0),.kdtheta(32'd0),.kphi(32'd0),.kdphi(32'd0),.pwm_limit(32'd150),.swing_gain(32'd0),.fault(fault),.command(command));
 integer checks=0,i,j;
 task clocks(input integer n);repeat(n)begin @(negedge clk);#1;end endtask
 task need(input bit b,input string why);checks++;if(!b)$fatal(1,"%s fault=%0d cmd=%0d",why,fault,command);endtask
 task tick;step=1;clocks(1);step=0;clocks(7);endtask
 task reset;
 rst_n=0;step=0;stop=0;armed=1;guard=0;state_in=3;phi=0;velocity=0;ref_in=0;workspace=300000;full=1;clocks(3);rst_n=1;clocks(4);
 endtask
 initial begin
  force dut.state=state_in;force dut.phase=0;force dut.tracking_initialized=1;
  force dut.guard_active=guard;force dut.reference=ref_in;force dut.swing_manual=0;
  force dut.pending_command_valid=1;force dut.pending_command=70;force dut.pending_command_limit=150;
  force dut.command_bad=0;
  reset();guard=1;for(i=0;i<999;i++)tick();need(fault==0,"rescue timeout premature");tick();need(fault==10&&command==0,"1000 sampled ms rescue must stop");
  reset();for(j=0;j<6;j++)begin guard=1;for(i=0;i<202;i++)tick();guard=0;tick();end need(fault==0,"normal separated recovery must reset dwell");
  for(j=0;j<2;j++)begin
   reset();phi=j? -90000:90000;velocity=j? -180000:180000;
   for(i=0;i<200;i++)tick();need(fault==0,"outward pipeline must not trip early");tick();need(fault==10&&command==0,"persistent outward escape must stop");
  end
  reset();phi=90000;velocity=-180000;for(i=0;i<300;i++)tick();need(fault==0,"fast inward return is not escape");
  reset();phi=60000;velocity=180000;for(i=0;i<300;i++)tick();need(fault==0,"speed alone is not escape");
  reset();phi=90000;velocity=180000;for(i=0;i<100;i++)tick();velocity=0;for(i=0;i<300;i++)tick();need(fault==0,"bounded disturbance must clear escape dwell");
  reset();guard=1;for(i=0;i<999;i++)tick();stop=1;clocks(2);need(command==0&&dut.r41_rescue_ticks==0,"STOP cancels pending safety and output");
  reset();workspace=60000;phi=90000;tick();need(fault==1&&command==0,"full turn must not bypass normal workspace");
  reset();workspace=60000;phi=90000;state_in=1;tick();need(fault==0,"full turn swing recovery retained");
  reset();workspace=60000;phi=90000;state_in=1;full=0;tick();need(fault==1,"ordinary swing workspace retained");
  reset();guard=1;state_in=1;for(i=0;i<1100;i++)tick();need(fault==0&&dut.r41_rescue_ticks==0,"swing is outside captured rescue timeout");
  $display("PASS R41 safety checks=%0d; actual controller RTL with injected state/measurements, no plant",checks);$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
