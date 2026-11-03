`timescale 1ns/1ps
module tb_supervisor;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,step=0,measurement_ok=1,armed=0;
 reg request_start=0,request_capture=0,request_clear=0,request_stop=0,request_jog=0,request_target=0,request_shuttle=0,shuttle=0;
 reg signed[31:0] theta=0,dtheta=0,phi=0,dphi=0,target=0,filtered_theta_q18=0;
 wire[2:0] state;wire[7:0] fault;wire signed[31:0] command,reference,ref_velocity,active_limit;
 wire result_valid,priority_active,reference_ready,trajectory_running;
 controller dut(.workspace_limit(32'sd68629),.shuttle_amplitude(32'sd17157),.motion_speed(2'd0),.shuttle_period(8'd20),.clk(clk),.rst_n(rst_n),.step(step),.frame_tick(step),.measurement_ok(measurement_ok),.armed(armed),
 .request_start(request_start),.request_capture(request_capture),.request_clear(request_clear),.request_stop(request_stop),.request_jog(request_jog),
 .request_target(request_target),.request_shuttle(request_shuttle),.jog_value(32'sd0),.theta(theta),.dtheta(dtheta),.phi(phi),.dphi(dphi),.target(target),.filtered_theta_q18(filtered_theta_q18),.shuttle(shuttle),
 .ktheta(32'sd0),.kdtheta(32'sd0),.kphi(32'sd0),.kdphi(32'sd0),.pwm_limit(32'sd150),.swing_gain(32'sd0),
 .state(state),.fault(fault),.command(command),.reference(reference),.ref_velocity(ref_velocity),.result_valid(result_valid),.priority_active(priority_active),.reference_ready(reference_ready),.trajectory_running(trajectory_running),.active_limit(active_limit),.recovery_boosting(),.recovery_used(),.settled(),.settle_timeout(),.observer_state(),.observer_samples(),.observer_ready_out());
 integer clocks_total=0,checks=0,last_latency=0,max_latency=0,i,j,lat_swing,lat_ref,lat_track,abort_clock;
 always @(posedge clk)clocks_total<=clocks_total+1;
 task clocks(input integer n);repeat(n)@(negedge clk);endtask
 task reset;
 begin
  rst_n=0;armed=0;step=0;request_start=0;request_capture=0;request_clear=0;request_stop=0;request_jog=0;request_target=0;request_shuttle=0;shuttle=0;
  theta=0;dtheta=0;phi=0;dphi=0;target=0;filtered_theta_q18=0;measurement_ok=1;
  clocks(3);rst_n=1;clocks(3);
  if(state!=0||fault||command)$fatal(1,"reset invariant");armed=1;
 end endtask
 task begin_manual;
 begin request_capture=1;clocks(1);request_capture=0;clocks(2);if(state!=2||active_limit!=150)$fatal(1,"manual start %d",state);end endtask
 task begin_auto;
 begin theta=196608;request_start=1;clocks(1);request_start=0;clocks(2);if(state!=1||active_limit!=350)$fatal(1,"automatic start");end endtask
 task run_tick;
 integer n;
 begin step=1;clocks(1);step=0;n=0;while(!result_valid&&n<49900)begin clocks(1);n=n+1;end
 if(n>=49900)$fatal(1,"completion timeout state=%d phase=%d",state,dut.phase);
 last_latency=n;if(n>max_latency)max_latency=n;
 if(command>active_limit||command< -active_limit)$fatal(1,"supervisor output exceeds mode limit");clocks(1);
 end endtask
 task tracking_ready;
 begin reset();begin_manual();for(integer k=0;k<2000;k=k+1)run_tick();if(state!=3||!reference_ready||active_limit!=150)$fatal(1,"2 second handoff failed %d %d",state,dut.hold_ticks);end endtask
 task fast_track_seed;
 begin reset();begin_manual();force dut.hold_ticks=1999;clocks(1);release dut.hold_ticks;run_tick();if(state!=3||!reference_ready)$fatal(1,"seed failed");end endtask
 task expect_fault(input integer code);
 begin run_tick();if(state!=5||fault!=code||command!=0)$fatal(1,"fault expected %d got %d state %d",code,fault,state);
 request_clear=1;clocks(1);request_clear=0;if(state!=5)$fatal(1,"clear while armed bypassed latch");
 armed=0;request_clear=1;clocks(1);request_clear=0;clocks(1);if(state!=0||fault!=0||command!=0)$fatal(1,"disarmed clear failed");checks=checks+1;
 end endtask
 initial begin
 reset();theta=12000;request_capture=1;clocks(1);request_capture=0;clocks(2);if(state!=0)$fatal(1,"capture gate");checks++;
 reset();theta=196608;request_start=1;clocks(1);request_start=0;clocks(2);if(state!=1)$fatal(1,"automatic start not accepted");checks++;
 tracking_ready();checks++;
 target=5719;request_target=1;request_shuttle=0;shuttle=0;clocks(1);request_target=0;request_shuttle=0;run_tick();
 if(trajectory_running || state!=4 || fault)$fatal(1,"bounded target did not enter TRACK");checks++;
 reset();begin_manual();measurement_ok=0;expect_fault(2);
 reset();begin_manual();phi=68630;expect_fault(1);
 reset();begin_manual();phi=-68630;expect_fault(1);
 reset();begin_manual();dphi=1310721;expect_fault(6);
 reset();begin_manual();theta=17158;expect_fault(4);
 reset();begin_manual();theta=-17158;expect_fault(4);
 reset();begin_manual();clocks(100004);if(state!=5||fault!=3||command)$fatal(1,"watchdog");checks++;
 reset();begin_manual();step=1;clocks(1);step=0;clocks(3);step=1;clocks(1);step=0;clocks(1);if(state!=5||fault!=3)$fatal(1,"overlap");checks++;
 fast_track_seed();force dut.session_ticks=119999;clocks(1);release dut.session_ticks;run_tick();if(state==5)$fatal(1,"early timeout");run_tick();if(state!=5||fault!=8||command)$fatal(1,"session timeout");checks++;
 reset();begin_manual();measurement_ok=0;step=1;request_clear=1;clocks(1);step=0;request_clear=0;if(dut.trip_code!=2||state==5)$fatal(1,"trip register");clocks(1);if(state!=5||fault!=2||command)$fatal(1,"trip consumed");checks++;
 reset();begin_manual();measurement_ok=0;step=1;clocks(1);step=0;request_stop=1;clocks(1);request_stop=0;measurement_ok=1;clocks(3);if(state!=0||command||dut.trip_code)$fatal(1,"STOP priority");begin_manual();run_tick();if(state!=2||fault)$fatal(1,"stale trip");checks++;
 reset();begin_manual();step=1;clocks(1);step=0;while(dut.phase!=4)clocks(1);
 force dut.observer_ready=1;force dut.observer_u=151;force dut.observer_done=1;clocks(1);release dut.observer_ready;release dut.observer_u;release dut.observer_done;clocks(1);
 if(state!=5||fault!=7||command)$fatal(1,"transaction cap bypass");checks++;
 // STOP, disarm, reset at every clock, including the observer commit.
 reset();begin_manual();run_tick();lat_track=last_latency;
 for(j=0;j<3;j++)begin
  for(abort_clock=0;abort_clock<=lat_track;abort_clock++)begin
   reset();begin_manual();step=1;clocks(1);step=0;clocks(abort_clock);
   if(j==0)begin request_stop=1;clocks(1);request_stop=0;end
   else if(j==1)begin armed=0;clocks(1);end
   else begin #1 rst_n=0;#1;if(state!=0||command)$fatal(1,"async reset");rst_n=1;end
   clocks(2);if(state!=0||command||dut.phase!=0)$fatal(1,"cancel %d %d",j,abort_clock);
   clocks(2000);if(state!=0||command||result_valid)$fatal(1,"stale result");
  end
  checks++;
 end
 // Automatic timeout now originates in the compute core (attempt register).
 // Its 11999->12000 boundary is covered by test_cpu.cpp transaction 220.
 // Here verify delivery of that core fault through the supervisor.
 reset();begin_auto();step=1;clocks(1);step=0;while(dut.phase!=2)clocks(1);
 force dut.swing_fault=1;force dut.swing_fault_code=5;clocks(1);
 release dut.swing_fault;release dut.swing_fault_code;clocks(2);
 if(state!=5||fault!=5||command)$fatal(1,"core swing timeout delivery");checks++;
 for(j=0;j<8;j++)begin
  reset();begin_auto();step=1;clocks(1);step=0;clocks(j*1000);request_stop=1;clocks(1);request_stop=0;clocks(49900);if(state!=0||command||dut.phase)$fatal(1,"swing abort");checks++;
 end
 $display("PASS supervisor checks=%0d latency=%0d every-clock STOP/disarm/reset",checks,lat_track);$finish;
 end
 initial begin #4000000000;$fatal(1,"global timeout");end
endmodule
