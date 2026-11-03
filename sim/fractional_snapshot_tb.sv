`timescale 1ns/1ps
module fractional_snapshot_tb;
 reg clk=0,rst_n=0,tick=0,adc_ready=1,calibrate=0,gap_allow=0,restart_observation=0;
 reg[9:0]adc=740,zero_raw=740;
 reg[13:0]adc_q4=11848,zero_q4=11848;reg adc_q4_valid=1;
 reg signed[31:0]encoder=0;
 wire step_valid,measurement_ok,gap_active;wire[7:0]gap_ms;
 wire signed[31:0]theta,dtheta,phi,dphi,theta_raw_q16,filtered_theta_q18;
 state_estimator dut(.*);
 integer n,i,k,checks=0;
 task cycle;begin clk=0;#10;clk=1;#10;end endtask
 task init;begin rst_n=0;tick=0;calibrate=0;adc=740;zero_raw=740;adc_q4=11848;zero_q4=11848;adc_q4_valid=1;cycle;rst_n=1;cycle;end endtask
 task launch;begin tick=1;cycle;tick=0;end endtask
 task finish_sample;begin n=0;while(!step_valid && n<1000)begin cycle;n=n+1;end
 if(!step_valid)$fatal(1,"missed sample");checks=checks+1;end endtask
 initial begin
 init;launch;
 // All four fine/raw/zero inputs mutate after acceptance, but commit uses snapshot.
 adc=760;zero_raw=700;adc_q4=12160;zero_q4=11200;
 finish_sample;
 if(theta!=0 || dtheta!=0 || filtered_theta_q18!=0 || theta_raw_q16!=0)$fatal(1,"input snapshot broken");checks=checks+1;
 if(dut.gap_bridge.offset!=11848*6176)$fatal(1,"gap fractional zero was truncated");checks=checks+1;
 for(k=1;k<=19;k=k+1)begin
  init;launch;
  for(i=0;i<k;i=i+1)cycle;
  calibrate=1;cycle;calibrate=0;
  for(i=0;i<600;i=i+1)begin cycle;if(step_valid)$fatal(1,"stale completion after calibration");checks=checks+1;end
  launch;finish_sample;
  if(theta!=0 || dtheta!=0 || filtered_theta_q18!=0)$fatal(1,"calibration restart error");checks=checks+1;
 end
 // Abort while bridge is busy, later than stage16 arrival.
 for(k=20;k<=180;k=k+20)begin
  init;launch;for(i=0;i<k;i=i+1)cycle;
  rst_n=0;cycle;rst_n=1;
  for(i=0;i<600;i=i+1)begin cycle;if(step_valid)$fatal(1,"stale completion after reset");checks=checks+1;end
  launch;finish_sample;if(theta!=0 || dtheta!=0)$fatal(1,"reset restart error");checks=checks+1;
 end
 $display("PASS %0d estimator snapshot/calibration/reset assertions",checks);$finish;
 end
endmodule
