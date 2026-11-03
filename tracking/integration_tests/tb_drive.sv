`timescale 1ns/1ps
module tb_drive;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,enable=0,clear=0;reg signed[31:0] request=0,limit=200;
 wire allowed,tripped;wire signed[31:0] safe,applied,registered_applied;
 wire a,b,p,oa,ob,op;
 drive_guard guard(.clk(clk),.rst_n(rst_n),.enable(enable),.clear(clear),.requested(request),.limit(limit),.allowed(allowed),.safe_command(safe),.tripped(tripped));
 motor_pwm motor(.clk(clk),.rst_n(rst_n),.enable(allowed),.command(safe),.in1(a),.in2(b),.pwm(p),.applied_debug(applied));
 motor_output_register regs(.clk(clk),.rst_n(rst_n),.in1(a),.in2(b),.pwm(p),.applied(applied),.out1(oa),.out2(ob),.out_pwm(op),.out_applied(registered_applied));
 integer i,j,on_count,checks=0;
 task clocks(input integer n);repeat(n)@(negedge clk);endtask
 task reset;
 begin enable=0;clear=0;rst_n=0;request=0;limit=350;clocks(3);rst_n=1;clocks(2);end endtask
 initial begin
  reset();enable=1;
  for(i=-350;i<=350;i=i+1)begin
   request=i;while(motor.counter!=2499)clocks(1);clocks(1);
   if(applied!=i)begin if(applied!=0)$fatal(1,"reverse break invalid");clocks(2500);end
   if(applied!=i)$fatal(1,"amplitude clipped/wrapped %d %d",i,applied);
   on_count=0;
   for(j=0;j<2500;j=j+1)begin
    if(p)on_count++;
    if(i>0&&(!a||b))$fatal(1,"positive direction");if(i<0&&(!b||a))$fatal(1,"negative direction");
    if(i==0&&(a||b||!p))$fatal(1,"zero should coast");clocks(1);
   end
   if(on_count!=(i==0?2500:((i<0?-i:i)*2500/1000)))$fatal(1,"PWM threshold %d got %d",i,on_count);
   checks++;
  end
  // New balance envelope: every integer duty, both boundary violations, old limit rejected.
  limit=200;
  for(i=-200;i<=200;i++)begin request=i;clocks(1);if(!allowed||tripped||safe!=i)$fatal(1,"200 envelope %d",i);checks++;end
  for(i=-1;i<=1;i=i+2)begin
   reset();enable=1;limit=200;request=i*201;clocks(1);
   if(allowed||!tripped||safe)$fatal(1,"201 boundary not rejected");checks++;
  end
  reset();enable=1;limit=149;request=1;clocks(1);if(allowed||!tripped)$fatal(1,"invalid149 mode accepted");checks++;
  reset();enable=1;
  request=350;clocks(5000);limit=200;clocks(1);if(allowed||!tripped||safe)$fatal(1,"mode limit transition failed");clocks(1);if(oa||ob||!op)$fatal(1,"register coast after guardtrip");checks++;
  enable=0;clear=1;clocks(1);clear=0;if(tripped)$fatal(1,"guard clear");
  enable=1;limit=350;request=-2147483648;clocks(1);if(allowed||!tripped)$fatal(1,"wide corruption wraps");checks++;
  reset();enable=1;limit=351;request=1;clocks(1);if(allowed||!tripped)$fatal(1,"invalid mode limit");checks++;
  for(i=150;i<=350;i++)begin reset();limit=i;request=i;enable=1;clocks(1);if(!allowed||tripped)$fatal(1,"dynamic cap positive");request=-i;clocks(1);if(!allowed||tripped)$fatal(1,"dynamic cap negative");request=i+1;clocks(1);if(allowed||!tripped)$fatal(1,"dynamic cap overflow");checks++;end
  reset();enable=1;request=350;clocks(5000);enable=0;clocks(1);if(oa||ob||!op)$fatal(1,"disable coast");checks++;
  reset();enable=1;request=350;clocks(5000);#1 rst_n=0;#1;if(oa||ob||!op||registered_applied)$fatal(1,"async reset coast");checks++;
  $display("PASS drive checks=%0d all701duties, ±350 limits, guard transition/latch, INT32_MIN, physical STOP/reset",checks);$finish;
 end
endmodule
