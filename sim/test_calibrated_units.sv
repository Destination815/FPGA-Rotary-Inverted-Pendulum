`timescale 1ns/1ps
`include "j280_coordinates.vh"
module test_calibrated_units;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,tick=0,cal=0,stop=0,valid=0;reg signed[31:0] enc=0,value=0;reg[7:0]seq=0,op=0;
 wire signed[31:0] zero,phi,out_value;wire sat,out_valid,overflow;wire[7:0]out_seq,out_op;
 calibrated_position p(.clk(clk),.rst_n(rst),.tick(tick),.calibrate(cal),.encoder(enc),.encoder_zero(zero),.phi_physical(phi),.saturated(sat));
 calibrated_command c(.clk(clk),.rst_n(rst),.stop_level(stop),.in_valid(valid),.in_seq(seq),.in_opcode(op),.in_value(value),.out_valid(out_valid),.out_seq(out_seq),.out_opcode(out_op),.out_value(out_value),.overflow(overflow));
 integer fd,code,n=0,i;reg signed[31:0] count_v,zero_v,target_v,expected_phi,expected_target;integer expected_sat,expected_overflow;
 task clocks(input integer count);repeat(count)begin @(negedge clk);#1;end endtask
 initial begin
  clocks(3);rst=1;clocks(3);
  fd=$fopen("vectors.txt","r");if(!fd)$fatal(1,"missing test vectors");
  while(!$feof(fd))begin
   code=$fscanf(fd,"%d %d %d %d %d %d %d\n",count_v,zero_v,target_v,expected_phi,expected_sat,expected_target,expected_overflow);
   if(code==7)begin
    enc=zero_v;cal=1;clocks(1);cal=0;clocks(1);
    enc=count_v;tick=1;clocks(1);tick=0;
    value=target_v;seq=n;op=8'h26;valid=1;clocks(1);valid=0;
    // Mapping must finish by28 clocks. Physical export must already be ready.
    for(i=0;i<26;i++)begin
     clocks(1);
     if(i<24 && out_valid)$fatal(1,"early target commit");
    end
    if(phi!==expected_phi || sat!==expected_sat[0])$fatal(1,"position mismatch vector%0d count%0d zero%0d actual%0d expected%0d",n,count_v,zero_v,phi,expected_phi);
    if(!out_valid || out_value!==expected_target || overflow!==expected_overflow[0] || out_seq!==seq || out_op!=8'h26)$fatal(1,"target mapping vector%0d out%0d expected%0d valid%b overflow%b expected%d",n,out_value,expected_target,out_valid,overflow,expected_overflow);
    clocks(2);n++;
   end
  end
  $fclose(fd);
  // Local STOP cancellation at every phase; immediate raw STOP is tested at j280_top.
  for(i=0;i<27;i++)begin
   op=8'h26;value=123456;valid=1;clocks(1);valid=0;clocks(i);
   op=1;value=0;valid=1;stop=1;#1;
   clocks(1);valid=0;stop=0;repeat(30)begin clocks(1);if(out_valid)$fatal(1,"stale target afterSTOP");end
  end
  for(i=0;i<27;i++)begin
   op=8'h26;value=-123456;valid=1;clocks(1);valid=0;clocks(i);
   stop=1;clocks(1);stop=0;repeat(30)begin clocks(1);if(out_valid)$fatal(1,"stale target afterSW4");end
  end
  $display("PASS calibrated units vectors=%0d STOP/SW4 cancellation phases=54",n);$finish;
 end
endmodule
