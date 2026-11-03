module test_r26r2_alu;
reg clk=0; always #5 clk=~clk;
reg rst_n=0,start=0,cancel=0; reg[5:0]op;reg signed[47:0]a,b;
wire f0,f1,b0,b1,d0,d1;wire signed[47:0]r0,r1;
r26_alu oldcore(clk,rst_n,start,cancel,op,a,b,f0,b0,d0,r0);
s23_alu newcore(clk,rst_n,start,cancel,op,a,b,f1,b1,d1,r1);
task tick;begin @(posedge clk);#1;assert({f0,b0,d0}=={f1,b1,d1})else $fatal(1,"flags");if(d0)assert(r0==r1)else $fatal(1,"result op%0d a%0d b%0d",op,a,b);@(negedge clk);end endtask
integer i,j,n;reg[31:0]seed=32'h2620;reg[47:0]edges[0:6];
initial begin edges[0]=0;edges[1]=1;edges[2]=-1;edges[3]=48'h7fffffffffff;edges[4]=48'h800000000000;edges[5]=33554432;edges[6]=-33554432;
tick();rst_n=1;tick();
for(i=0;i<20000;i=i+1)begin
op=(i%8<2)?8+i%8:12+i%8;
a={$urandom(seed),$urandom(seed)};b={$urandom(seed),$urandom(seed)};
if(i<392)begin a=edges[(i/8)%7];b=edges[(i/56)%7];end
start=1;tick();start=0;n=0;while(!d0 && n<150)begin tick();n=n+1;end
assert(d0)else $fatal(1,"deadline");tick();
end
for(i=0;i<8;i=i+1)for(j=0;j<4;j=j+1)begin op=(i<2)?8+i:12+i;a=-12345678;b=98765;start=1;tick();start=0;repeat(j)tick();cancel=1;tick();cancel=0;repeat(6)tick();assert(!b0 && !d0)else $fatal(1,"cancel");end
$display("PASS 20000 old/new ALU transactions, 392 signed edge combinations, 32 cancellations; result/flags/latency exact");$finish;end
endmodule
