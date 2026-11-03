from pathlib import Path
import json,re
D=Path(__file__).resolve().parent;a=json.loads((D/'hybrid_program.json').read_text());old=a['names'];const={i for n,i in old.items()if n.startswith(('constant_','bank[','coefficients[','phase_commands['))};order=[i for i in range(len(a['initial']))if i not in const]+sorted(const);mapping={v:i for i,v in enumerate(order)};names={n:mapping[i]for n,i in old.items()};data=[a['initial'][i]for i in order];limit=len(order)-len(const);code=[]
for op,d,x,y in a['code']:code.append([op,d if op in [22,23,24]else mapping[d],mapping[x],mapping[y]])
# Restore nonzero mutable state through ordinary instructions after zeroing RAM.
entry=len(code)
for oldi in order[:limit]:
 val=a['initial'][oldi]
 if val:
  try:k=next(mapping[i]for i in const if a['initial'][i]==val)
  except StopIteration:k=len(data);data.append(val)
  code.append([1,mapping[oldi],k,0])
code.append([29,0,0,0]);assert len(code)<=2048 and len(data)<=1024
(D/'layout.json').write_text(json.dumps(dict(names=names,initial=data,code=code,writable=limit,initialize_entry=entry,instruction_capacity=2048,data_capacity=1024),indent=2))
assert names['resume_elapsed']==79
progress_address=names['progress_measurement_valid']
(D/'r29_swing_io.vh').write_text(
 '// Generated from the same remapped memory layout as s23_memory.v.\n'
 f'localparam [9:0] R29_PROGRESS_VALID_ADDRESS=10\'d{progress_address};\n'
 f"localparam [9:0] R34_OBSERVER_READY_ADDRESS=10'd{names['observer_ready_input']};\n")
(D/'layout.hpp').write_text('#pragma once\nstatic const int OLD_WORDS='+str(len(order))+';static const int WRITABLE='+str(limit)+';static const int MAP[]={'+','.join(str(mapping[i])for i in range(len(order)))+'};\n')
s='''`default_nettype none
module s23_compute(input wire clk,rst_n,cancel,initialize,start,
 input wire ext_write,input wire[9:0]ext_address,input wire signed[47:0]ext_data,
 output wire signed[47:0]ext_read,output reg ready,busy,done,fault,output reg[31:0]cycles);
 reg[3:0]phase;reg[10:0]pc;reg[9:0]destination,clear_address;reg initializing;
 reg[36:0]ir;reg[10:0]stack[0:7];reg[3:0]sp;
 wire[5:0]op=ir[36:31];wire[10:0]rd=ir[30:20];wire[9:0]ra=ir[19:10],rb=ir[9:0];
 reg[9:0]aa,ab;reg we;reg signed[47:0]wd;wire signed[47:0]va,vb;
 assign ext_read=vb;
 reg write_q;reg[9:0]write_address_q;reg[47:0]write_data_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin write_q<=0;write_address_q<=0;write_data_q<=0;end
  else begin write_q<=we&&!cancel&&!initialize;write_address_q<=aa;write_data_q<=wd;end
 end
 s23_memory mem(.clk(clk),.we(write_q&&!cancel&&!initialize),.aa(write_q?write_address_q:aa),.ab(ab),.wd(write_data_q),.qa(va),.qb(vb));
 (* rom_style="block" *) reg[36:0]rom[0:2047];
 initial begin
'''
for i in range(2048):
 o,d,x,y=code[i]if i<len(code)else[0]*4;s+=f"rom[{i}]=37'h{(o<<31)|(d<<20)|(x<<10)|y:010x};\n"
s+='''end
 always @(posedge clk)if(phase==1)ir<=rom[pc];
 wire arun=phase==3&&(op==4||op==5||op==10||op==11||op==12);
 wire adone,abusy,afault;wire signed[47:0]ar;
 s23_alu #(.SUPPORT_TANH(0)) alu(.clk(clk),.rst_n(rst_n),.cancel(cancel||initialize),.start(arun),.op(op),.a(va),.b(vb),.busy(abusy),.done(adone),.fault(afault),.result(ar));
 wire signed[48:0]sum={va[47],va}+{vb[47],vb},diff={va[47],va}-{vb[47],vb};
 always @*begin
 aa=0;ab=ext_address;we=0;wd=0;
 if(phase==2)begin aa=ra;ab=rb;end
 else if(phase==3)begin
 aa=rd[9:0];ab=rb;
 case(op)
'''
exprs={1:'va',2:'sum[47:0]',3:'diff[47:0]',6:'-va',7:'va<0?-va:va',8:'va<vb?va:vb',9:'va>vb?va:vb',14:'va<vb',15:'va<=vb',16:'va>vb',17:'va>=vb',18:'va==vb',19:'va!=vb',20:'va!=0&&vb!=0',21:'va!=0||vb!=0',31:'va >>> (vb>>>25)',32:'va <<< (vb>>>25)'}
for op,e in exprs.items():
 if 14<=op<=21:e=f"({e})?48'sd33554432:48'sd0"
 s+=f"{op}:begin we=1;wd={e};end\n"
s+='''26:aa=ra+vb[34:25];
27:begin aa=rd[9:0]+va[34:25];we=1;wd=vb;end
endcase end
else if(phase==4&&adone&&!afault)begin aa=destination;we=1;wd=ar;end
else if(phase==5)begin aa=destination;we=1;wd=va;end
else if(phase==6)begin aa=clear_address;we=1;wd=0;end
else if(phase==0&&ready&&ext_write)begin aa=ext_address;we=1;wd=ext_data;end
'''+f"if(aa>=10'd{limit})we=0;\nend\n"
s+='''always @(posedge clk or negedge rst_n)begin
if(!rst_n)begin phase<=0;pc<=0;sp<=0;ready<=0;busy<=0;done<=0;fault<=0;cycles<=0;initializing<=0;clear_address<=0;destination<=0;end
else if(cancel)begin phase<=0;sp<=0;ready<=0;busy<=0;done<=0;fault<=0;initializing<=0;end
else if(initialize)begin phase<=6;sp<=0;ready<=0;busy<=1;done<=0;fault<=0;cycles<=0;initializing<=1;clear_address<=0;end
else begin
 if(busy)cycles<=cycles+1'b1;
 case(phase)
 0:if(start&&ready&&!fault)begin phase<=1;pc<=0;sp<=0;busy<=1;done<=0;cycles<=0;end
 1:phase<=2;
 2:phase<=3;
 3:begin
 pc<=pc+1'b1;phase<=1;
 if(op==0||op==29)begin
  if(sp!=0)begin sp<=sp-1'b1;pc<=stack[sp[2:0]-3'd1];end
  else begin busy<=0;done<=1;phase<=0;if(initializing)begin initializing<=0;ready<=1;end end
 end else if(op==22)begin if(va==0)pc<=rd;end
 else if(op==23)pc<=rd;
 else if(op==24)begin
 if(sp==8)begin fault<=1;busy<=0;done<=1;phase<=0;end
 else begin stack[sp[2:0]]<=pc+1'b1;sp<=sp+1'b1;pc<=rd;end
 end else if(op==25)begin
 if(sp==0)begin fault<=1;busy<=0;done<=1;phase<=0;end
 else begin sp<=sp-1'b1;pc<=stack[sp[2:0]-3'd1];end
 end else if(op==26)begin phase<=5;destination<=rd[9:0];end
 else if(arun)begin phase<=4;destination<=rd[9:0];end
 else if((op==2&&sum[48]!=sum[47])||(op==3&&diff[48]!=diff[47])||op==13)begin fault<=1;busy<=0;done<=1;phase<=0;end
 end
 4:if(adone)begin
 if(afault)begin fault<=1;busy<=0;done<=1;phase<=0;end else phase<=1;
 end
 5:phase<=1;
 6:begin
'''+f"if(clear_address==10'd{limit-1})begin pc<=11'd{entry};phase<=1;end else clear_address<=clear_address+1'b1;\n"
s+='''end
 default:begin fault<=1;busy<=0;done<=1;phase<=0;end
 endcase
end end
endmodule
`default_nettype wire
'''
(D/'s23_compute.v').write_text(s)
# Explicit supported GW2A dual-port RAM configuration, preserved byte enables.
s='''module s23_memory(input wire clk,we,input wire[9:0]aa,ab,input wire[47:0]wd,output wire[47:0]qa,qb);
'''
for lane in range(3):
 s+='DPB #(.READ_MODE0(1\'b0),.READ_MODE1(1\'b0),.WRITE_MODE0(2\'b00),.WRITE_MODE1(2\'b00),.BIT_WIDTH_0(16),.BIT_WIDTH_1(16),.BLK_SEL_0(3\'b000),.BLK_SEL_1(3\'b000),.RESET_MODE("SYNC"),\n'
 words=[((data[i]if i<len(data)else 0)>>(16*lane))&65535 for i in range(1024)]
 for block in range(64):
  val=sum(words[16*block+j]<<(16*j)for j in range(16));s+=f'.INIT_RAM_{block:02X}(256\'h{val:064x})'+(','if block<63 else '')+'\n'
 s+=f''') ram{lane}(.CLKA(clk),.CLKB(clk),.CEA(1'b1),.CEB(!we),.OCEA(1'b1),.OCEB(1'b1),.WREA(we),.WREB(1'b0),.RESETA(1'b0),.RESETB(1'b0),.BLKSELA(3'b000),.BLKSELB(3'b000),.ADA({{aa,4'b0011}}),.ADB({{ab,4'b0011}}),.DIA(wd[{16*lane+15}:{16*lane}]),.DIB(16'd0),.DOA(qa[{16*lane+15}:{16*lane}]),.DOB(qb[{16*lane+15}:{16*lane}]));\n'''
s+='endmodule\n';(D/'s23_memory.v').write_text(s)
source=D
s=(source/'alu_template.v').read_text().replace('module r17_fixed_alu','module s23_alu');(D/'s23_alu.v').write_text(s)
print('RTL',len(code),'instructions',limit,'mutable words',len(data),'total words')

# Force LUT arithmetic: spare DSP count is only one on the baseline device.
s=(D/'s23_alu.v').read_text().replace('wire[47:0] partial_product=mul_left*mul_right;','wire[47:0] partial_product; s23_soft_mul24 mul(.a(mul_left),.b(mul_right),.product(partial_product));')
s=s.replace('reg[1:0] multiply_part;', 'reg[1:0] multiply_part;\n wire signed[31:0] lut_delta=lv1-lv0;\n wire signed[40:0] lut_product='+ '+'.join(f"(fraction[{i}] ? ($signed({{lut_delta[31],lut_delta}}) <<< {i}) : 41'sd0)" for i in range(8))+';')
s=s.replace("$signed(lv1-lv0)*$signed({1'b0,fraction})",'lut_product');(D/'s23_alu.v').write_text(s)
s=(source/'soft_template.v').read_text();s=s[s.index('module r17_soft_mul24'):].replace('r17_soft_mul24','s23_soft_mul24');(D/'s23_soft_mul24.v').write_text(s)
# Four multiplier bits per clock: exact product, bounded extra latency, zero DSP.
s=(D/'s23_alu.v').read_text().replace('s23_soft_mul24 mul(.a(mul_left),.b(mul_right),.product(partial_product));', 's23_soft_mul24 mul(.clk(clk),.rst_n(rst_n),.cancel(cancel),.start(phase==3),.a(mul_left),.b(mul_right),.product(partial_product),.done(multiply_done));\n wire multiply_done;')
s=s.replace('    3:begin\n     if(single_part', '    3:phase<=26;\n    26:if(multiply_done)phase<=27;\n    27:begin\n     if(single_part')
s=s.replace('if(next_parts==0)phase<=4;', 'if(next_parts==0)phase<=4;else phase<=3;')
(D/'s23_alu.v').write_text(s)

(D/'s23_soft_mul24.v').write_text('''module s23_soft_mul24(input wire clk,rst_n,cancel,start,input wire[23:0]a,b,output reg[47:0]product,output reg done);
reg active;reg[2:0]count;reg[23:0]left,right;reg[47:0]acc;
wire[27:0]p0={4'd0,left & {24{right[0]}}};
wire[27:0]p1={3'd0,left & {24{right[1]}},1'b0};
wire[27:0]p2={2'd0,left & {24{right[2]}},2'b0};
wire[27:0]p3={1'd0,left & {24{right[3]}},3'b0};
wire[27:0]x=p0^p1^p2,y=((p0&p1)|(p0&p2)|(p1&p2))<<1;
wire[27:0]z=x^y^p3,c=((x&y)|(x&p3)|(y&p3))<<1;
wire[47:0]next=(acc>>4)+{(z+c),20'd0};
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin active<=0;done<=0;product<=0;count<=0;left<=0;right<=0;acc<=0;end
 else if(cancel)begin active<=0;done<=0;product<=0;end
 else begin done<=0;
 if(start&&!active)begin left<=a;right<=b;acc<=0;count<=0;active<=1;end
 else if(active)begin acc<=next;right<=right>>4;count<=count+1'b1;if(count==5)begin product<=next;done<=1;active<=0;end end
 end end
endmodule
''')

# R22R2 pipeline the full-precision LUT interpolation; no table/rounding changes.
s=(D/'s23_alu.v').read_text()
s=s.replace('wire signed[31:0] lut_delta=lv1-lv0;', 'reg signed[31:0] lut_delta; reg signed[40:0] lp0,lp1,lp2,lp3,ls0,ls1;')
s=s.replace('20:begin wide<=lut_product;phase<=21;end', """20:begin lut_delta<=lv1-lv0;phase<=30;end
30:begin
 lp0<=(fraction[0]?($signed({lut_delta[31],lut_delta})<<<0):41'sd0)+(fraction[1]?($signed({lut_delta[31],lut_delta})<<<1):41'sd0);
 lp1<=(fraction[2]?($signed({lut_delta[31],lut_delta})<<<2):41'sd0)+(fraction[3]?($signed({lut_delta[31],lut_delta})<<<3):41'sd0);
 lp2<=(fraction[4]?($signed({lut_delta[31],lut_delta})<<<4):41'sd0)+(fraction[5]?($signed({lut_delta[31],lut_delta})<<<5):41'sd0);
 lp3<=(fraction[6]?($signed({lut_delta[31],lut_delta})<<<6):41'sd0)+(fraction[7]?($signed({lut_delta[31],lut_delta})<<<7):41'sd0);phase<=31;end
31:begin ls0<=lp0+lp1;ls1<=lp2+lp3;phase<=32;end
32:begin wide<=ls0+ls1;phase<=21;end""")
# Split input sign/add and LUT sign/output stages instead of combined 48-bit mux/add paths.
s=s.replace("6'd10,6'd11:begin if(aa>","6'd10,6'd11:begin if(aa>")
s=s.replace("angle<=aa+(operation==6'd11 ? 48'sd52707178:48'sd0);table_id<=0;phase<=8;", "angle<=aa;table_id<=0;phase<=33;")
s=s.replace('21:begin result<=lut_negative?-$signed(interpolated[47:0]):$signed(interpolated[47:0]);phase<=15;end', '21:begin wide<=interpolated;phase<=34;end\n34:begin result<=lut_negative?-$signed(wide[47:0]):$signed(wide[47:0]);phase<=15;end\n33:begin if(operation==11)angle<=angle+48\'sd52707178;phase<=8;end')
(D/'s23_alu.v').write_text(s)

# R26R2 implementation-only comparator staging, exploiting existing phase28.
s=(D/'s23_alu.v').read_text()
s=s.replace('reg signed[47:0] aa,bb;', 'reg signed[47:0] aa,bb;\n reg less_q,equal_q;')
s=s.replace('28:begin', '28:begin\n     less_q<=aa<bb;equal_q<=aa==bb;')
for before,after in [('aa<bb?','less_q?'),('aa>bb?','(!less_q && !equal_q)?'),('aa<=bb?','(less_q || equal_q)?'),('aa>=bb?','(!less_q)?'),('aa==bb?','equal_q?'),('aa!=bb?','(!equal_q)?')]:s=s.replace(before,after)
(D/'s23_alu.v').write_text(s)

# R38T1 timing: phase28 already registers compare flags before operation1.
# For DIV only, equal_q is otherwise dead: use it as the zero-denominator
# flag, breaking bb -> abs/zero/mux -> result without adding a clock or FF.
s=(D/'s23_alu.v').read_text()
assert s.count('less_q<=aa<bb;equal_q<=aa==bb;')==1
assert s.count('if(bb==0)begin fault<=1;result<=0;phase<=15;end')==1
s=s.replace('less_q<=aa<bb;equal_q<=aa==bb;',
 'less_q<=aa<bb;equal_q<=operation==6\'d5 ? bb==0 : aa==bb;')
s=s.replace('if(bb==0)begin fault<=1;result<=0;phase<=15;end',
 'if(equal_q)begin fault<=1;result<=0;phase<=15;end')
(D/'s23_alu.v').write_text(s)

# R39T2 exact same-edge mutually exclusive CPU shift sharing.
from r39t2_shift_share import transform as r39t2_shift_transform
(D/'s23_alu.v').write_text(r39t2_shift_transform((D/'s23_alu.v').read_text()))
