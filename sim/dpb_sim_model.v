// Simulation only: exact supported DPB subset. Vendor primitive is used for synthesis.
// synthesis translate_off
module DPB #(parameter READ_MODE0=1'b0,READ_MODE1=1'b0,WRITE_MODE0=2'b00,WRITE_MODE1=2'b00,BIT_WIDTH_0=16,BIT_WIDTH_1=16,BLK_SEL_0=3'b0,BLK_SEL_1=3'b0,RESET_MODE="SYNC",
parameter INIT_RAM_00=256'h0,
parameter INIT_RAM_01=256'h0,
parameter INIT_RAM_02=256'h0,
parameter INIT_RAM_03=256'h0,
parameter INIT_RAM_04=256'h0,
parameter INIT_RAM_05=256'h0,
parameter INIT_RAM_06=256'h0,
parameter INIT_RAM_07=256'h0,
parameter INIT_RAM_08=256'h0,
parameter INIT_RAM_09=256'h0,
parameter INIT_RAM_0A=256'h0,
parameter INIT_RAM_0B=256'h0,
parameter INIT_RAM_0C=256'h0,
parameter INIT_RAM_0D=256'h0,
parameter INIT_RAM_0E=256'h0,
parameter INIT_RAM_0F=256'h0,
parameter INIT_RAM_10=256'h0,
parameter INIT_RAM_11=256'h0,
parameter INIT_RAM_12=256'h0,
parameter INIT_RAM_13=256'h0,
parameter INIT_RAM_14=256'h0,
parameter INIT_RAM_15=256'h0,
parameter INIT_RAM_16=256'h0,
parameter INIT_RAM_17=256'h0,
parameter INIT_RAM_18=256'h0,
parameter INIT_RAM_19=256'h0,
parameter INIT_RAM_1A=256'h0,
parameter INIT_RAM_1B=256'h0,
parameter INIT_RAM_1C=256'h0,
parameter INIT_RAM_1D=256'h0,
parameter INIT_RAM_1E=256'h0,
parameter INIT_RAM_1F=256'h0,
parameter INIT_RAM_20=256'h0,
parameter INIT_RAM_21=256'h0,
parameter INIT_RAM_22=256'h0,
parameter INIT_RAM_23=256'h0,
parameter INIT_RAM_24=256'h0,
parameter INIT_RAM_25=256'h0,
parameter INIT_RAM_26=256'h0,
parameter INIT_RAM_27=256'h0,
parameter INIT_RAM_28=256'h0,
parameter INIT_RAM_29=256'h0,
parameter INIT_RAM_2A=256'h0,
parameter INIT_RAM_2B=256'h0,
parameter INIT_RAM_2C=256'h0,
parameter INIT_RAM_2D=256'h0,
parameter INIT_RAM_2E=256'h0,
parameter INIT_RAM_2F=256'h0,
parameter INIT_RAM_30=256'h0,
parameter INIT_RAM_31=256'h0,
parameter INIT_RAM_32=256'h0,
parameter INIT_RAM_33=256'h0,
parameter INIT_RAM_34=256'h0,
parameter INIT_RAM_35=256'h0,
parameter INIT_RAM_36=256'h0,
parameter INIT_RAM_37=256'h0,
parameter INIT_RAM_38=256'h0,
parameter INIT_RAM_39=256'h0,
parameter INIT_RAM_3A=256'h0,
parameter INIT_RAM_3B=256'h0,
parameter INIT_RAM_3C=256'h0,
parameter INIT_RAM_3D=256'h0,
parameter INIT_RAM_3E=256'h0,
parameter INIT_RAM_3F=256'h0)(
 output reg[15:0]DOA,DOB,input[15:0]DIA,DIB,input[2:0]BLKSELA,BLKSELB,input[13:0]ADA,ADB,
 input WREA,WREB,CLKA,CLKB,CEA,CEB,OCEA,OCEB,RESETA,RESETB);
reg[15:0]mem[0:1023];
wire[16383:0]init_image={INIT_RAM_3F,INIT_RAM_3E,INIT_RAM_3D,INIT_RAM_3C,INIT_RAM_3B,INIT_RAM_3A,INIT_RAM_39,INIT_RAM_38,INIT_RAM_37,INIT_RAM_36,INIT_RAM_35,INIT_RAM_34,INIT_RAM_33,INIT_RAM_32,INIT_RAM_31,INIT_RAM_30,INIT_RAM_2F,INIT_RAM_2E,INIT_RAM_2D,INIT_RAM_2C,INIT_RAM_2B,INIT_RAM_2A,INIT_RAM_29,INIT_RAM_28,INIT_RAM_27,INIT_RAM_26,INIT_RAM_25,INIT_RAM_24,INIT_RAM_23,INIT_RAM_22,INIT_RAM_21,INIT_RAM_20,INIT_RAM_1F,INIT_RAM_1E,INIT_RAM_1D,INIT_RAM_1C,INIT_RAM_1B,INIT_RAM_1A,INIT_RAM_19,INIT_RAM_18,INIT_RAM_17,INIT_RAM_16,INIT_RAM_15,INIT_RAM_14,INIT_RAM_13,INIT_RAM_12,INIT_RAM_11,INIT_RAM_10,INIT_RAM_0F,INIT_RAM_0E,INIT_RAM_0D,INIT_RAM_0C,INIT_RAM_0B,INIT_RAM_0A,INIT_RAM_09,INIT_RAM_08,INIT_RAM_07,INIT_RAM_06,INIT_RAM_05,INIT_RAM_04,INIT_RAM_03,INIT_RAM_02,INIT_RAM_01,INIT_RAM_00};
integer i;
initial begin
 if(READ_MODE0!=0||READ_MODE1!=0||WRITE_MODE0!=0||WRITE_MODE1!=0||BIT_WIDTH_0!=16||BIT_WIDTH_1!=16||RESET_MODE!="SYNC")$fatal(1,"Unsupported DPB model configuration");
 DOA=0;DOB=0;for(i=0;i<1024;i=i+1)mem[i]=init_image[i*16+:16];
end
always @(posedge CLKA)begin
 if(CEA&&BLKSELA==BLK_SEL_0&&WREA)begin
  if(ADA[0])mem[ADA[13:4]][7:0]<=DIA[7:0];
  if(ADA[1])mem[ADA[13:4]][15:8]<=DIA[15:8];
 end
 if(RESETA)DOA<=0;else if(CEA&&BLKSELA==BLK_SEL_0&&!WREA)DOA<=mem[ADA[13:4]];
end
always @(posedge CLKB)begin
 if(CEB&&WREB)$fatal(1,"Port B is read only in R17");
 if(RESETB)DOB<=0;else if(CEB&&BLKSELB==BLK_SEL_1&&!WREB)DOB<=mem[ADB[13:4]];
end
endmodule
// synthesis translate_on
