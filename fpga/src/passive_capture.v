`default_nettype none
// Isolated C831/v1 prototype. No connection from this module to motion control.
// First 4096 samples are immutable; following samples occupy a 4096-row ring.
// Every row contains its own true millis. Gaps and duplicate timestamps survive.
module passive_capture(
 input wire clk,rst_n,begin_record,automatic_start,sample,stop,
 input wire[31:0] millis, input wire[13:0] adc_q4,
 input wire adc_fine_valid, input wire signed[31:0] encoder,command_pwm,applied_pwm,
 input wire[15:0] flags, input wire[3:0] state, input wire[31:0] limit,
 input wire[7:0] fault,input wire signed[31:0] theta_q16,dtheta_q16,
 // Preserve existing D02 1024x320 storage in this same RAM: two words per row.
 input wire legacy_select,legacy_write,input wire[9:0] legacy_address,legacy_read_index,
 input wire[319:0] legacy_data,output reg[319:0] legacy_read_data,
 output reg legacy_read_valid,legacy_overrun,
 input wire[12:0] read_index, output reg[175:0] read_data,output reg read_valid,
 output reg busy,valid,automatic_session,output reg[13:0] count,
 output reg[12:0] head_count,tail_count,output reg[11:0] tail_base,
 output reg[31:0] capture_id,total_samples,begin_ms,event_ms,last_ms,
 output reg[31:0] first_gap_ms,trigger_ms,output reg[15:0] gaps,trigger_index,
 output reg[7:0] reason,freeze_fault
);
 reg[175:0] memory[0:8191];
 reg[11:0] tail_wr;reg[1:0] settling;
 reg write_q;reg[12:0] write_address;reg[175:0] write_data;
 reg[3:0] previous_state;reg triggered;reg[31:0] trigger_ordinal;
 reg legacy_pending,legacy_read_phase,legacy_phase_q,legacy_read_pending,legacy_low_valid;
 reg[9:0] legacy_saved_address,legacy_address_q,legacy_low_address;
 reg[143:0] legacy_upper;reg[175:0] legacy_low;
 wire range_ok=command_pwm=={{22{command_pwm[9]}},command_pwm[9:0]} &&
  applied_pwm=={{22{applied_pwm[9]}},applied_pwm[9:0]} && limit[31:9]==0 && fault[7:4]==0 &&
  theta_q16=={{11{theta_q16[20]}},theta_q16[20:0]} &&
  dtheta_q16=={{9{dtheta_q16[22]}},dtheta_q16[22:0]};
 wire take=!legacy_select && busy && sample && !stop && !begin_record && range_ok;
 wire[175:0] packed_sample={fault[3:0],dtheta_q16[22:0],theta_q16[20:0],adc_fine_valid,limit[8:0],state,flags,
  applied_pwm[9:0],command_pwm[9:0],encoder,adc_q4,millis};
 wire[12:0] logical_tail=read_index-head_count;
 wire[11:0] tail_address=tail_base+logical_tail[11:0];
 wire[12:0] compact_address=read_index<head_count?read_index:{1'b1,tail_address};
 wire[12:0] read_address=legacy_select?{2'd0,legacy_read_index,legacy_read_phase}:compact_address;
 wire legacy_take=legacy_select && legacy_write && !legacy_pending && !begin_record;
 wire auto_capture=!triggered && state==2 && fault==0 &&
  (previous_state==1 || previous_state==6 || previous_state==7);
 // A registered write and synchronous read infer one simple dual-port RAM.
 always @(posedge clk) begin
  read_data<=memory[read_address];
  read_valid<=rst_n && !legacy_select && !begin_record && !stop && valid && {1'b0,read_index}<count;
  if(rst_n && write_q)memory[write_address]<=write_data;
  write_q<=rst_n && (take || (legacy_select && !begin_record && (legacy_take || legacy_pending)));
  write_address<=legacy_select?(legacy_pending?{2'd0,legacy_saved_address,1'b1}:{2'd0,legacy_address,1'b0}):
   (head_count<4096?head_count:{1'b1,tail_wr});
  write_data<=legacy_select?(legacy_pending?{32'd0,legacy_upper}:legacy_data[175:0]):packed_sample;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   legacy_pending<=0;legacy_saved_address<=0;legacy_upper<=0;legacy_overrun<=0;
   legacy_read_phase<=0;legacy_phase_q<=0;legacy_read_pending<=0;legacy_low_valid<=0;
   legacy_address_q<=0;legacy_low_address<=0;legacy_low<=0;legacy_read_data<=0;legacy_read_valid<=0;
  end else begin
   legacy_read_valid<=0;
   if(!legacy_select || begin_record)begin
    legacy_pending<=0;legacy_overrun<=0;legacy_read_phase<=0;legacy_read_pending<=0;legacy_low_valid<=0;
   end else begin
    legacy_pending<=legacy_take;
    if(legacy_take)begin legacy_saved_address<=legacy_address;legacy_upper<=legacy_data[319:176];end
    if(legacy_write && legacy_pending)legacy_overrun<=1;
    legacy_read_phase<=!legacy_read_phase;legacy_phase_q<=legacy_read_phase;
    legacy_read_pending<=1;legacy_address_q<=legacy_read_index;
    if(legacy_read_pending)begin
     if(!legacy_phase_q)begin legacy_low<=read_data;legacy_low_address<=legacy_address_q;legacy_low_valid<=1;end
     else if(legacy_low_valid && legacy_low_address==legacy_address_q && legacy_address_q==legacy_read_index)begin
      legacy_read_data<={read_data[143:0],legacy_low};legacy_read_valid<=1;
     end
    end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;valid<=0;automatic_session<=0;count<=0;head_count<=0;tail_count<=0;
   tail_base<=0;tail_wr<=0;settling<=0;capture_id<=0;total_samples<=0;
   begin_ms<=0;event_ms<=0;last_ms<=0;first_gap_ms<=0;gaps<=0;
   reason<=0;freeze_fault<=0;previous_state<=0;triggered<=0;
   trigger_ms<=0;trigger_ordinal<=0;trigger_index<=16'hffff;
  end else if(legacy_select && !begin_record)begin
   busy<=0;valid<=0;count<=0;settling<=0;
  end else if(stop && busy)begin
   // A coincident STOP wins over begin/sample/fault. Pending earlier write commits.
   busy<=0;valid<=0;settling<=2;event_ms<=millis;reason<=1;freeze_fault<=fault;
  end else if(begin_record && !stop)begin
   busy<=1;valid<=0;automatic_session<=automatic_start;count<=0;
   head_count<=0;tail_count<=0;tail_wr<=0;tail_base<=0;settling<=0;
   capture_id<=capture_id+1'b1;total_samples<=0;begin_ms<=millis;
   event_ms<=0;last_ms<=millis;first_gap_ms<=0;gaps<=0;
   reason<=0;freeze_fault<=0;previous_state<=0;triggered<=0;
   trigger_ms<=0;trigger_ordinal<=0;trigger_index<=16'hffff;
  end else if(settling!=0)begin
   settling<=settling-1'b1;
   if(settling==1)begin
    count<={1'b0,head_count}+{1'b0,tail_count};
    valid<=head_count!=0 || tail_count!=0;
    tail_base<=tail_count==4096?tail_wr:12'd0;
    if(!triggered)trigger_index<=16'hffff;
    else if(trigger_ordinal<{19'd0,head_count})trigger_index<=trigger_ordinal[15:0];
    else if(trigger_ordinal>=total_samples-{19'd0,tail_count})
     trigger_index<={3'd0,head_count}+trigger_ordinal[15:0]-(total_samples[15:0]-{3'd0,tail_count});
    else trigger_index<=16'hffff; // Trigger exists but has aged out of retained tail.
   end
  end else if(busy && sample)begin
   if(!range_ok)begin
    busy<=0;valid<=0;settling<=2;event_ms<=millis;reason<=8;freeze_fault<=fault;
   end else begin
    if(total_samples!=0 && millis!=last_ms+32'd1)begin
     if(gaps==0)first_gap_ms<=millis;
     if(gaps!=16'hffff)gaps<=gaps+1'b1;
    end
    last_ms<=millis;total_samples<=total_samples+1'b1;previous_state<=state;
    if(head_count<4096)head_count<=head_count+1'b1;
    else begin tail_wr<=tail_wr+1'b1;if(tail_count<4096)tail_count<=tail_count+1'b1;end
    if(auto_capture)begin triggered<=1;trigger_ordinal<=total_samples;trigger_ms<=millis;end
    if(fault!=0 || total_samples==32'hfffffffe)begin
     busy<=0;valid<=0;settling<=2;event_ms<=millis;
     reason<=fault!=0?8'd3:8'd9;freeze_fault<=fault;
    end
   end
  end
 end
endmodule
`default_nettype wire

`default_nettype none
// Payload before the existing UART logger adds CRC16 bytes 62..63.
// New packet types 7/8 are deliberately not old type3/352-bit records.
module compact_frames(
 input wire[31:0] build,capture_id,total_samples,begin_ms,event_ms,last_ms,first_gap_ms,trigger_ms,
 input wire[13:0] count,input wire[12:0] head_count,tail_count,
 input wire[11:0] tail_base,input wire[15:0] gaps,trigger_index,
 input wire[7:0] reason,freeze_fault,input wire busy,valid,automatic_session,
 input wire[12:0] first_index,input wire[1:0] row_count,
 input wire[175:0] row0,row1,
 output reg[495:0] info_payload,data_payload
);
 wire[7:0] status={4'd0,(gaps!=0),automatic_session,valid,busy};
 always @*begin
  info_payload=0;info_payload[31:0]=32'h40075aa5;
  info_payload[63:32]=build;info_payload[95:64]=capture_id;
  info_payload[111:96]=16'hc831;info_payload[119:112]=1;info_payload[127:120]=22;
  info_payload[143:128]=16'h0060;info_payload[159:144]=8192;
  info_payload[175:160]={3'd0,head_count};info_payload[191:176]={3'd0,tail_count};
  info_payload[207:192]={4'd0,tail_base};info_payload[223:208]={2'd0,count};
  info_payload[255:224]=total_samples;info_payload[287:256]=begin_ms;
  info_payload[319:288]=event_ms;info_payload[351:320]=last_ms;
  info_payload[383:352]=first_gap_ms;info_payload[399:384]=gaps;
  info_payload[415:400]=trigger_index;info_payload[447:416]=trigger_ms;
  info_payload[455:448]=reason;info_payload[463:456]=status;
  info_payload[471:464]=freeze_fault;
  data_payload=0;data_payload[31:0]=32'h40085aa5;
  data_payload[63:32]=capture_id;data_payload[79:64]={3'd0,first_index};
  data_payload[87:80]={6'd0,row_count};data_payload[95:88]=1;
  data_payload[271:96]=row0;data_payload[447:272]=row1;
  data_payload[479:448]=build;data_payload[495:480]=16'hc831;
 end
endmodule
`default_nettype wire
