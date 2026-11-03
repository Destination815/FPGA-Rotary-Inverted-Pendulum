`default_nettype none
module compact_test_top(
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
 output reg[7:0] reason,freeze_fault,
 input wire[12:0] first_index,input wire[1:0] row_count,input wire[175:0] row0,row1,output wire[495:0] info_payload,data_payload
);
 passive_capture recorder(.*);
 compact_frames frames(.build(32'h4a070233),.capture_id(capture_id|32'h80000000),.total_samples(total_samples),.begin_ms(begin_ms),.event_ms(event_ms),.last_ms(last_ms),.first_gap_ms(first_gap_ms),.trigger_ms(trigger_ms),.count(count),.head_count(head_count),.tail_count(tail_count),.tail_base(tail_base),.gaps(gaps),.trigger_index(trigger_index),.reason(reason),.freeze_fault(freeze_fault),.busy(busy),.valid(valid),.automatic_session(automatic_session),.first_index(first_index),.row_count(row_count),.row0(row0),.row1(row1),.info_payload(info_payload),.data_payload(data_payload));
endmodule
