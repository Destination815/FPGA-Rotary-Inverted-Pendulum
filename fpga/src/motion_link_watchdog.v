`default_nettype none
// Unlimited motion duration, bounded loss of the CRC-checked INFO heartbeat.
// The caller gives expired the same priority and cancellation paths as STOP.
module motion_link_watchdog #(parameter integer TIMEOUT_MS=3000)(
 input wire clk,rst_n,tick,armed,heartbeat,
 output wire expired);
 reg[11:0] age_ms;
 assign expired=armed && age_ms>=TIMEOUT_MS;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)age_ms<=0;
  else if(!armed)age_ms<=0;
  else if(heartbeat)age_ms<=0;
  else if(tick && age_ms<TIMEOUT_MS)age_ms<=age_ms+1'b1;
 end
endmodule
`default_nettype wire
