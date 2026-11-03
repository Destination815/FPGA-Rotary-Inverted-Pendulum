// Shared by controller and its component test. Q18 theta/omega, no gain.
// Force only while not already returning fast enough to halve |theta|/s.
// Widen before absolute value and double, including INT32_MIN boundaries.
function r29_returning_gate;
 input signed [31:0] theta_q18,omega_q18;
 reg [32:0] theta_abs,omega_abs;
 reg strict_inward;
 begin
  theta_abs=theta_q18[31]?-{theta_q18[31],theta_q18}:{1'b0,theta_q18};
  omega_abs=omega_q18[31]?-{omega_q18[31],omega_q18}:{1'b0,omega_q18};
  strict_inward=((theta_q18>0)&&(omega_q18<0))||
                ((theta_q18<0)&&(omega_q18>0));
  r29_returning_gate=(theta_abs!=0)&&
                    (!strict_inward||({theta_abs,1'b0}>{1'b0,omega_abs}));
 end
endfunction
