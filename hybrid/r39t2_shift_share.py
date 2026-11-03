"""Exact same-edge two-direction shift sharing; no stored state or new clock."""
PREFIX=''' // R39T2: both mutually exclusive shifts use one right barrel shifter.
 // Bit reversal implements a left logical shift; a separate sign bit supplies
 // arithmetic fill only on SHR. Keep the complete signed48 shift-count operand.
 wire r39t2_shift_left=(operation==6'd32);
 wire [47:0] r39t2_shift_input,r39t2_shift_output;
 wire signed[48:0] r39t2_shift_wide={(!r39t2_shift_left && aa[47]),r39t2_shift_input};
 wire signed[48:0] r39t2_shifted=r39t2_shift_wide >>> (bb>>>25);
 genvar r39t2_bit;
 generate for(r39t2_bit=0;r39t2_bit<48;r39t2_bit=r39t2_bit+1)begin:r39t2_shift_wiring
  assign r39t2_shift_input[r39t2_bit]=r39t2_shift_left?aa[47-r39t2_bit]:aa[r39t2_bit];
  assign r39t2_shift_output[r39t2_bit]=r39t2_shift_left?r39t2_shifted[47-r39t2_bit]:r39t2_shifted[r39t2_bit];
 end endgenerate
'''
def transform(source):
 assert source.count(' reg less_q,equal_q;')==1
 source=source.replace(' reg less_q,equal_q;',PREFIX+' reg less_q,equal_q;')
 for old in ['aa >>> (bb>>>25)','aa <<< (bb>>>25)']:
  assert source.count('result<='+old+';')==1
  source=source.replace('result<='+old+';','result<=r39t2_shift_output;')
 return source
