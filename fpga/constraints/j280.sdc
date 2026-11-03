# T02 COMPILE ONLY. Check exception coverage and detailed path arithmetic in Gowin.
# ADC timing assumptions/derivation: analysis/t02_adc_20261002/README.md.
create_clock -name clk50 -period 20.000 [get_ports {clk_50m}]
create_generated_clock -name adc25 -source [get_ports {clk_50m}] -divide_by 20 [get_ports {adc_clk}]
# 3PA1030 Fig.22 (p20) launches output data on FALLING edges. p8 gives
# T_OD=25ns TYPICAL only, with neither min nor max. 0..80ns is therefore an
# explicit engineering interface budget, NOT a vendor-guaranteed specification.
# It covers external data-valid delay + board clock/data flight/skew; generated
# clock propagation and FPGA input routing must also appear in the STA report.
set_input_delay -clock adc25 -clock_fall -max 80.000 [get_ports {adc_data_in[*]}]
set_input_delay -clock adc25 -clock_fall -min 0.000 [get_ports {adc_data_in[*]}]
# Enable-aware absolute windows (SUG940 5.4.2, printed pp68-69):
# ADC falling edge t=200ns -> phase15 input FF capture t=320ns -> next falling
# edge t=600ns. Setup has 120ns total (80ns external leaves about 40ns internal,
# before setup/skew). Hold may allow the next data change only 280ns AFTER
# capture. -280 is the permitted window in the min-delay check, not negative
# physical routing. The old fast-clock setup16/hold15 did not model this CE.
# These exceptions apply ONLY from ADC ports to the 10 dedicated input FFs.
# Input FF -> sum/average remains an ordinary 20ns synchronous timing path.
# Acceptance requires non-empty destinations AND detailed reports confirming
# input-delay inclusion, generated-clock latency and these setup/hold windows.
set_max_delay -from [get_ports {adc_data_in[*]}] -to [get_regs {*adc_inst*sample_data*}] 120.000
set_min_delay -from [get_ports {adc_data_in[*]}] -to [get_regs {*adc_inst*sample_data*}] -280.000
# These inputs ONLY feed the first stage of explicit synchronizers in this RTL.
# No false path is applied to the second synchronizer stage or the control datapath.
set_false_path -from [get_ports {rst_n uart_rx key_sw[*] enc1_a enc1_b enc2_a enc2_b}]
set_output_delay -clock clk50 -max 5.000 [get_ports {AN1 AN2 PWMA BN1 BN2 PWMB uart_tx led_n[*] adc_oe_n}]
set_output_delay -clock clk50 -min 0.000 [get_ports {AN1 AN2 PWMA BN1 BN2 PWMB uart_tx led_n[*] adc_oe_n}]
# Full STA output and exception coverage (Gowin SUG940, sections 5.5.1/5.5.2).
report_timing -setup -max_paths 200 -max_common_paths 1
report_timing -hold -max_paths 200 -max_common_paths 1
report_timing -recovery -max_paths 50 -max_common_paths 1
report_timing -removal -max_paths 50 -max_common_paths 1
report_exceptions -setup -max_paths 100 -max_common_paths 1
report_exceptions -hold -max_paths 100 -max_common_paths 1
report_exceptions -recovery -max_paths 50 -max_common_paths 1
report_exceptions -removal -max_paths 50 -max_common_paths 1
# Include ADC paths even if the global worst paths are dominated by another module.
report_timing -setup -from [get_ports {adc_data_in[*]}] -to [get_regs {*adc_inst*sample_data*}] -max_paths 20 -max_common_paths 1
report_timing -hold -from [get_ports {adc_data_in[*]}] -to [get_regs {*adc_inst*sample_data*}] -max_paths 20 -max_common_paths 1
