# Open j280_draft.gprj first. In Gowin's Tcl console:
# source C:/fpga/j280/fpga/configure_project.tcl
set j280_dir [file dirname [file normalize [info script]]]
set_option -top_module j280_top
set_option -include_path [file join $j280_dir src]
set_option -verilog_std v2001
# Match the real 50 MHz input clock used by the PnR SDC; Arora defaults to 100 MHz.
# Gowin SUG1220: -global_freq is specified in MHz.
set_option -global_freq 50
# The official J280 example uses T20 for SW4, shared with SSPI.
# Keep its physical pin and enable the same regular-IO mode as the example.
set_option -use_sspi_as_gpio 1

# R41T2: GUI and the one-click entry use the same explicit physical options.
set_option -place_option 1
set_option -route_option 0
set_option -timing_driven 1
set_option -correct_hold_violation 1
set_option -cst_warn_to_error 1
