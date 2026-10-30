# 50 MHz board oscillator
create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]
derive_pll_clocks
derive_clock_uncertainty
# asynchronous / slow board I/O
set_false_path -from [get_ports {KEY[*]}]
set_false_path -to   [get_ports {LEDR[*] LEDG[*]}]
set_false_path -to   [get_ports {VGA_*}]
