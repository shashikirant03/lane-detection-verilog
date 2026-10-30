# =============================================================================
#  lane_detection.tcl  -  Quartus project + pin assignments
#                         (DE2-115, Cyclone IV E EP4CE115F29C7, OV7670 camera)
#
#  HOW TO USE
#   a) Command prompt in this folder:     quartus_sh -t lane_detection.tcl
#      then open lane_detection.qpf in Quartus and Start Compilation.
#   b) Or in Quartus: View -> Utility Windows -> Tcl Console,
#      cd to this folder and type:        source lane_detection.tcl
#   (If a project is already open, it just applies the settings and pins.)
#
#  Sections: 1 project + files, 2 timing, 3 board I/O, 4 VGA, 5 camera
# =============================================================================

# ------------------------------------------------------------------ 1. project
set project lane_detection
set top     lane_detection_top

set opened_here 0
if {![is_project_open]} {
    if {[project_exists $project]} {
        project_open $project
    } else {
        project_new $project
    }
    set opened_here 1
}

set_global_assignment -name FAMILY "Cyclone IV E"
set_global_assignment -name DEVICE EP4CE115F29C7
set_global_assignment -name TOP_LEVEL_ENTITY $top
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "3.3-V LVTTL"
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED"
set_global_assignment -name CYCLONEII_RESERVE_NCEO_AFTER_CONFIGURATION "USE AS REGULAR IO"
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL

foreach f {
    lane_detection_top.v
    pll_25m.v
    ov7670_config.v
    sccb_master.v
    ov7670_capture.v
    gray_scale.v
    async_fifo.v
    lane_detection.v
    canny_edge_detection.v
    gaussian_smoothing.v
    sobel_operator.v
    square_root.v
    non_max_suppression.v
    hysteresis.v
    sliding_window.v
    hough_transform.v
    cordic_rotator.v
    local_maxima.v
    vga_display.v
    dual_port_ram.v
} {
    set_global_assignment -name VERILOG_FILE $f
}

# ------------------------------------------------------------------ 2. timing
# clocks: 50 MHz board clock, camera PCLK (constrained for up to 50 MHz to
# be safe, normally ~25 MHz), PLL outputs (25 MHz).
# The OV7670 changes D/HREF/VSYNC on the falling edge of PCLK (+0..5 ns),
# they are sampled on the rising edge.
set sdc [open "lane_detection.sdc" w]
puts $sdc {create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]}
puts $sdc {create_clock -name OV_PCLK  -period 20.000 [get_ports {OV_PCLK}]}
puts $sdc {derive_pll_clocks}
puts $sdc {derive_clock_uncertainty}
puts $sdc {set_clock_groups -asynchronous -group [get_clocks {CLOCK_50}] -group [get_clocks {OV_PCLK}] -group [get_clocks {*pll1|clk[0]}] -group [get_clocks {*pll1|clk[1]}]}
puts $sdc {set_input_delay -clock OV_PCLK -max 15.0 [get_ports {OV_D[*] OV_HREF OV_VSYNC}]}
puts $sdc {set_input_delay -clock OV_PCLK -min 10.0 [get_ports {OV_D[*] OV_HREF OV_VSYNC}]}
puts $sdc {set_false_path -from [get_ports {KEY[*] SW[*] OV_SIOD}]}
puts $sdc {set_false_path -to [get_ports {LEDR[*] LEDG[*] HEX* VGA_* OV_XCLK OV_RESET_N OV_PWDN OV_SIOC OV_SIOD}]}
close $sdc
set_global_assignment -name SDC_FILE lane_detection.sdc

# helpers: one pin / a bus (pins listed from bit 0 upwards)
proc set_pin {name location} {
    set_location_assignment PIN_$location -to $name
    set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to $name
}
proc set_bus {name locations} {
    set i 0
    foreach loc $locations {
        set_pin "${name}\[$i\]" $loc
        incr i
    }
}

# ------------------------------------------------------------------ 3. board I/O
set_pin CLOCK_50 Y2
set_bus KEY  {M23 M21}                     ;# KEY0, KEY1 (KEY2, KEY3 not used)
set_bus SW   {AB28 AC28 AC27 AD27 AB27 AC26 AD26 AB26 AC25 AB25 AC24 AB24 AB23 AA24 AA23 AA22 Y24 Y23}
set_bus LEDR {G19 F19 E19 F21 F18 E18 J19 H19 J17 G17 J15 H16 J16 H17 F15 G15 G16 H15}
set_bus LEDG {E21 E22 E25 E24 H21 G20 G22 G21 F17}
# 7-segment displays (not used by the design, kept dark: segment a..g = bit 0..6)
set_bus HEX0 {G18  F22  E17  L26  L25  J22  H22}
set_bus HEX1 {M24  Y22  W21  W22  W25  U23  U24}
set_bus HEX2 {AA25 AA26 Y25  W26  Y26  W27  W28}
set_bus HEX3 {V21  U21  AB20 AA21 AD24 AF23 Y19}
set_bus HEX4 {AB19 AA19 AG21 AH21 AE19 AF19 AE18}
set_bus HEX5 {AD18 AC18 AB18 AH19 AG19 AF18 AH18}
set_bus HEX6 {AA17 AB16 AA16 AB17 AB15 AA15 AC17}
set_bus HEX7 {AD17 AE17 AG17 AH17 AF17 AG18 AA14}

# ------------------------------------------------------------------ 4. VGA (ADV7123 DAC)
set_pin VGA_CLK     A12
set_pin VGA_HS      G13
set_pin VGA_VS      C13
set_pin VGA_BLANK_N F11
set_pin VGA_SYNC_N  C10
#           bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7
set_bus VGA_R  {E12  E11  D10  F12  G10  J12  H8   H10}
set_bus VGA_G  {G8   G11  F8   H12  C8   B8   F10  C9}
set_bus VGA_B  {B10  A10  C11  B11  A11  C12  D11  D12}

# ------------------------------------------------------------------ 5. OV7670 camera on GPIO header JP5
#  Wire the OV7670 module to JP5 with female-female jumper wires:
#
#     OV7670 pin    signal        FPGA pin   GPIO      JP5 header pin
#     ----------    ----------    --------   -------   --------------
#     PCLK          OV_PCLK       AB22       GPIO[0]    1
#     VSYNC         OV_VSYNC      AC15       GPIO[1]    2
#     HREF (HS)     OV_HREF       AB21       GPIO[2]    3
#     D0            OV_D[0]       Y17        GPIO[3]    4
#     D1            OV_D[1]       AC21       GPIO[4]    5
#     D2            OV_D[2]       Y16        GPIO[5]    6
#     D3            OV_D[3]       AD21       GPIO[6]    7
#     D4            OV_D[4]       AE16       GPIO[7]    8
#     D5            OV_D[5]       AD15       GPIO[8]    9
#     D6            OV_D[6]       AE15       GPIO[9]   10
#     (do NOT use)  5 V                                11
#     GND           GND                                12
#     D7            OV_D[7]       AC19       GPIO[10]  13
#     XCLK (MCLK)   OV_XCLK       AF16       GPIO[11]  14
#     SIOC (SCL)    OV_SIOC       AD19       GPIO[12]  15
#     SIOD (SDA)    OV_SIOD       AF15       GPIO[13]  16
#     RESET (RST)   OV_RESET_N    AF24       GPIO[14]  17
#     PWDN          OV_PWDN       AE21       GPIO[15]  18
#     3V3 (VCC)     3.3 V                              29
#
#  The OV7670 is a 3.3 V part: power it from pin 29 (3.3 V), never pin 11 (5 V).
#  SIOD needs a pull-up: the FPGA weak pull-up is switched on below; if the
#  camera does not answer (LEDG4 on) add a 4.7 k resistor from SIOD to 3.3 V.
set_pin  OV_PCLK         AB22    ;# GPIO[0]   1
set_pin  OV_VSYNC        AC15    ;# GPIO[1]   2
set_pin  OV_HREF         AB21    ;# GPIO[2]   3
set_pin  {OV_D[0]}       Y17     ;# GPIO[3]   4
set_pin  {OV_D[1]}       AC21    ;# GPIO[4]   5
set_pin  {OV_D[2]}       Y16     ;# GPIO[5]   6
set_pin  {OV_D[3]}       AD21    ;# GPIO[6]   7
set_pin  {OV_D[4]}       AE16    ;# GPIO[7]   8
set_pin  {OV_D[5]}       AD15    ;# GPIO[8]   9
set_pin  {OV_D[6]}       AE15    ;# GPIO[9]   10
set_pin  {OV_D[7]}       AC19    ;# GPIO[10]  13
set_pin  OV_XCLK         AF16    ;# GPIO[11]  14
set_pin  OV_SIOC         AD19    ;# GPIO[12]  15
set_pin  OV_SIOD         AF15    ;# GPIO[13]  16
set_pin  OV_RESET_N      AF24    ;# GPIO[14]  17
set_pin  OV_PWDN         AE21    ;# GPIO[15]  18
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to OV_SIOD
# camera inputs go straight into the I/O-cell registers (best timing)
set_instance_assignment -name FAST_INPUT_REGISTER ON -to {OV_D[*]}
set_instance_assignment -name FAST_INPUT_REGISTER ON -to OV_HREF
set_instance_assignment -name FAST_INPUT_REGISTER ON -to OV_VSYNC

export_assignments
if {$opened_here} { project_close }
puts "lane_detection.tcl done - open lane_detection.qpf and compile."
