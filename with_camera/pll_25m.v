// ============================================================================
// pll_25m.v  -  Cyclone IV E PLL:  50 MHz  ->  c0 = 25 MHz (VGA pixel clock)
//                                            c1 = 25 MHz (OV7670 XCLK)
// The Altera "altpll" megafunction is instantiated directly (same settings the
// IP-Catalog "ALTPLL" wizard produces), so no wizard run is needed.
// ============================================================================
module pll_25m (
    input  inclk0,     // 50 MHz
    input  areset,
    output c0,         // 25 MHz VGA
    output c1,         // 25 MHz camera
    output locked
);
    wire [4:0] clk_out;

    altpll altpll_component (
        .inclk  ({1'b0, inclk0}),
        .areset (areset),
        .clk    (clk_out),
        .locked (locked)
    );
    defparam
        altpll_component.bandwidth_type          = "AUTO",
        altpll_component.clk0_divide_by          = 2,
        altpll_component.clk0_duty_cycle         = 50,
        altpll_component.clk0_multiply_by        = 1,
        altpll_component.clk0_phase_shift        = "0",
        altpll_component.clk1_divide_by          = 2,
        altpll_component.clk1_duty_cycle         = 50,
        altpll_component.clk1_multiply_by        = 1,
        altpll_component.clk1_phase_shift        = "0",
        altpll_component.compensate_clock        = "CLK0",
        altpll_component.inclk0_input_frequency  = 20000,
        altpll_component.intended_device_family  = "Cyclone IV E",
        altpll_component.lpm_hint                = "CBX_MODULE_PREFIX=pll_25m",
        altpll_component.lpm_type                = "altpll",
        altpll_component.operation_mode          = "NORMAL",
        altpll_component.pll_type                = "AUTO",
        altpll_component.port_activeclock        = "PORT_UNUSED",
        altpll_component.port_areset             = "PORT_USED",
        altpll_component.port_clkbad0            = "PORT_UNUSED",
        altpll_component.port_clkbad1            = "PORT_UNUSED",
        altpll_component.port_clkloss            = "PORT_UNUSED",
        altpll_component.port_clkswitch          = "PORT_UNUSED",
        altpll_component.port_configupdate       = "PORT_UNUSED",
        altpll_component.port_fbin               = "PORT_UNUSED",
        altpll_component.port_inclk0             = "PORT_USED",
        altpll_component.port_inclk1             = "PORT_UNUSED",
        altpll_component.port_locked             = "PORT_USED",
        altpll_component.port_pfdena             = "PORT_UNUSED",
        altpll_component.port_phasecounterselect = "PORT_UNUSED",
        altpll_component.port_phasedone          = "PORT_UNUSED",
        altpll_component.port_phasestep          = "PORT_UNUSED",
        altpll_component.port_phaseupdown        = "PORT_UNUSED",
        altpll_component.port_pllena             = "PORT_UNUSED",
        altpll_component.port_scanaclr           = "PORT_UNUSED",
        altpll_component.port_scanclk            = "PORT_UNUSED",
        altpll_component.port_scanclkena         = "PORT_UNUSED",
        altpll_component.port_scandata           = "PORT_UNUSED",
        altpll_component.port_scandataout        = "PORT_UNUSED",
        altpll_component.port_scandone           = "PORT_UNUSED",
        altpll_component.port_scanread           = "PORT_UNUSED",
        altpll_component.port_scanwrite          = "PORT_UNUSED",
        altpll_component.port_clk0               = "PORT_USED",
        altpll_component.port_clk1               = "PORT_USED",
        altpll_component.port_clk2               = "PORT_UNUSED",
        altpll_component.port_clk3               = "PORT_UNUSED",
        altpll_component.port_clk4               = "PORT_UNUSED",
        altpll_component.port_clk5               = "PORT_UNUSED",
        altpll_component.port_clkena0            = "PORT_UNUSED",
        altpll_component.port_clkena1            = "PORT_UNUSED",
        altpll_component.port_clkena2            = "PORT_UNUSED",
        altpll_component.port_clkena3            = "PORT_UNUSED",
        altpll_component.port_clkena4            = "PORT_UNUSED",
        altpll_component.port_clkena5            = "PORT_UNUSED",
        altpll_component.port_extclk0            = "PORT_UNUSED",
        altpll_component.port_extclk1            = "PORT_UNUSED",
        altpll_component.port_extclk2            = "PORT_UNUSED",
        altpll_component.port_extclk3            = "PORT_UNUSED",
        altpll_component.self_reset_on_loss_lock = "OFF",
        altpll_component.width_clock             = 5;

    assign c0 = clk_out[0];
    assign c1 = clk_out[1];
endmodule
