// ============================================================================
// dual_port_ram.v
// Simple dual-port RAM: one write port, one read port (read data registered).
// Written in the standard Quartus template so it is placed in M9K block RAM.
//   wclk / rclk may be the same clock or two different clocks.
// ============================================================================
module dual_port_ram #(
    parameter DW    = 8,        // data width
    parameter AW    = 9,        // address width
    parameter DEPTH = 320       // number of words
)(
    input               wclk,
    input               we,
    input      [AW-1:0] waddr,
    input      [DW-1:0] wdata,
    input               rclk,
    input      [AW-1:0] raddr,
    output reg [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge wclk)
        if (we)
            mem[waddr] <= wdata;

    always @(posedge rclk)
        rdata <= mem[raddr];
endmodule
