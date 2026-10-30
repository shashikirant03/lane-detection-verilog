// ============================================================================
// sdp_ram.v : simple dual-port RAM (1 write port, 1 read port, 1 clock)
//             registered read address -> read data valid 1 clock later.
//             Quartus infers M9K blocks from this template.
// ============================================================================
module sdp_ram #(
    parameter DW    = 8,
    parameter AW    = 10,
    parameter DEPTH = 1024
)(
    input                 clk,
    input                 we,
    input      [AW-1:0]   waddr,
    input      [DW-1:0]   wdata,
    input      [AW-1:0]   raddr,
    output reg [DW-1:0]   rdata
);
    (* ramstyle = "M9K, no_rw_check" *) reg [DW-1:0] mem [0:DEPTH-1];

    // simulation only: start from zero instead of X (M9K blocks power up to 0)
    // synthesis translate_off
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DW{1'b0}};
        rdata = {DW{1'b0}};
    end
    // synthesis translate_on

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule
