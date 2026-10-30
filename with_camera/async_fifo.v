// ============================================================================
// async_fifo.v  -  small FIFO between two clock domains
//
// The camera runs on its own clock (OV7670 PCLK), the lane detection and the
// VGA run on the 25 MHz FPGA clock.  The 320 x 240 pixels (and an end-of-frame
// marker) are passed through this FIFO, so ONLY this module crosses clock
// domains.
//
// Classic design: binary read / write pointers + Gray-coded copies.  A Gray
// code changes only one bit per step, so a pointer can be safely passed to
// the other clock through two flip-flops.
//   empty : read  pointer == synchronised write pointer
//   full  : write pointer == synchronised read pointer with the top 2 bits
//           inverted (the writer is one lap ahead)
// The read side outputs one word per clock while the FIFO is not empty
// (out_valid = 1 for that clock).
// The camera writes at most one pixel every 4 PCLK, the reader takes one per
// clock, so the FIFO never gets full.
// ============================================================================
module async_fifo #(
    parameter DW = 42,
    parameter AW = 4                 // 2^AW words
)(
    // write side (camera clock)
    input               wclk,
    input               wrst,
    input               we,
    input      [DW-1:0] wdata,
    output              full,
    // read side (25 MHz)
    input               rclk,
    input               rrst,
    output reg          out_valid,
    output reg [DW-1:0] out_data
);
    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]   wbin, wgray;                     // write pointer (binary, Gray)
    reg [AW:0]   rbin, rgray;                     // read pointer  (binary, Gray)

    // ---------------- write side
    reg  [AW:0] rgray_w1, rgray_w2;               // read pointer, synchronised
    wire [AW:0] wbin_n  = wbin + 1'b1;
    wire [AW:0] wgray_n = (wbin_n >> 1) ^ wbin_n;  // binary -> Gray

    assign full = (wgray == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});

    always @(posedge wclk) begin
        if (wrst) begin
            wbin <= 0;  wgray <= 0;
        end else if (we && !full) begin
            mem[wbin[AW-1:0]] <= wdata;
            wbin  <= wbin_n;
            wgray <= wgray_n;
        end
    end

    always @(posedge wclk) begin
        if (wrst) begin rgray_w1 <= 0;  rgray_w2 <= 0; end
        else      begin rgray_w1 <= rgray;  rgray_w2 <= rgray_w1; end
    end

    // ---------------- read side
    reg  [AW:0] wgray_r1, wgray_r2;               // write pointer, synchronised
    wire [AW:0] rbin_n  = rbin + 1'b1;
    wire [AW:0] rgray_n = (rbin_n >> 1) ^ rbin_n;
    wire        empty   = (rgray == wgray_r2);

    always @(posedge rclk) begin
        if (rrst) begin wgray_r1 <= 0;  wgray_r2 <= 0; end
        else      begin wgray_r1 <= wgray;  wgray_r2 <= wgray_r1; end
    end

    always @(posedge rclk) begin
        if (rrst) begin
            rbin <= 0;  rgray <= 0;  out_valid <= 1'b0;
        end else begin
            out_valid <= !empty;
            if (!empty) begin
                out_data <= mem[rbin[AW-1:0]];
                rbin  <= rbin_n;
                rgray <= rgray_n;
            end
        end
    end
endmodule
