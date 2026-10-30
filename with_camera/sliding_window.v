// ============================================================================
// sliding_window.v   -  paper Fig. 11 ("Line mem" + "Pixel mem")
//
// Builds a K x K window of pixels from a raster pixel stream:
//   * (K-1) LINE MEMORIES store the previous image rows,
//   * K x K PIXEL MEMORIES (registers) hold the window (left/centre/right...).
// Every new pixel shifts the window one column to the right, so the next
// stage (Gaussian / Sobel / NMS) gets a complete window every clock.
//
//   X0, Y0 : coordinate of the first pixel that arrives from the previous stage
//   out_x, out_y : coordinate of the CENTRE pixel of the window
//   win[((r*K)+c)*DW +: DW] : r = row (0 = top), c = column (0 = left)
// The border (K-1)/2 pixels, where the window would leave the image, are not
// produced.   Latency: 2 clocks.
// ============================================================================
module sliding_window #(
    parameter DW    = 8,
    parameter K     = 3,
    parameter IMG_W = 320,
    parameter X0    = 0,
    parameter Y0    = 0
)(
    input                    clk,
    input                    rst,
    input                    in_valid,
    input      [8:0]         in_x,
    input      [7:0]         in_y,
    input      [DW-1:0]      in_pix,
    output reg               out_valid,
    output reg [8:0]         out_x,
    output reg [7:0]         out_y,
    output     [K*K*DW-1:0]  win
);
    // window radius (K-1)/2 : 2 for the 5x5 window, 1 for the 3x3 windows
    localparam [8:0] RX = (K == 5) ? 9'd2 : 9'd1;
    localparam [7:0] RY = (K == 5) ? 8'd2 : 8'd1;

    // ---- register the input (line memories are read meanwhile) ----
    reg          v1;
    reg [8:0]    x1;
    reg [7:0]    y1;
    reg [DW-1:0] p1;

    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else     v1 <= in_valid;
        x1 <= in_x;
        y1 <= in_y;
        p1 <= in_pix;
    end

    // ---- line memories: line g holds image row (y-1-g) ----
    wire [(K-1)*DW-1:0] line_q;

    genvar g;
    generate
        for (g = 0; g < K-1; g = g + 1) begin : LINE
            wire [DW-1:0] wdata;
            if (g == 0) begin : FIRST
                assign wdata = p1;                        // current row goes in
            end else begin : NEXT
                assign wdata = line_q[(g-1)*DW +: DW];    // row moves one line down
            end
            dual_port_ram #(.DW(DW), .AW(9), .DEPTH(IMG_W)) u_line (
                .wclk(clk), .we(v1), .waddr(x1), .wdata(wdata),
                .rclk(clk), .raddr(in_x), .rdata(line_q[g*DW +: DW])
            );
        end
    endgenerate

    // ---- pixel memories (the window) ----
    reg [DW-1:0] w [0:K*K-1];
    integer r, c;

    always @(posedge clk) begin
        if (v1) begin
            for (r = 0; r < K; r = r + 1)
                for (c = 0; c < K-1; c = c + 1)
                    w[r*K + c] <= w[r*K + c + 1];                 // shift left
            for (r = 0; r < K-1; r = r + 1)
                w[r*K + K-1] <= line_q[(K-2-r)*DW +: DW];         // old rows
            w[(K-1)*K + K-1] <= p1;                               // new pixel
        end
    end

    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= v1 && (x1 >= X0 + K - 1) && (y1 >= Y0 + K - 1);
        out_x <= x1 - RX;
        out_y <= y1 - RY;
    end

    genvar i;
    generate
        for (i = 0; i < K*K; i = i + 1) begin : WIN
            assign win[i*DW +: DW] = w[i];
        end
    endgenerate
endmodule
