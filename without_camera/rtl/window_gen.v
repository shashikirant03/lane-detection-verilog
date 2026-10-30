// ============================================================================
// window_gen.v : K x K sliding window generator for a raster pixel stream
//
//  * (K-1) line memories, packed into ONE RAM word per column
//    (Fig. 11 of the paper: "line mem" + "pixel mem" registers)
//  * accepts one pixel per clock when in_valid = 1
//  * out_valid is asserted 2 clocks after each accepted pixel; win then
//    holds the K x K neighbourhood whose NEWEST pixel is the one accepted.
//
//  win layout : element (r,c) at win[(r*K+c)*DW +: DW]
//               r = 0 top (oldest row) .. K-1 bottom (newest row)
//               c = 0 left (oldest col) .. K-1 right (newest col)
//  The window centre (r=c=(K-1)/2) is therefore the pixel (K-1)/2 rows and
//  (K-1)/2 columns BEHIND the newest input.  Near the left/right image
//  border the window wraps into the previous/next row; those results are
//  removed later by the border mask.
// ============================================================================
module window_gen #(
    parameter DW = 8,
    parameter K  = 3,
    parameter W  = 320,
    parameter AW = 9
)(
    input                     clk,
    input                     rst,
    input                     in_valid,
    input      [DW-1:0]       in_data,
    output reg                out_valid,
    output reg [K*K*DW-1:0]   win
);
    localparam LW = (K-1)*DW;

    reg  [AW-1:0] col;
    reg           d1_valid;
    reg  [DW-1:0] d1_data;
    reg  [AW-1:0] d1_col;
    wire [LW-1:0] lb_q;
    wire [LW-1:0] lb_new;

    // shift the column up by one row: drop the oldest, append the new pixel
    generate
        if (K > 2) begin : g_sh
            assign lb_new = {d1_data, lb_q[LW-1:DW]};
        end else begin : g_nosh
            assign lb_new = d1_data;
        end
    endgenerate

    sdp_ram #(.DW(LW), .AW(AW), .DEPTH(W)) u_lb (
        .clk(clk), .we(d1_valid), .waddr(d1_col), .wdata(lb_new),
        .raddr(col), .rdata(lb_q));

    always @(posedge clk) begin
        if (rst) begin
            col      <= {AW{1'b0}};
            d1_valid <= 1'b0;
        end else begin
            d1_valid <= in_valid;
            if (in_valid) col <= (col == W-1) ? {AW{1'b0}} : col + 1'b1;
        end
        d1_data <= in_data;
        d1_col  <= col;
    end

    integer r, c;
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= d1_valid;
        if (d1_valid) begin
            for (r = 0; r < K; r = r + 1)
                for (c = 0; c < K-1; c = c + 1)
                    win[(r*K+c)*DW +: DW] <= win[(r*K+c+1)*DW +: DW];
            for (r = 0; r < K-1; r = r + 1)
                win[(r*K+K-1)*DW +: DW] <= lb_q[r*DW +: DW];
            win[((K-1)*K+K-1)*DW +: DW] <= d1_data;
        end
    end
endmodule
