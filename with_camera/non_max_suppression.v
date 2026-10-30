// ============================================================================
// non_max_suppression.v  -  Canny step 4: NON-MAXIMUM SUPPRESSION
//                           paper Sec. IV-A-4, Algorithm 3 lines 13-22
//
// "Each image pixel on the edge is compared with its neighbouring pixels.
//  If a pixel is brighter than its neighbouring pixels, its intensity is
//  maintained. Otherwise, it is set to zero."
//
//   for every pixel:  n1, n2 = the two neighbours in the gradient direction
//       if (G < n1) or (G < n2)  ->  Pix_nonMax = 0
//       else                     ->  Pix_nonMax = G
// Input word = {dir, |G|} from sobel_operator.   Latency: 2 + 1 clocks.
// ============================================================================
module non_max_suppression #(
    parameter IMG_W = 320,
    parameter X0    = 3,
    parameter Y0    = 3
)(
    input              clk,
    input              rst,
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_mag,
    input      [1:0]   in_dir,
    output reg         out_valid,
    output reg [8:0]   out_x,
    output reg [7:0]   out_y,
    output reg [7:0]   out_mag
);
    wire           wv;
    wire [8:0]     wx;
    wire [7:0]     wy;
    wire [9*10-1:0] win;

    sliding_window #(.DW(10), .K(3), .IMG_W(IMG_W), .X0(X0), .Y0(Y0)) u_win (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_pix({in_dir, in_mag}),
        .out_valid(wv), .out_x(wx), .out_y(wy), .win(win)
    );

    // gradient magnitudes of the 3x3 neighbourhood
    wire [7:0] g00 = win[0*10 +: 8], g01 = win[1*10 +: 8], g02 = win[2*10 +: 8];
    wire [7:0] g10 = win[3*10 +: 8], g11 = win[4*10 +: 8], g12 = win[5*10 +: 8];
    wire [7:0] g20 = win[6*10 +: 8], g21 = win[7*10 +: 8], g22 = win[8*10 +: 8];
    wire [1:0] dir = win[4*10 + 8 +: 2];                 // direction of the centre

    reg [7:0] n1, n2;
    always @(*) begin
        case (dir)
            2'd0:    begin n1 = g10; n2 = g12; end       // 0 deg  : left, right
            2'd1:    begin n1 = g00; n2 = g22; end       // 45 deg : up-left, down-right
            2'd2:    begin n1 = g01; n2 = g21; end       // 90 deg : up, down
            default: begin n1 = g02; n2 = g20; end       // 135 deg: up-right, down-left
        endcase
    end

    always @(posedge clk) begin
        if ((g11 < n1) || (g11 < n2))
            out_mag <= 8'd0;                             // not a maximum -> 0
        else
            out_mag <= g11;                              // local maximum -> keep
        out_x <= wx;
        out_y <= wy;
        if (rst) out_valid <= 1'b0; else out_valid <= wv;
    end
endmodule
