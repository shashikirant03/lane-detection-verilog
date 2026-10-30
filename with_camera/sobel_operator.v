// ============================================================================
// sobel_operator.v  -  Canny step 3: SOBEL OPERATOR
//                      paper Sec. IV-A-3, eq. (3)-(6), Fig. 11, Fig. 12,
//                      Algorithm 3 lines 6-12
//
//          [-1 0 1]                  [-1 -2 -1]
//   Gx  =  [-2 0 2] * B       Gy  =  [ 0  0  0] * B                 eq. (3)
//          [-1 0 1]                  [ 1  2  1]
//
//   |G|   = sqrt( Gx^2 + Gy^2 )                                      eq. (5)
//   theta = atan( |Gy| / |Gx| )                                      eq. (6)
//
// Fig. 12 datapath: 6 weighted pixels are added for each direction, the
// results are squared, added, LIMITED (so that |G| fits 0..255 -> limit
// 255^2 = 65025) and square-rooted.
//
// theta is needed only to choose the two neighbours in non-maximum
// suppression, so it is quantised to 4 directions ("pre-determined theta
// value", Algorithm 3 line 22) without a divider:
//   tan(22.5 deg) = 0.414 ~ 53/128
//   dir 0 : |theta| < 22.5 deg   (|Gy|*128 <= |Gx|*53)  -> left / right
//   dir 2 : |theta| > 67.5 deg   (|Gx|*128 <= |Gy|*53)  -> up / down
//   dir 1 : ~ +45 deg (Gx, Gy same sign)                  -> up-left / down-right
//   dir 3 : ~ -45 deg (Gx, Gy opposite sign)              -> up-right / down-left
// (image y axis points down)
// Latency: 2 (window) + 3 + 8 (sqrt) clocks.
// ============================================================================
module sobel_operator #(
    parameter IMG_W = 320,
    parameter X0    = 2,
    parameter Y0    = 2
)(
    input              clk,
    input              rst,
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_pix,
    output             out_valid,
    output     [8:0]   out_x,
    output     [7:0]   out_y,
    output     [7:0]   out_mag,
    output     [1:0]   out_dir
);
    wire          wv;
    wire [8:0]    wx;
    wire [7:0]    wy;
    wire [9*8-1:0] win;

    sliding_window #(.DW(8), .K(3), .IMG_W(IMG_W), .X0(X0), .Y0(Y0)) u_win (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_pix(in_pix),
        .out_valid(wv), .out_x(wx), .out_y(wy), .win(win)
    );

    // b<row><column> of the 3x3 window (top, centre, bottom / left, centre, right)
    wire [7:0] b00 = win[0*8 +: 8], b01 = win[1*8 +: 8], b02 = win[2*8 +: 8];
    wire [7:0] b10 = win[3*8 +: 8],                      b12 = win[5*8 +: 8];
    wire [7:0] b20 = win[6*8 +: 8], b21 = win[7*8 +: 8], b22 = win[8*8 +: 8];

    // ---- stage 1 : Gx, Gy  (-1020 .. +1020) ----
    wire [10:0] gx_pos = b02 + {b12, 1'b0} + b22;      //  1*b02 + 2*b12 + 1*b22
    wire [10:0] gx_neg = b00 + {b10, 1'b0} + b20;      //  1*b00 + 2*b10 + 1*b20
    wire [10:0] gy_pos = b20 + {b21, 1'b0} + b22;
    wire [10:0] gy_neg = b00 + {b01, 1'b0} + b02;

    reg signed [10:0] gx, gy;
    reg               v1;  reg [8:0] x1;  reg [7:0] y1;

    always @(posedge clk) begin
        gx <= $signed(gx_pos) - $signed(gx_neg);
        gy <= $signed(gy_pos) - $signed(gy_neg);
        x1 <= wx;  y1 <= wy;
        if (rst) v1 <= 1'b0; else v1 <= wv;
    end

    // ---- stage 2 : squares + sum, direction ----
    wire [10:0] gx_abs = gx[10] ? -gx : gx;
    wire [10:0] gy_abs = gy[10] ? -gy : gy;
    wire [9:0]  ax = gx_abs[9:0];                        // |Gx| <= 1020
    wire [9:0]  ay = gy_abs[9:0];                        // |Gy| <= 1020

    reg [20:0] sum_sq;
    reg [1:0]  dir2;
    reg        v2;  reg [8:0] x2;  reg [7:0] y2;

    always @(posedge clk) begin
        sum_sq <= ax * ax + ay * ay;                    // Gx^2 + Gy^2
        if ({ay, 7'd0} <= ax * 6'd53)       dir2 <= 2'd0;
        else if ({ax, 7'd0} <= ay * 6'd53)  dir2 <= 2'd2;
        else if (gx[10] == gy[10])          dir2 <= 2'd1;
        else                                dir2 <= 2'd3;
        x2 <= x1;  y2 <= y1;
        if (rst) v2 <= 1'b0; else v2 <= v1;
    end

    // ---- stage 3 : limit ----
    reg [15:0] limited;
    reg [1:0]  dir3;
    reg        v3;  reg [8:0] x3;  reg [7:0] y3;

    always @(posedge clk) begin
        limited <= (sum_sq > 21'd65025) ? 16'd65025 : sum_sq[15:0];
        dir3 <= dir2;
        x3 <= x2;  y3 <= y2;
        if (rst) v3 <= 1'b0; else v3 <= v2;
    end

    // ---- stage 4 : square root ----
    wire [18:0] tag;

    square_root #(.TW(19)) u_sqrt (
        .clk(clk), .rst(rst),
        .in_valid(v3), .in_val(limited), .in_tag({dir3, y3, x3}),
        .out_valid(out_valid), .out_root(out_mag), .out_tag(tag)
    );

    assign out_x   = tag[8:0];
    assign out_y   = tag[16:9];
    assign out_dir = tag[18:17];
endmodule
