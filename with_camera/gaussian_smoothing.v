// ============================================================================
// gaussian_smoothing.v  -  Canny step 2: GAUSSIAN SMOOTHING
//                          paper Sec. IV-A-2, eq. (1), (2), Algorithm 3 lines 2-5
//
// 5 x 5 Gaussian kernel, sigma = 1.0, mean = 0 :
//
//            1   [ 1  4  7  4  1 ]
//     B  =  ---  [ 4 16 26 16  4 ]  * A
//           273  [ 7 26 41 26  7 ]
//                [ 4 16 26 16  4 ]
//                [ 1  4  7  4  1 ]
//
// The 25 products of the window are computed in parallel.
// Division by 273 in fixed point:  sum / 273  =  (sum * 240) >> 16
// (240 / 65536 = 1 / 273.07).
// Latency: 2 (window) + 3 clocks.
// ============================================================================
module gaussian_smoothing #(
    parameter IMG_W = 320,
    parameter X0    = 0,
    parameter Y0    = 0
)(
    input              clk,
    input              rst,
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_pix,
    output reg         out_valid,
    output reg [8:0]   out_x,
    output reg [7:0]   out_y,
    output reg [7:0]   out_pix
);
    wire           wv;
    wire [8:0]     wx;
    wire [7:0]     wy;
    wire [25*8-1:0] win;

    sliding_window #(.DW(8), .K(5), .IMG_W(IMG_W), .X0(X0), .Y0(Y0)) u_win (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_pix(in_pix),
        .out_valid(wv), .out_x(wx), .out_y(wy), .win(win)
    );

    // A[r][c] = pixel at window row r, column c
    function [7:0] A;
        input [25*8-1:0] wbus;
        input [3:0] r;
        input [3:0] c;
        begin
            A = wbus[(r*5 + c)*8 +: 8];
        end
    endfunction

    // ---- stage 1 : one weighted sum per kernel row ----
    // kernel coefficients of eq. (2)
    localparam [14:0] K4 = 15'd4, K7 = 15'd7, K16 = 15'd16, K26 = 15'd26, K41 = 15'd41;
    reg [14:0] row0, row1, row2, row3, row4;
    reg        v1;  reg [8:0] x1;  reg [7:0] y1;

    always @(posedge clk) begin
        row0 <=        A(win,0,0) + K4 *A(win,0,1) + K7 *A(win,0,2) + K4 *A(win,0,3) +        A(win,0,4);
        row1 <= K4 *A(win,1,0) + K16*A(win,1,1) + K26*A(win,1,2) + K16*A(win,1,3) + K4 *A(win,1,4);
        row2 <= K7 *A(win,2,0) + K26*A(win,2,1) + K41*A(win,2,2) + K26*A(win,2,3) + K7 *A(win,2,4);
        row3 <= K4 *A(win,3,0) + K16*A(win,3,1) + K26*A(win,3,2) + K16*A(win,3,3) + K4 *A(win,3,4);
        row4 <=        A(win,4,0) + K4 *A(win,4,1) + K7 *A(win,4,2) + K4 *A(win,4,3) +        A(win,4,4);
        x1 <= wx;  y1 <= wy;
        if (rst) v1 <= 1'b0; else v1 <= wv;
    end

    // ---- stage 2 : total (max 273 * 255 = 69615) ----
    reg [16:0] total;
    reg        v2;  reg [8:0] x2;  reg [7:0] y2;

    always @(posedge clk) begin
        total <= row0 + row1 + row2 + row3 + row4;
        x2 <= x1;  y2 <= y1;
        if (rst) v2 <= 1'b0; else v2 <= v1;
    end

    // ---- stage 3 : divide by 273 ----
    wire [24:0] scaled = total * 8'd240;

    always @(posedge clk) begin
        out_pix <= scaled[23:16];
        out_x   <= x2;
        out_y   <= y2;
        if (rst) out_valid <= 1'b0; else out_valid <= v2;
    end
endmodule
