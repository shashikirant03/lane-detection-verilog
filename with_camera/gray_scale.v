// ============================================================================
// gray_scale.v   -  Canny step 1: GRAYSCALE CONVERSION
//                   paper Sec. IV-A-1, Algorithm 2 (Gray_scale), eq. (8)
//
// Paper (fixed point, FPGA version):
//      Y = 0.299 R + 0.587 G + 0.114 B
//        ~ (5 R + 9 G + 2 B) / 16           (0.299*16=4.78~5, 0.587*16=9.39~9,
//                                            0.114*16=1.82~2)
//
// The OV7670 gives a 640 x 480 RGB565 image (R 5 bit, G 6 bit, B 5 bit).
// Each 2 x 2 block of pixels is averaged into ONE pixel, so the output image
// is 320 x 240 (same size as the paper's processing image).
//   * the two pixels of a row are added in registers
//   * the sum of an even row is kept in a 320-word line memory and added to
//     the odd row below it
// Then R, G, B are scaled to 8 bit and eq. (8) gives the gray value.
// The colour (RGB565) is also output for the display.
// ============================================================================
module gray_scale (
    input              clk,
    input              rst,
    input              in_valid,
    input      [9:0]   in_x,          // 0..639
    input      [8:0]   in_y,          // 0..479
    input      [15:0]  in_rgb565,
    output reg         out_valid,
    output reg [8:0]   out_x,         // 0..319
    output reg [7:0]   out_y,         // 0..239
    output reg [7:0]   out_gray,
    output reg [15:0]  out_rgb565
);
    wire [4:0] r5 = in_rgb565[15:11];
    wire [5:0] g6 = in_rgb565[10:5];
    wire [4:0] b5 = in_rgb565[4:0];
    wire [8:0] bx = in_x[9:1];                   // block column = output x

    // ---- sum of the 2 pixels of this row of the block ----
    reg  [4:0] hR, hB;                           // left pixel of the pair
    reg  [5:0] hG;
    wire [5:0] sR = hR + r5;                     // 2 x 5 bit -> 6 bit
    wire [6:0] sG = hG + g6;                     // 2 x 6 bit -> 7 bit
    wire [5:0] sB = hB + b5;

    always @(posedge clk)
        if (in_valid && !in_x[0]) begin          // even column: keep it
            hR <= r5;  hG <= g6;  hB <= b5;
        end

    // ---- line memory: sums of the even row ----
    wire [18:0] up;                              // {R 6 bit, G 7 bit, B 6 bit}
    dual_port_ram #(.DW(19), .AW(9), .DEPTH(320)) u_line (
        .wclk(clk), .we(in_valid && in_x[0] && !in_y[0]), .waddr(bx), .wdata({sR, sG, sB}),
        .rclk(clk), .raddr(bx), .rdata(up)
    );
    // (the read address bx is already stable one pixel earlier, at the even
    //  column, so 'up' is ready when the odd column arrives)

    // ---- odd row, odd column: the 2 x 2 block is complete ----
    reg        cv;
    reg [8:0]  cx;
    reg [7:0]  cy;
    reg [7:0]  R, G, B;
    wire [6:0] tR = up[18:13] + sR;              // sum of 4 x 5 bit (0..124)
    wire [7:0] tG = up[12:6]  + sG;              // sum of 4 x 6 bit (0..252)
    wire [6:0] tB = up[5:0]   + sB;

    always @(posedge clk) begin
        if (rst) cv <= 1'b0;
        else     cv <= in_valid && in_x[0] && in_y[0];
        cx <= bx;
        cy <= in_y[8:1];
        // average and scale to 8 bit:
        //   R8 = (sum/4) * 8 = sum * 2     (5 bit -> 8 bit), +sum/64 so 124 -> 249
        //   G8 = (sum/4) * 4 = sum         (6 bit -> 8 bit)
        R  <= {tR, 1'b0} + tR[6];
        G  <= tG;
        B  <= {tB, 1'b0} + tB[6];
    end

    // ---- eq. (8) : Y = (5R + 9G + 2B) / 16 ----
    wire [11:0] y16 = 12'd5 * R + 12'd9 * G + 12'd2 * B;

    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= cv;
        out_x      <= cx;
        out_y      <= cy;
        out_gray   <= y16[11:4];
        out_rgb565 <= {R[7:3], G[7:2], B[7:3]};
    end
endmodule
