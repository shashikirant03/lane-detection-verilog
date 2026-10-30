// ============================================================================
// ov7670_capture.v  -  receives the pixel stream of the OV7670 camera
//
//  D[7:0] : one byte per PCLK  (the camera changes it on the falling edge,
//           so it is sampled on the rising edge)
//  HREF   : 1 while a row is output  (640 pixels = 1280 bytes)
//  VSYNC  : short high pulse at the start of every frame
//
//  RGB565 = 2 bytes per pixel:
//      1st byte : R4 R3 R2 R1 R0 G5 G4 G3
//      2nd byte : G2 G1 G0 B4 B3 B2 B1 B0
//
// Outputs every pixel with its column x (0..639) and row y (0..479).
// frame_end is a one-clock pulse when VSYNC rises (= the previous frame is
// complete).  Nothing is output until the first VSYNC, so the first (half)
// frame after power-up is ignored.
// ============================================================================
module ov7670_capture (
    input              pclk,
    input              rst,
    input      [7:0]   ov_d,
    input              ov_href,
    input              ov_vsync,
    output reg         pix_valid,
    output reg [9:0]   pix_x,
    output reg [8:0]   pix_y,
    output reg [15:0]  pix_rgb565,
    output reg         frame_end
);
    reg [7:0] rD;
    reg       rH, rV;           // sampled inputs
    reg       pH, pV;           // previous values
    reg       synced;           // a VSYNC has been seen
    reg       second;           // 0 = 1st byte of the pixel, 1 = 2nd byte
    reg [7:0] first_byte;
    reg [9:0] xcnt;
    reg [8:0] ycnt;

    always @(posedge pclk) begin
        rD <= ov_d;
        rH <= ov_href;
        rV <= ov_vsync;
    end

    always @(posedge pclk) begin
        if (rst) begin
            pH <= 1'b0;  pV <= 1'b0;  synced <= 1'b0;
            second <= 1'b0;
            xcnt <= 10'd0;  ycnt <= 9'd0;
            pix_valid <= 1'b0;  frame_end <= 1'b0;
        end else begin
            pH <= rH;
            pV <= rV;
            pix_valid <= 1'b0;
            frame_end <= rV & ~pV & synced;          // VSYNC rising edge

            if (rV) begin                            // new frame starts
                synced <= 1'b1;
                ycnt   <= 9'd0;
                xcnt   <= 10'd0;
                second <= 1'b0;
            end else if (rH) begin                   // inside a row
                if (!second) begin
                    first_byte <= rD;
                    second     <= 1'b1;
                end else begin
                    second     <= 1'b0;
                    pix_valid  <= synced && (xcnt < 10'd640) && (ycnt < 9'd480);
                    pix_x      <= xcnt;
                    pix_y      <= ycnt;
                    pix_rgb565 <= {first_byte, rD};
                    xcnt       <= xcnt + 10'd1;
                end
            end else begin                           // horizontal blanking
                second <= 1'b0;
                xcnt   <= 10'd0;
                if (pH) ycnt <= ycnt + 9'd1;         // HREF fell: next row
            end
        end
    end
endmodule
