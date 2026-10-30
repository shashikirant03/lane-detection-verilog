// ============================================================================
// hysteresis.v  -  Canny step 5: HYSTERESIS (thresholding)
//                  paper Sec. IV-A-5, Algorithm 3 lines 23-27
//
//   if (Pix_nonMax < Th)  Pix_Hys = 0      (weak edge removed)
//   else                  Pix_Hys = 1      (edge pixel)
//
// Th comes from the slide switches, so it can be tuned to the lighting.
// Latency: 1 clock.
// ============================================================================
module hysteresis (
    input              clk,
    input              rst,
    input      [7:0]   th,
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_mag,
    output reg         out_valid,
    output reg [8:0]   out_x,
    output reg [7:0]   out_y,
    output reg         out_edge
);
    always @(posedge clk) begin
        out_edge <= (in_mag < th) ? 1'b0 : 1'b1;
        out_x    <= in_x;
        out_y    <= in_y;
        if (rst) out_valid <= 1'b0; else out_valid <= in_valid;
    end
endmodule
