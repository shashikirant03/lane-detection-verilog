// ============================================================================
// vga_display.v  -  640 x 480 @ 60 Hz VGA output + drawing of the lanes
//                   (paper Fig. 10 "Image post-processing":
//                    convert lane data (rho, theta) to (x, y), draw lines)
//
// The 320 x 240 frame memories are shown 2x enlarged.
// A pixel belongs to a lane line when it satisfies eq. (7):
//
//        | x*K*cos(theta) + y*K*sin(theta) - K*rho |  <  1.5
//
// (x, y) = screen pixel relative to the screen centre.  Screen pixels are
// half image pixels, so rho is doubled.
// K*cos(theta) and K*sin(theta) are produced with the SAME CORDIC rotator as
// the Hough transform (Fig. 13): rotating the vector (1, 0) i times gives
//        x(i) =  K_i cos(i*phi),   y(i) = -K_i sin(i*phi)
// This is done once per frame (in the vertical blanking, <= 100 clocks), so
// the gain K_i is exactly the same as in the votes and no table is needed.
// Lines are drawn only below the ROI row (bottom half), as in the paper.
// Both lanes are drawn in red, 3 pixels wide.  No line is drawn at the ROI
// row itself (the picture stays clean).
//
// The two lines NEVER touch: when both lanes are found, a pixel of one line
// is drawn only if it is on the inner side of the other lane and at least
// MIN_SEP = 14 screen pixels away from the other lane's centre line.  A
// pixel of the other line is less than 1.5 pixels from that centre line,
// so there are always at least 14 - 1.5 = 12.5 pixels between the two
// lines, whatever the lane angles are; near the vanishing point both
// lines stop before they meet.
//
// mode : 0 = camera colour + lanes      1 = grayscale
//        2 = Canny edges                3 = Canny edges + lanes
// ============================================================================
module vga_display #(
    parameter ROI_Y = 120
)(
    input              clk,            // 25 MHz pixel clock
    input              rst,
    input      [1:0]   mode,
    // lane result
    input              left_ok,
    input      [6:0]   left_t,
    input      [8:0]   left_r,
    input              right_ok,
    input      [6:0]   right_t,
    input      [8:0]   right_r,
    // frame memories (data 1 clock after the address)
    output reg [16:0]  mem_addr,
    input      [15:0]  mem_rgb565,
    input              mem_edge,
    // VGA
    output reg [7:0]   vga_r,
    output reg [7:0]   vga_g,
    output reg [7:0]   vga_b,
    output reg         vga_hs,
    output reg         vga_vs,
    output reg         vga_blank_n,
    output             vga_sync_n
);
    // ---- 640x480 @ 60 Hz timing ----
    localparam H_VISIBLE = 640, H_FRONT = 16, H_SYNC = 96, H_TOTAL = 800;
    localparam V_VISIBLE = 480, V_FRONT = 10, V_SYNC = 2,  V_TOTAL = 525;
    localparam signed [27:0] HALF_WIDTH = 28'sd6144;      // 1.5 in Q12 (line 3 px wide)
    localparam signed [27:0] MIN_SEP    = 28'sd57344;     // 14  in Q12 (lines never closer)

    assign vga_sync_n = 1'b0;

    reg [9:0] hc, vc;
    always @(posedge clk) begin
        if (rst) begin
            hc <= 10'd0;  vc <= 10'd0;
        end else if (hc == H_TOTAL - 1) begin
            hc <= 10'd0;
            vc <= (vc == V_TOTAL - 1) ? 10'd0 : vc + 10'd1;
        end else
            hc <= hc + 10'd1;
    end

    wire visible0 = (hc < H_VISIBLE) && (vc < V_VISIBLE);
    wire hs0      = ~((hc >= H_VISIBLE + H_FRONT) && (hc < H_VISIBLE + H_FRONT + H_SYNC));
    wire vs0      = ~((vc >= V_VISIBLE + V_FRONT) && (vc < V_VISIBLE + V_FRONT + V_SYNC));
    wire roi0     = (vc >= 2 * ROI_Y);                    // bottom half (region of interest)
    wire [16:0] addr0 = vc[8:1] * 17'd320 + hc[9:1];      // image pixel address

    wire new_frame = (hc == 10'd0) && (vc == V_VISIBLE);  // start of vertical blanking

    // ---- lane parameters + K*cos, K*sin with the CORDIC (once per frame) ----
    reg        Lok, Rok;
    reg [6:0]  Lt, Rt;
    reg signed [27:0] Lrho, Rrho;                         // 2*rho in Q12
    reg [6:0]  step_cnt;
    reg        stepping;

    wire signed [23:0] lx, ly, rx, ry;                    // Q20
    cordic_rotator #(.W(24)) u_cos_left (
        .clk(clk), .load(new_frame), .step(stepping && (step_cnt < Lt)),
        .x_in(24'sd1048576), .y_in(24'sd0), .x(lx), .y(ly)
    );
    cordic_rotator #(.W(24)) u_cos_right (
        .clk(clk), .load(new_frame), .step(stepping && (step_cnt < Rt)),
        .x_in(24'sd1048576), .y_in(24'sd0), .x(rx), .y(ry)
    );

    reg signed [14:0] LC, LS, RC, RS;                     // K cos, K sin in Q12
    wire signed [23:0] lc_q12 = (lx + 24'sd128) >>> 8;
    wire signed [23:0] ls_q12 = (24'sd128 - ly) >>> 8;
    wire signed [23:0] rc_q12 = (rx + 24'sd128) >>> 8;
    wire signed [23:0] rs_q12 = (24'sd128 - ry) >>> 8;

    always @(posedge clk) begin
        if (rst) begin
            Lok <= 1'b0;  Rok <= 1'b0;  Lt <= 7'd0;  Rt <= 7'd0;
            Lrho <= 28'sd0;  Rrho <= 28'sd0;
            stepping <= 1'b0;  step_cnt <= 7'd0;
            LC <= 15'sd4096;  LS <= 15'sd0;  RC <= 15'sd4096;  RS <= 15'sd0;
        end else if (new_frame) begin
            Lok  <= left_ok;   Lt <= left_t;
            Rok  <= right_ok;  Rt <= right_t;
            Lrho <= ($signed({1'b0, left_r})  - 10'sd256) <<< 13;
            Rrho <= ($signed({1'b0, right_r}) - 10'sd256) <<< 13;
            stepping <= 1'b1;
            step_cnt <= 7'd0;
        end else if (stepping) begin
            step_cnt <= step_cnt + 7'd1;
            if (step_cnt == 7'd101) begin                 // all rotations done
                stepping <= 1'b0;
                LC <= lc_q12[14:0];                       // Q20 -> Q12 (rounded)
                LS <= ls_q12[14:0];
                RC <= rc_q12[14:0];
                RS <= rs_q12[14:0];
            end
        end
    end

    // ---- pipeline stage 1 : memory address, pixel position ----
    reg               vis1, hs1, vs1, roi1;
    reg signed [10:0] xs1, ys1;
    always @(posedge clk) begin
        mem_addr <= addr0;
        vis1 <= visible0;  hs1 <= hs0;  vs1 <= vs0;  roi1 <= roi0;
        xs1  <= $signed({1'b0, hc}) - 11'sd320;           // relative to centre
        ys1  <= $signed({1'b0, vc}) - 11'sd240;
    end

    // ---- stage 2 : x*Kcos + y*Ksin ----
    reg               vis2, hs2, vs2, roi2;
    reg signed [27:0] mL, mR;
    always @(posedge clk) begin
        mL <= xs1 * LC + ys1 * LS;
        mR <= xs1 * RC + ys1 * RS;
        vis2 <= vis1;  hs2 <= hs1;  vs2 <= vs1;  roi2 <= roi1;
    end

    // ---- stage 3 : on the line?  + pixel from memory ----
    wire signed [27:0] dL = mL - Lrho;
    wire signed [27:0] dR = mR - Rrho;
    reg        vis3, hs3, vs3, onL, onR, edge3;
    reg [15:0] rgb3;
    always @(posedge clk) begin
        // on the line, in the bottom half, and (both lanes found) at least
        // MIN_SEP away from the other lane, on its inner side
        onL   <= Lok && roi2 && (dL > -HALF_WIDTH) && (dL < HALF_WIDTH) && (!Rok || (dR >= MIN_SEP));
        onR   <= Rok && roi2 && (dR > -HALF_WIDTH) && (dR < HALF_WIDTH) && (!Lok || (dL >= MIN_SEP));
        rgb3  <= mem_rgb565;
        edge3 <= mem_edge;
        vis3 <= vis2;  hs3 <= hs2;  vs3 <= vs2;
    end

    // ---- stage 4 : colour ----
    wire [7:0]  R8 = {rgb3[15:11], rgb3[15:13]};
    wire [7:0]  G8 = {rgb3[10:5],  rgb3[10:9]};
    wire [7:0]  B8 = {rgb3[4:0],   rgb3[4:2]};
    wire [11:0] y16 = 12'd5 * R8 + 12'd9 * G8 + 12'd2 * B8;   // eq. (8)
    wire [7:0]  Y8  = y16[11:4];
    wire        show_lanes = (mode == 2'd0) || (mode == 2'd3);

    always @(posedge clk) begin
        vga_hs      <= hs3;
        vga_vs      <= vs3;
        vga_blank_n <= vis3;
        if (!vis3) begin
            vga_r <= 8'd0;   vga_g <= 8'd0;   vga_b <= 8'd0;
        end else if (show_lanes && (onL || onR)) begin
            vga_r <= 8'd255; vga_g <= 8'd0;   vga_b <= 8'd0;      // lane = red
        end else begin
            case (mode)
                2'd0:    begin vga_r <= R8; vga_g <= G8; vga_b <= B8; end
                2'd1:    begin vga_r <= Y8; vga_g <= Y8; vga_b <= Y8; end
                default: begin vga_r <= {8{edge3}}; vga_g <= {8{edge3}}; vga_b <= {8{edge3}}; end
            endcase
        end
    end
endmodule
