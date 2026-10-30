// ============================================================================
// lane_detection_top.v  -  TOP LEVEL (DE2-115 + OV7670 camera + VGA)
//
// Real-time lane detection following
//   H. Yun, D. Park, "Low-Power Lane Detection Unit With Sliding-Based
//   Parallel Segment Detection Accelerator for FPGA", IEEE Access, 2024.
//
//  camera clock (PCLK)                 |  25 MHz FPGA clock
//                                      |
//  OV7670 -> ov7670_capture            |
//          (640x480 RGB565)            |
//       -> gray_scale -----> async_fifo --> lane_detection :
//          (2x2 average,     (clock    |      Canny (Gaussian, Sobel, NMS,
//           Algorithm 2,      domain   |             Hysteresis)
//           eq. 8, 320x240)   crossing)|      Hough transform (CORDIC, Fig. 13)
//                                      |      Local maxima (Algorithm 1)
//                                      |  -> colour + edge frame memories
//                                      |  -> vga_display (draws the lanes)
// CONTROLS
//   KEY0        reset
//   KEY1        send the camera settings again
//   SW1..SW0    display: 00 colour camera + lanes | 01 gray
//                        10 Canny edges          | 11 edges + lanes
//               Both lanes are drawn as red lines in the bottom half of the
//               picture; the two lines never touch.
//   SW2         freeze picture
//   SW3         camera mirror (left/right)
//   SW4         camera flip   (upside down)
//   SW9         lane peak search: 0 = highest cell of each lane region (default)
//                                 1 = Algorithm 1 of the paper (divide & conquer)
//   SW17..SW10  hysteresis threshold Th (all 0 = 30)
//   HEX7..HEX0  not used (all segments kept OFF)
//   LEDG0/1     left / right lane found   LEDG2 Hough busy
//   LEDG3       camera configured         LEDG4 camera SCCB error (no answer)
//   LEDG5       frozen                    LEDG6 camera frame blink
//   LEDG7       FIFO overflow (should stay off)
//   LEDG8       PLL locked                LEDR  = switches
// ============================================================================
module lane_detection_top (
    input          CLOCK_50,
    input  [1:0]   KEY,              // KEY0 reset, KEY1 camera re-config
    input  [17:0]  SW,
    output [17:0]  LEDR,
    output [8:0]   LEDG,
    output [6:0]   HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
    // VGA
    output         VGA_CLK,
    output         VGA_HS,
    output         VGA_VS,
    output         VGA_BLANK_N,
    output         VGA_SYNC_N,
    output [7:0]   VGA_R,
    output [7:0]   VGA_G,
    output [7:0]   VGA_B,
    // OV7670 camera (GPIO header JP5)
    input          OV_PCLK,
    input  [7:0]   OV_D,
    input          OV_HREF,
    input          OV_VSYNC,
    output         OV_XCLK,
    output         OV_RESET_N,
    output         OV_PWDN,
    output         OV_SIOC,
    inout          OV_SIOD
);
    // ================================================================ clocks
    wire clk50 = CLOCK_50;
    wire pclk  = OV_PCLK;                      // camera pixel clock
    wire clk25, clk_cam, pll_locked;

    pll_25m u_pll (
        .inclk0(clk50), .areset(1'b0),
        .c0(clk25), .c1(clk_cam), .locked(pll_locked)
    );

    assign OV_XCLK = clk_cam;                  // 25 MHz camera input clock
    assign VGA_CLK = ~clk25;                   // DAC samples in the middle of a pixel

    // ================================================================ resets
    // asynchronous assert, synchronous release in every clock domain
    wire reset_n = KEY[0] & pll_locked;
    reg [2:0] r50, r25, rpc;
    always @(posedge clk50 or negedge reset_n) if (!reset_n) r50 <= 3'b111; else r50 <= {r50[1:0], 1'b0};
    always @(posedge clk25 or negedge reset_n) if (!reset_n) r25 <= 3'b111; else r25 <= {r25[1:0], 1'b0};
    always @(posedge pclk  or negedge reset_n) if (!reset_n) rpc <= 3'b111; else rpc <= {rpc[1:0], 1'b0};
    wire rst50 = r50[2];
    wire rst25 = r25[2];
    wire rstpc = rpc[2];

    // switches synchronised to each clock
    reg [17:0] sw50a, sw50, sw25a, sw25;
    always @(posedge clk50) begin sw50a <= SW; sw50 <= sw50a; end
    always @(posedge clk25) begin sw25a <= SW; sw25 <= sw25a; end

    // ================================================================ camera set-up (50 MHz)
    reg [2:0] key1;
    always @(posedge clk50) key1 <= {key1[1:0], ~KEY[1]};
    wire reconfig = key1[1] & ~key1[2];            // KEY1 pressed

    wire sda_oe, cam_ok, cam_err;

    ov7670_config u_config (
        .clk(clk50), .rst(rst50),
        .mirror(sw50[3]), .vflip(sw50[4]), .reconfig(reconfig),
        .cam_reset_n(OV_RESET_N), .cam_pwdn(OV_PWDN),
        .sccb_scl(OV_SIOC), .sccb_sda_oe(sda_oe), .sccb_sda_in(OV_SIOD),
        .config_done(cam_ok), .ack_error(cam_err)
    );
    assign OV_SIOD = sda_oe ? 1'b0 : 1'bz;         // open drain

    // ================================================================ camera clock domain (PCLK)
    wire        cap_valid, cap_fend;
    wire [9:0]  cap_x;
    wire [8:0]  cap_y;
    wire [15:0] cap_rgb;

    ov7670_capture u_capture (
        .pclk(pclk), .rst(rstpc),
        .ov_d(OV_D), .ov_href(OV_HREF), .ov_vsync(OV_VSYNC),
        .pix_valid(cap_valid), .pix_x(cap_x), .pix_y(cap_y), .pix_rgb565(cap_rgb),
        .frame_end(cap_fend)
    );

    wire        g_valid;
    wire [8:0]  g_x;
    wire [7:0]  g_y, g_gray;
    wire [15:0] g_rgb;

    gray_scale u_gray (
        .clk(pclk), .rst(rstpc),
        .in_valid(cap_valid), .in_x(cap_x), .in_y(cap_y), .in_rgb565(cap_rgb),
        .out_valid(g_valid), .out_x(g_x), .out_y(g_y),
        .out_gray(g_gray), .out_rgb565(g_rgb)
    );

    // FIFO word = {end-of-frame marker, x, y, gray, rgb565}
    // (a pixel and the end of frame never come in the same clock: VSYNC
    //  rises many rows after the last pixel)
    wire        fifo_full, f_valid;
    wire [41:0] f_data;

    async_fifo #(.DW(42), .AW(4)) u_fifo (
        .wclk(pclk), .wrst(rstpc),
        .we(g_valid | cap_fend),
        .wdata(cap_fend ? {1'b1, 41'd0} : {1'b0, g_x, g_y, g_gray, g_rgb}),
        .full(fifo_full),
        .rclk(clk25), .rrst(rst25),
        .out_valid(f_valid), .out_data(f_data)
    );

    reg overflow;                                  // sticky error flag
    always @(posedge pclk)
        if (rstpc) overflow <= 1'b0; else if ((g_valid | cap_fend) && fifo_full) overflow <= 1'b1;

    // ================================================================ 25 MHz domain
    wire        p_fend  = f_valid &  f_data[41];
    wire        p_valid = f_valid & ~f_data[41];
    wire [8:0]  p_x     = f_data[40:32];
    wire [7:0]  p_y     = f_data[31:24];
    wire [7:0]  p_gray  = f_data[23:16];
    wire [15:0] p_rgb   = f_data[15:0];

    wire [7:0] th = (sw25[17:10] == 8'd0) ? 8'd30 : sw25[17:10];

    wire        record, e_valid, e_bit, lanes_valid, hough_busy;
    wire [8:0]  e_x;
    wire [7:0]  e_y;
    wire        l_ok, r_ok;
    wire [6:0]  l_t, r_t;
    wire [8:0]  l_r, r_r;

    lane_detection u_lane (
        .clk(clk25), .rst(rst25), .th(th), .peak_mode(sw25[9]), .freeze(sw25[2]),
        .in_valid(p_valid), .in_x(p_x), .in_y(p_y), .in_gray(p_gray),
        .frame_end(p_fend),
        .record(record),
        .edge_valid(e_valid), .edge_x(e_x), .edge_y(e_y), .edge_bit(e_bit),
        .lanes_valid(lanes_valid),
        .left_ok(l_ok),  .left_t(l_t),  .left_r(l_r),
        .right_ok(r_ok), .right_t(r_t), .right_r(r_r),
        .hough_busy(hough_busy)
    );

    // keep the last result for the display
    reg       hl_ok, hr_ok;
    reg [6:0] hl_t, hr_t;
    reg [8:0] hl_r, hr_r;
    always @(posedge clk25) begin
        if (rst25) begin
            hl_ok <= 1'b0;  hr_ok <= 1'b0;
            hl_t <= 7'd0;  hr_t <= 7'd0;  hl_r <= 9'd256;  hr_r <= 9'd256;
        end else if (lanes_valid) begin
            hl_ok <= l_ok;  hl_t <= l_t;  hl_r <= l_r;
            hr_ok <= r_ok;  hr_t <= r_t;  hr_r <= r_r;
        end
    end

    reg frame_blink;
    always @(posedge clk25)
        if (rst25) frame_blink <= 1'b0; else if (p_fend) frame_blink <= ~frame_blink;

    // ================================================================ frame memories (320 x 240)
    wire [16:0] rgb_waddr  = p_y * 17'd320 + p_x;
    wire [16:0] edge_waddr = e_y * 17'd320 + e_x;
    wire [16:0] vga_addr;
    wire [15:0] vga_rgb;
    wire        vga_edge;

    dual_port_ram #(.DW(16), .AW(17), .DEPTH(76800)) u_rgb_mem (
        .wclk(clk25), .we(p_valid && record), .waddr(rgb_waddr), .wdata(p_rgb),
        .rclk(clk25), .raddr(vga_addr), .rdata(vga_rgb)
    );

    dual_port_ram #(.DW(1), .AW(17), .DEPTH(76800)) u_edge_mem (
        .wclk(clk25), .we(e_valid && record), .waddr(edge_waddr), .wdata(e_bit),
        .rclk(clk25), .raddr(vga_addr), .rdata(vga_edge)
    );

    // ================================================================ VGA
    vga_display #(.ROI_Y(120)) u_vga (
        .clk(clk25), .rst(rst25), .mode(sw25[1:0]),
        .left_ok(hl_ok),  .left_t(hl_t),  .left_r(hl_r),
        .right_ok(hr_ok), .right_t(hr_t), .right_r(hr_r),
        .mem_addr(vga_addr), .mem_rgb565(vga_rgb), .mem_edge(vga_edge),
        .vga_r(VGA_R), .vga_g(VGA_G), .vga_b(VGA_B),
        .vga_hs(VGA_HS), .vga_vs(VGA_VS), .vga_blank_n(VGA_BLANK_N), .vga_sync_n(VGA_SYNC_N)
    );

    // ================================================================ 7-segment displays / LEDs
    // seven-segment displays are not used: segments are active low, so
    // driving 1 keeps every segment dark
    assign HEX0 = 7'h7F;  assign HEX1 = 7'h7F;  assign HEX2 = 7'h7F;  assign HEX3 = 7'h7F;
    assign HEX4 = 7'h7F;  assign HEX5 = 7'h7F;  assign HEX6 = 7'h7F;  assign HEX7 = 7'h7F;

    assign LEDR = SW;
    assign LEDG = {pll_locked, overflow, frame_blink, ~record, cam_err, cam_ok,
                   hough_busy, hr_ok, hl_ok};
endmodule
