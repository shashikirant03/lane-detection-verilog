// ============================================================================
// de2_115_lane_top.v : Terasic DE2-115 (Cyclone IV E EP4CE115F29C7) top level
//
//   KEY[0]   : reset (active low, hold to reset)
//   KEY[1]   : run the lane detector again (it also runs once after power-up)
//   LEDG[0]  : done          LEDG[1] : left lane found   LEDG[2] : right found
//   LEDR[0]  : busy          LEDG[8] : heartbeat
//   VGA      : 640x480@60, shows the result (2x zoom): white = edge, red = lane
//
// Memories reachable through Quartus "In-System Memory Content Editor" (JTAG):
//   IB0..IB3 : input image banks (24-bit RGB)  -> import new .mif, press KEY1
//   OUT      : output image, W*H words x 2 bit -> export to .mif (the result)
//   RES      : 16 x 32-bit result words        -> export to .mif
// ============================================================================
module de2_115_lane_top (
    input         CLOCK_50,
    input  [3:0]  KEY,
    output [17:0] LEDR,
    output [8:0]  LEDG,
    output        VGA_CLK,
    output        VGA_HS,
    output        VGA_VS,
    output        VGA_BLANK_N,
    output        VGA_SYNC_N,
    output [7:0]  VGA_R,
    output [7:0]  VGA_G,
    output [7:0]  VGA_B
);
    // ---------------- design parameters (keep in sync with lane_params.py) --
    localparam W = 320, H = 240, T = 4;          // this top supports T = 4
    localparam S = H / T, BD = S * W, NPIX = W * H;
    localparam BAW = $clog2(BD), OAW = $clog2(NPIX);

    wire clk = CLOCK_50;

    // ---------------- reset synchroniser ----------------
    reg [2:0] rst_sync = 3'b111;
    always @(posedge clk) rst_sync <= {rst_sync[1:0], ~KEY[0]};
    wire rst = rst_sync[2];

    // ---------------- start: once after reset, and on KEY[1] --------------
    reg [20:0] por_cnt;
    reg        por_done;
    reg [2:0]  k1_sync;
    reg        start;
    always @(posedge clk) begin
        k1_sync <= {k1_sync[1:0], KEY[1]};
        start   <= 1'b0;
        if (rst) begin
            por_cnt  <= 21'd0;
            por_done <= 1'b0;
        end else begin
            if (!por_done) begin
                por_cnt <= por_cnt + 1'b1;
                if (&por_cnt) begin por_done <= 1'b1; start <= 1'b1; end
            end
            if (k1_sync[2] & ~k1_sync[1]) start <= 1'b1;   // KEY1 pressed
        end
    end

    // ---------------- input image ROM banks ----------------
    wire [BAW-1:0]  rom_addr;
    wire [T*24-1:0] rom_data;
    img_bank_rom #(.INIT_FILE("img_bank0.mif"), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=IB0"), .DEPTH(BD), .AW(BAW)) u_ib0 (
        .clk(clk), .addr(rom_addr), .q(rom_data[0*24 +: 24]));
    img_bank_rom #(.INIT_FILE("img_bank1.mif"), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=IB1"), .DEPTH(BD), .AW(BAW)) u_ib1 (
        .clk(clk), .addr(rom_addr), .q(rom_data[1*24 +: 24]));
    img_bank_rom #(.INIT_FILE("img_bank2.mif"), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=IB2"), .DEPTH(BD), .AW(BAW)) u_ib2 (
        .clk(clk), .addr(rom_addr), .q(rom_data[2*24 +: 24]));
    img_bank_rom #(.INIT_FILE("img_bank3.mif"), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=IB3"), .DEPTH(BD), .AW(BAW)) u_ib3 (
        .clk(clk), .addr(rom_addr), .q(rom_data[3*24 +: 24]));

    // ---------------- the accelerator ----------------
    wire            busy, done, out_we;
    wire [OAW-1:0]  out_addr;
    wire [1:0]      out_data;
    wire            found_l, found_r;
    wire [6:0]      k_l, k_r;
    wire [15:0]     bin_l, bin_r, votes_l, votes_r;
    wire [31:0]     n_edge, cyc_canny, cyc_hough, cyc_render, cyc_total;

    lane_detect_core #(.W(W), .H(H), .T(T)) u_core (
        .clk(clk), .rst(rst), .start(start), .busy(busy), .done(done),
        .rom_addr(rom_addr), .rom_data(rom_data),
        .out_we(out_we), .out_addr(out_addr), .out_data(out_data),
        .found_l(found_l), .k_l(k_l), .bin_l(bin_l), .votes_l(votes_l),
        .found_r(found_r), .k_r(k_r), .bin_r(bin_r), .votes_r(votes_r),
        .n_edge(n_edge), .cyc_canny(cyc_canny), .cyc_hough(cyc_hough),
        .cyc_render(cyc_render), .cyc_total(cyc_total));

    // ---------------- OUT : output image, readable over JTAG ----------------
    jtag_sp_ram #(.DW(2), .AW(OAW), .DEPTH(NPIX), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=OUT")) u_out (
        .clk(clk), .we(out_we), .addr(out_addr), .wdata(out_data), .q());

    // ---------------- RES : 16 result words, readable over JTAG -------------
    reg        done_d;
    reg        res_we;
    reg [4:0]  res_idx;
    reg [31:0] res_word;
    always @(*) begin
        case (res_idx[3:0])
            4'd0:    res_word = 32'h4C414E45;                  // "LANE"
            4'd1:    res_word = (W << 16) | H;
            4'd2:    res_word = {found_l, 8'd0, k_l, bin_l};
            4'd3:    res_word = {16'd0, votes_l};
            4'd4:    res_word = {found_r, 8'd0, k_r, bin_r};
            4'd5:    res_word = {16'd0, votes_r};
            4'd6:    res_word = cyc_canny;
            4'd7:    res_word = cyc_hough;
            4'd8:    res_word = cyc_render;
            4'd9:    res_word = cyc_total;
            4'd10:   res_word = n_edge;
            default: res_word = 32'd0;
        endcase
    end
    always @(posedge clk) begin
        done_d <= done;
        if (rst) begin
            res_we  <= 1'b0;
            res_idx <= 5'd16;
        end else if (done & ~done_d) begin
            res_we  <= 1'b1;
            res_idx <= 5'd0;
        end else if (res_we) begin
            if (res_idx == 5'd15) res_we <= 1'b0;
            res_idx <= res_idx + 1'b1;
        end
    end
    jtag_sp_ram #(.DW(32), .AW(4), .DEPTH(16), .LPM_HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=RES")) u_res (
        .clk(clk), .we(res_we), .addr(res_idx[3:0]), .wdata(res_word), .q());

    // ---------------- VGA display (mirror of OUT) ----------------
    vga_display #(.W(W), .H(H)) u_vga (
        .clk(clk), .rst(rst),
        .wr_en(out_we), .wr_addr(out_addr), .wr_data(out_data),
        .VGA_CLK(VGA_CLK), .VGA_HS(VGA_HS), .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N), .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B));

    // ---------------- LEDs ----------------
    reg [25:0] hb;
    always @(posedge clk) hb <= hb + 1'b1;
    assign LEDG = {hb[25], 5'd0, found_r, found_l, done};
    assign LEDR = {17'd0, busy};
endmodule
