// ============================================================================
// de2_115_support.v : board-level helper modules for de2_115_lane_top
//   img_bank_rom : 24-bit image ROM (altsyncram) with In-System Memory
//                  Content Editor access, initialised from a .mif
//   jtag_sp_ram  : single-port RAM (altsyncram) with In-System Memory
//                  Content Editor access (used to read results over JTAG)
//   vga_display  : 640x480@60 Hz VGA output of the 2-bit result image (2x zoom)
//
// Define BEHAVIORAL_MEM (e.g. vlog +define+BEHAVIORAL_MEM) to replace the
// altsyncram primitives by plain Verilog models (lint / simulation without
// the altera_mf library).
// ============================================================================

// ---------------------------------------------------------------------------
module img_bank_rom #(
    parameter INIT_FILE = "img_bank0.mif",
    parameter LPM_HINT  = "ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=IB0",
    parameter DEPTH     = 19200,
    parameter AW        = 15
)(
    input           clk,
    input  [AW-1:0] addr,
    output [23:0]   q
);
`ifdef BEHAVIORAL_MEM
    reg [23:0] mem [0:DEPTH-1];
    reg [23:0] qr;
    integer i;
    initial for (i = 0; i < DEPTH; i = i + 1) mem[i] = 24'd0;
    always @(posedge clk) qr <= mem[addr];
    assign q = qr;
`else
    altsyncram #(
        .operation_mode         ("ROM"),
        .width_a                (24),
        .widthad_a              (AW),
        .numwords_a             (DEPTH),
        .outdata_reg_a          ("UNREGISTERED"),
        .address_aclr_a         ("NONE"),
        .outdata_aclr_a         ("NONE"),
        .clock_enable_input_a   ("BYPASS"),
        .clock_enable_output_a  ("BYPASS"),
        .init_file              (INIT_FILE),
        .intended_device_family ("Cyclone IV E"),
        .lpm_hint               (LPM_HINT),
        .lpm_type               ("altsyncram"),
        .width_byteena_a        (1)
    ) u_rom (
        .clock0         (clk),
        .address_a      (addr),
        .q_a            (q),
        .aclr0          (1'b0),
        .aclr1          (1'b0),
        .address_b      (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0),
        .byteena_a      (1'b1),
        .byteena_b      (1'b1),
        .clock1         (1'b1),
        .clocken0       (1'b1),
        .clocken1       (1'b1),
        .clocken2       (1'b1),
        .clocken3       (1'b1),
        .data_a         ({24{1'b1}}),
        .data_b         (1'b1),
        .eccstatus      (),
        .q_b            (),
        .rden_a         (1'b1),
        .rden_b         (1'b1),
        .wren_a         (1'b0),
        .wren_b         (1'b0)
    );
`endif
endmodule

// ---------------------------------------------------------------------------
module jtag_sp_ram #(
    parameter DW    = 2,
    parameter AW    = 17,
    parameter DEPTH = 76800,
    parameter LPM_HINT = "ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=OUT"
)(
    input           clk,
    input           we,
    input  [AW-1:0] addr,
    input  [DW-1:0] wdata,
    output [DW-1:0] q
);
`ifdef BEHAVIORAL_MEM
    reg [DW-1:0] mem [0:DEPTH-1];
    reg [DW-1:0] qr;
    integer i;
    initial for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DW{1'b0}};
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        qr <= mem[addr];
    end
    assign q = qr;
`else
    altsyncram #(
        .operation_mode                ("SINGLE_PORT"),
        .width_a                       (DW),
        .widthad_a                     (AW),
        .numwords_a                    (DEPTH),
        .outdata_reg_a                 ("UNREGISTERED"),
        .address_aclr_a                ("NONE"),
        .outdata_aclr_a                ("NONE"),
        .clock_enable_input_a          ("BYPASS"),
        .clock_enable_output_a         ("BYPASS"),
        .intended_device_family        ("Cyclone IV E"),
        .lpm_hint                      (LPM_HINT),
        .lpm_type                      ("altsyncram"),
        .power_up_uninitialized        ("FALSE"),
        .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .width_byteena_a               (1)
    ) u_ram (
        .clock0         (clk),
        .address_a      (addr),
        .data_a         (wdata),
        .wren_a         (we),
        .q_a            (q),
        .aclr0          (1'b0),
        .aclr1          (1'b0),
        .address_b      (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0),
        .byteena_a      (1'b1),
        .byteena_b      (1'b1),
        .clock1         (1'b1),
        .clocken0       (1'b1),
        .clocken1       (1'b1),
        .clocken2       (1'b1),
        .clocken3       (1'b1),
        .data_b         (1'b1),
        .eccstatus      (),
        .q_b            (),
        .rden_a         (1'b1),
        .rden_b         (1'b1),
        .wren_b         (1'b0)
    );
`endif
endmodule

// ---------------------------------------------------------------------------
// 640x480 @ 60 Hz (25 MHz pixel clock derived from CLOCK_50 with an enable).
// VGA_CLK rises in the middle of every pixel so the ADV7123 samples stable
// data.  The W x H result is shown 2x zoomed, top-left aligned.
// ---------------------------------------------------------------------------
module vga_display #(parameter W = 320, parameter H = 240) (
    input             clk,
    input             rst,
    input             wr_en,
    input  [$clog2(W*H)-1:0] wr_addr,
    input  [1:0]      wr_data,
    output            VGA_CLK,
    output reg        VGA_HS,
    output reg        VGA_VS,
    output reg        VGA_BLANK_N,
    output            VGA_SYNC_N,
    output reg [7:0]  VGA_R,
    output reg [7:0]  VGA_G,
    output reg [7:0]  VGA_B
);
    localparam AW = $clog2(W*H);
    localparam H_VIS = 640, H_FP = 16, H_SY = 96, H_BP = 48, H_TOT = 800;
    localparam V_VIS = 480, V_FP = 10, V_SY = 2,  V_BP = 33, V_TOT = 525;

    // frame buffer (mirror of the OUT memory)
    reg  [AW-1:0] rd_addr;
    wire [1:0]    pix;
    sdp_ram #(.DW(2), .AW(AW), .DEPTH(W*H)) u_fb (
        .clk(clk), .we(wr_en), .waddr(wr_addr), .wdata(wr_data),
        .raddr(rd_addr), .rdata(pix));

    reg pe;                       // pixel enable (25 MHz)
    reg vclk;
    always @(posedge clk) begin
        pe   <= rst ? 1'b0 : ~pe;
        vclk <= ~pe;
    end
    assign VGA_CLK    = vclk;
    assign VGA_SYNC_N = 1'b0;

    reg [9:0] hc, vc;
    reg       hs1, vs1, act1, img1;
    wire [8:0] zx = hc[9:1];
    wire [8:0] zy = vc[9:1];
    always @(posedge clk) begin
        if (rst) begin
            hc <= 10'd0; vc <= 10'd0;
        end else if (pe) begin
            // stage 1 : counters -> frame-buffer address, sync
            if (hc == H_TOT - 1) begin
                hc <= 10'd0;
                vc <= (vc == V_TOT - 1) ? 10'd0 : vc + 1'b1;
            end else hc <= hc + 1'b1;
            hs1  <= ~((hc >= H_VIS + H_FP) && (hc < H_VIS + H_FP + H_SY));
            vs1  <= ~((vc >= V_VIS + V_FP) && (vc < V_VIS + V_FP + V_SY));
            act1 <= (hc < H_VIS) && (vc < V_VIS);
            img1 <= (zx < W) && (zy < H);
            rd_addr <= zy * W + zx;
            // stage 2 : colour (frame-buffer data read one 50 MHz clock ago)
            VGA_HS      <= hs1;
            VGA_VS      <= vs1;
            VGA_BLANK_N <= act1;
            if (act1 && img1) begin
                if (pix[1])      {VGA_R, VGA_G, VGA_B} <= {8'hFF, 8'h00, 8'h00};  // lane
                else if (pix[0]) {VGA_R, VGA_G, VGA_B} <= {8'hFF, 8'hFF, 8'hFF};  // edge
                else             {VGA_R, VGA_G, VGA_B} <= {8'h10, 8'h10, 8'h10};
            end else             {VGA_R, VGA_G, VGA_B} <= 24'd0;
        end
    end
endmodule
