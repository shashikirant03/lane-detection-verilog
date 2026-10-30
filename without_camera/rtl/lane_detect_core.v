// ============================================================================
// lane_detect_core.v
//   Low-power lane detection unit with sliding-based parallel segment
//   detection (H. Yun, D. Park, IEEE Access 2024) -- Verilog implementation
//
//   Canny edge detection  : T parallel engines ("threads", Fig. 7), each
//                           owns S = H/T image rows (a "segment") plus
//                           HALO overlapping rows above/below it.
//   Hough lane detection  : sequential (as in the paper), CORDIC rotation.
//   Post-processing       : lane lines drawn in hardware.
//
// Memory interface:
//   The input image lives in T ROM banks (bank j = rows j*S .. j*S+S-1),
//   24-bit RGB, 1-clock read latency, all banks share ONE address bus.
//   Sliding trick: while all engines step through local rows
//   r = 0 .. S+2*HALO-1, every bank reads the SAME address
//   ((r-HALO) mod S)*W + c, and engine j takes its data from
//       bank j-1 (r < HALO, top halo), bank j (own rows), bank j+1 (bottom
//       halo).  No bank is ever needed by two engines at the same time, so
//   the T engines run in lock-step without any memory conflicts and without
//   storing the overlap rows twice.
//
// Output interface: one 2-bit word per pixel (bit1 = lane, bit0 = edge),
//   written in raster order (out_addr = y*W + x) during the render phase.
// ============================================================================
module lane_detect_core #(
    parameter W = 320,
    parameter H = 240,
    parameter T = 4,
    parameter HALO = 5,
    parameter BORDER = 5,
    parameter TH_HIGH = 160,
    parameter TH_LOW = 80,
    parameter ROI_Y = 120,
    parameter NTH = 101,
    parameter NRHO = 401,
    parameter F = 10,
    parameter K_EXCL_LO = 45,
    parameter K_EXCL_HI = 55,
    parameter K_LEFT_MIN = 10,
    parameter K_LEFT_MAX = 42,
    parameter K_RIGHT_MIN = 58,
    parameter K_RIGHT_MAX = 90,
    parameter LANE_FRAC_NUM = 3,
    parameter YB = 120,
    parameter EXT_TOP = 72,
    parameter MAX_GAP = 12,
    parameter HIT_TOL_Q14 = 24576,
    parameter GAP_Q14 = 196608,
    parameter MIN_VOTES = 20,
    parameter ACC_BITS = 9,
    parameter LINE_THICK_Q14 = 16384,
    // derived (do not override)
    parameter BAW = $clog2((H/T)*W),     // bank / edge-memory address width
    parameter OAW = $clog2(W*H),         // output address width
    parameter XAW = $clog2(W)            // column address width
)(
    input                  clk,
    input                  rst,
    input                  start,        // 1-clock pulse
    output                 busy,
    output reg             done,         // level, cleared by start
    // input image ROM banks
    output reg [BAW-1:0]   rom_addr,
    input      [T*24-1:0]  rom_data,     // bank j at [j*24 +: 24]
    // output image
    output                 out_we,
    output     [OAW-1:0]   out_addr,
    output     [1:0]       out_data,
    // results
    output                 found_l,
    output     [6:0]       k_l,
    output     [15:0]      bin_l,
    output     [15:0]      votes_l,
    output                 found_r,
    output     [6:0]       k_r,
    output     [15:0]      bin_r,
    output     [15:0]      votes_r,
    output     [31:0]      n_edge,
    output reg [31:0]      cyc_canny,
    output reg [31:0]      cyc_hough,
    output reg [31:0]      cyc_render,
    output reg [31:0]      cyc_total
);
    localparam S     = H / T;
    localparam BD    = S * W;
    localparam LROWS = S + 2 * HALO;
    localparam NPIX  = LROWS * W;           // pixels streamed per engine

    localparam ST_IDLE = 3'd0, ST_CANNY = 3'd1, ST_HOUGH = 3'd2,
               ST_RENDER = 3'd3, ST_DONE = 3'd4;
    reg [2:0] st;
    assign busy = (st != ST_IDLE) && (st != ST_DONE);

    wire go = start && !busy;
    reg  eng_clr;                           // resets the engines' counters

    // =====================================================================
    // 1) Sliding read of the T banks
    // =====================================================================
    reg [31:0]  rd_cnt;
    reg [9:0]   rd_c;
    reg [9:0]   rd_r;                       // local row 0 .. LROWS-1
    reg         rd_on;
    wire [1:0]  rd_phase = (rd_r < HALO) ? 2'd0 : (rd_r < S + HALO) ? 2'd1 : 2'd2;

    reg         p1_valid;
    reg [1:0]   p1_phase;
    reg         e_valid;
    reg [T*24-1:0] e_rgb;

    integer j;
    always @(posedge clk) begin
        p1_valid <= rst ? 1'b0 : rd_on;
        p1_phase <= rd_phase;
        e_valid  <= rst ? 1'b0 : p1_valid;
        for (j = 0; j < T; j = j + 1) begin
            case (p1_phase)
            2'd0:    e_rgb[j*24 +: 24] <= (j == 0)   ? 24'd0 : rom_data[((j+T-1)%T)*24 +: 24];
            2'd1:    e_rgb[j*24 +: 24] <= rom_data[j*24 +: 24];
            default: e_rgb[j*24 +: 24] <= (j == T-1) ? 24'd0 : rom_data[((j+1)%T)*24 +: 24];
            endcase
        end
    end

    // =====================================================================
    // 2) T parallel Canny engines
    // =====================================================================
    wire [T-1:0] eo_valid;
    wire [T-1:0] eo_edge;
    genvar g;
    generate
        for (g = 0; g < T; g = g + 1) begin : g_eng
            canny_engine #(.W(W), .AW(XAW), .TH_HIGH(TH_HIGH), .TH_LOW(TH_LOW)) u_eng (
                .clk(clk), .rst(rst | eng_clr),
                .in_valid(e_valid), .in_rgb(e_rgb[g*24 +: 24]),
                .out_valid(eo_valid[g]), .out_edge(eo_edge[g]));
        end
    endgenerate

    // =====================================================================
    // 3) Collect engine outputs -> edge memory (T bits per word)
    //    output #i of an engine belongs to stream position i - (5W+5)
    // =====================================================================
    reg signed [11:0] o_x;
    reg signed [11:0] o_ly;
    reg [31:0]        o_cnt;
    reg [BAW-1:0]     ew_addr;
    reg               ew_we;
    reg [BAW-1:0]     ew_waddr;
    reg [T-1:0]       ew_data;

    wire o_rows = (o_ly >= HALO) && (o_ly < HALO + S);
    wire o_colb = (o_x < BORDER) || (o_x >= W - BORDER);
    reg  [T-1:0] o_bits;
    integer b;
    always @(*) begin
        for (b = 0; b < T; b = b + 1) begin
            // global row of this engine's output: b*S + o_ly - HALO
            o_bits[b] = eo_edge[b] & ~o_colb &
                        ~((b*S + o_ly - HALO) < BORDER) &
                        ~((b*S + o_ly - HALO) >= H - BORDER);
        end
    end

    always @(posedge clk) begin
        ew_we <= 1'b0;
        if (rst || go) begin
            o_x     <= W - 5;               // position -(5W+5) = row -6, col W-5
            o_ly    <= -12'sd6;
            o_cnt   <= 32'd0;
            ew_addr <= {BAW{1'b0}};
        end else if (eo_valid[0]) begin
            o_cnt <= o_cnt + 1'b1;
            if (o_rows) begin
                ew_we    <= 1'b1;
                ew_waddr <= ew_addr;
                ew_data  <= o_bits;
                ew_addr  <= ew_addr + 1'b1;
            end
            if (o_x == W - 1) begin o_x <= 12'sd0; o_ly <= o_ly + 1'b1; end
            else                     o_x <= o_x + 1'b1;
        end
    end

    // edge memory: written by Canny, read by Hough and Render
    wire [BAW-1:0] h_raddr, r_raddr;
    wire [T-1:0]   edge_rdata;
    sdp_ram #(.DW(T), .AW(BAW), .DEPTH(BD)) u_edge (
        .clk(clk), .we(ew_we), .waddr(ew_waddr), .wdata(ew_data),
        .raddr((st == ST_RENDER) ? r_raddr : h_raddr), .rdata(edge_rdata));

    // =====================================================================
    // 4) Hough + 5) Render
    // =====================================================================
    reg  h_start, r_start;
    wire h_done,  r_done;

    hough_unit #(.W(W), .H(H), .T(T), .ROI_Y(ROI_Y), .NTH(NTH), .NRHO(NRHO), .F(F),
                 .K_EXCL_LO(K_EXCL_LO), .K_EXCL_HI(K_EXCL_HI),
                 .K_LEFT_MIN(K_LEFT_MIN), .K_LEFT_MAX(K_LEFT_MAX),
                 .K_RIGHT_MIN(K_RIGHT_MIN), .K_RIGHT_MAX(K_RIGHT_MAX),
                 .LANE_FRAC_NUM(LANE_FRAC_NUM), .YB(YB),
                 .MIN_VOTES(MIN_VOTES), .ACC_BITS(ACC_BITS), .EAW(BAW)) u_hough (
        .clk(clk), .rst(rst), .start(h_start), .done(h_done),
        .edge_raddr(h_raddr), .edge_rdata(edge_rdata),
        .found_l(found_l), .k_l(k_l), .bin_l(bin_l), .votes_l(votes_l),
        .found_r(found_r), .k_r(k_r), .bin_r(bin_r), .votes_r(votes_r),
        .n_edge(n_edge));

    render_unit #(.W(W), .H(H), .T(T), .ROI_Y(ROI_Y), .NRHO(NRHO),
                  .EXT_TOP(EXT_TOP), .MAX_GAP(MAX_GAP), .YB(YB),
                  .HIT_TOL_Q14(HIT_TOL_Q14), .GAP_Q14(GAP_Q14),
                  .LINE_THICK_Q14(LINE_THICK_Q14), .EAW(BAW), .OAW(OAW)) u_render (
        .clk(clk), .rst(rst), .start(r_start), .done(r_done),
        .edge_raddr(r_raddr), .edge_rdata(edge_rdata),
        .found_l(found_l), .k_l(k_l), .bin_l(bin_l),
        .found_r(found_r), .k_r(k_r), .bin_r(bin_r),
        .out_we(out_we), .out_addr(out_addr), .out_data(out_data));

    // =====================================================================
    // Top-level sequencer
    // =====================================================================
    always @(posedge clk) begin
        eng_clr <= 1'b0;
        h_start <= 1'b0;
        r_start <= 1'b0;
        if (rst) begin
            st    <= ST_IDLE;
            rd_on <= 1'b0;
            done  <= 1'b0;
        end else begin
            if (busy) cyc_total <= cyc_total + 1'b1;
            case (st)
            ST_IDLE, ST_DONE: if (go) begin
                st         <= ST_CANNY;
                done       <= 1'b0;
                eng_clr    <= 1'b1;
                rd_on      <= 1'b1;
                rd_cnt     <= 32'd0;
                rd_c       <= 10'd0;
                rd_r       <= 10'd0;
                rom_addr   <= (S - HALO) * W;       // (r-HALO) mod S at r=0
                cyc_canny  <= 32'd0;
                cyc_hough  <= 32'd0;
                cyc_render <= 32'd0;
                cyc_total  <= 32'd0;
            end

            ST_CANNY: begin
                cyc_canny <= cyc_canny + 1'b1;
                if (rd_on) begin
                    rd_cnt   <= rd_cnt + 1'b1;
                    rom_addr <= (rom_addr == BD - 1) ? {BAW{1'b0}} : rom_addr + 1'b1;
                    if (rd_c == W - 1) begin rd_c <= 10'd0; rd_r <= rd_r + 1'b1; end
                    else                     rd_c <= rd_c + 1'b1;
                    if (rd_cnt == NPIX - 1) rd_on <= 1'b0;
                end
                if (o_cnt == NPIX) begin              // all engine outputs seen
                    st      <= ST_HOUGH;
                    h_start <= 1'b1;
                end
            end

            ST_HOUGH: begin
                cyc_hough <= cyc_hough + 1'b1;
                if (h_done) begin st <= ST_RENDER; r_start <= 1'b1; end
            end

            ST_RENDER: begin
                cyc_render <= cyc_render + 1'b1;
                if (r_done) begin st <= ST_DONE; done <= 1'b1; end
            end

            default: st <= ST_IDLE;
            endcase
        end
    end
endmodule
