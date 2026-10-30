// ============================================================================
// hough_unit.v : Hough lane detection (paper section IV-B, Fig. 13, Alg. 1)
//
//  1. CLEAR : zero the (theta, rho) accumulator
//  2. SCAN  : raster-scan the edge memory, bottom half only (y >= ROI_Y)
//  3. ROT   : for every edge pixel, CORDIC-style shift-add rotation
//               X(k+1) = X(k) + (Y(k) >>> 5)      ("Adder")
//               Y(k+1) = Y(k) - (X(k) >>> 5)      ("Subtractor")
//             so X(k) = G^k * rho(theta_k),  theta_k = k * atan(2^-5)
//             (1.79 deg steps, k = 0..NTH-1 -> 0..179 deg).
//             rho = X(k) * (1/G^k)  (gain compensation LUT) -> vote +1
//             votes for 80 < theta < 100 deg (horizontal lines) are skipped
//  4. MAXS  : peak (strongest cell) of the left-lane region
//             (theta 17.9..75.2 deg) and of the right-lane region
//             (theta 103.8..161.1 deg) of the accumulator
//  5. SEL   : the accumulator is streamed again through a 3x3 window
//             (window_gen, like an image).  Candidates = 3x3 local maxima with
//             >= 75 % of the region peak.  The candidate line closest to the
//             car (bottom-centre point (0,YB)):  | rho - YB*sin(theta) |  min
//             is the ego-lane boundary  -> guard rails, road edges and
//             neighbouring lanes are rejected.
//
//  Origin of (x, y) is the image centre, so |rho| <= RHO_OFF.
// ============================================================================
module hough_unit #(
    parameter W = 320, parameter H = 240, parameter T = 4,
    parameter ROI_Y = 120,
    parameter NTH = 101, parameter NRHO = 401, parameter F = 10,
    parameter K_EXCL_LO = 45, parameter K_EXCL_HI = 55,
    parameter K_LEFT_MIN = 10, parameter K_LEFT_MAX = 42,
    parameter K_RIGHT_MIN = 58, parameter K_RIGHT_MAX = 90,
    parameter LANE_FRAC_NUM = 3, parameter YB = 120,
    parameter MIN_VOTES = 20, parameter ACC_BITS = 9,
    parameter EAW = $clog2((H/T)*W)          // edge memory address width
)(
    input                 clk,
    input                 rst,
    input                 start,
    output reg            done,          // 1-clock pulse
    // edge memory read port
    output     [EAW-1:0]  edge_raddr,
    input      [T-1:0]    edge_rdata,
    // results
    output reg            found_l,
    output reg [6:0]      k_l,
    output reg [15:0]     bin_l,
    output reg [15:0]     votes_l,
    output reg            found_r,
    output reg [6:0]      k_r,
    output reg [15:0]     bin_r,
    output reg [15:0]     votes_r,
    output reg [31:0]     n_edge
);
    localparam S       = H / T;
    localparam BD      = S * W;
    localparam NACC    = NTH * NRHO;
    localparam AAW     = $clog2(NACC);
    localparam RHO_OFF = (NRHO - 1) / 2;
    localparam RWD     = 24;                    // rotator register width
    localparam integer RNDI = 1 << (14 + F);    // rounding constant

    localparam ST_IDLE = 3'd0, ST_CLR = 3'd1, ST_SCAN = 3'd2, ST_ROT = 3'd3,
               ST_DRAIN = 3'd4, ST_MAXS = 3'd5, ST_FIN = 3'd6, ST_SEL = 3'd7;
    reg [2:0] st;

    // ------------------------------------------------------------ scan ----
    reg [9:0]      sx;          // next pixel to read
    reg [9:0]      sy;
    reg [EAW-1:0]  saddr;
    reg [7:0]      sbank;
    reg            d1_valid;    // pixel read last clock
    reg [9:0]      d1_x, d1_y;
    reg [7:0]      d1_bank;
    wire           hit = d1_valid & edge_rdata[d1_bank];
    assign edge_raddr = saddr;

    // --------------------------------------------------------- rotator ----
    reg signed [RWD-1:0] rx, ry;
    reg [6:0]            rk;

    // ------------------------------------------------- vote pipeline ----
    reg                   v0_valid;  reg [6:0] v0_k;  reg signed [RWD-1:0] v0_x;
    reg                   v1_valid;  reg [6:0] v1_k;  reg signed [RWD-1:0] v1_x;
    reg signed [17:0]     v1_g;
    reg                   v2_valid;  reg [6:0] v2_k;  reg signed [41:0] v2_p;
    reg                   v3_valid;  reg [6:0] v3_k;  reg signed [15:0] v3_rho;
    reg                   v4_valid;  reg [AAW-1:0] v4_addr;
    reg                   v5_valid;  reg [AAW-1:0] v5_addr;

    wire signed [17:0] ginv;
    wire signed [15:0] unused_c, unused_s;
    trig_lut u_lut (.k(v0_k), .ginv_q15(ginv), .cos_q14(unused_c), .sin_q14(unused_s));

    wire signed [41:0] psum    = v2_p + RNDI;
    wire signed [41:0] rho_all = psum >>> (15 + F);
    wire signed [15:0] bin_s   = v3_rho + RHO_OFF;
    wire               bin_ok  = (bin_s >= 0) && (bin_s < NRHO);

    // ------------------------------------------------------ accumulator ----
    reg                 acc_we;
    reg  [AAW-1:0]      acc_waddr;
    reg  [ACC_BITS-1:0] acc_wdata;
    wire [AAW-1:0]      acc_raddr;
    wire [ACC_BITS-1:0] acc_rdata;

    reg  [AAW-1:0] cnt;             // clear / max-search address
    reg  [6:0]     mk;              // max-search: theta index of cnt
    reg  [15:0]    mbin;            // max-search: rho bin of cnt
    reg            m1_valid;
    reg  [6:0]     m1_k;
    reg  [15:0]    m1_bin;
    reg  [3:0]     drain;

    assign acc_raddr = (st == ST_MAXS || st == ST_SEL) ? cnt : v4_addr;

    sdp_ram #(.DW(ACC_BITS), .AW(AAW), .DEPTH(NACC)) u_acc (
        .clk(clk), .we(acc_we), .waddr(acc_waddr), .wdata(acc_wdata),
        .raddr(acc_raddr), .rdata(acc_rdata));

    // accumulator write mux: clear  or  read-modify-write (+1, saturating)
    always @(*) begin
        if (st == ST_CLR) begin
            acc_we    = 1'b1;
            acc_waddr = cnt;
            acc_wdata = {ACC_BITS{1'b0}};
        end else begin
            acc_we    = v5_valid;
            acc_waddr = v5_addr;
            acc_wdata = (&acc_rdata) ? acc_rdata : acc_rdata + 1'b1;   // saturate
        end
    end

    // ---------------------------------------------------- vote pipeline ----
    always @(posedge clk) begin
        v1_valid <= rst ? 1'b0 : v0_valid;  v1_k <= v0_k;  v1_x <= v0_x;  v1_g <= ginv;
        v2_valid <= rst ? 1'b0 : v1_valid;  v2_k <= v1_k;  v2_p <= v1_x * v1_g;
        v3_valid <= rst ? 1'b0 : v2_valid;  v3_k <= v2_k;  v3_rho <= rho_all[15:0];
        v4_valid <= rst ? 1'b0 : (v3_valid & bin_ok);
        v4_addr  <= v3_k * NRHO + bin_s[AAW-1:0];
        v5_valid <= rst ? 1'b0 : v4_valid;  v5_addr <= v4_addr;
    end

    // ------------------------------------------ lane selection (SEL) ----
    reg  [15:0] pv_l, pv_r, pb_l, pb_r;      // region peaks (MAXS)
    reg  [6:0]  pk_l, pk_r;
    reg         sel_clr;
    reg         q1_valid;                    // accumulator word read last clock
    always @(posedge clk) q1_valid <= (st == ST_SEL) && (cnt < NACC);

    wire        aw_valid;
    wire [80:0] aw;                          // 3x3 x 9-bit window of the accumulator
    window_gen #(.DW(ACC_BITS), .K(3), .W(NRHO), .AW($clog2(NRHO))) u_accwin (
        .clk(clk), .rst(rst | sel_clr), .in_valid(q1_valid), .in_data(acc_rdata),
        .out_valid(aw_valid), .win(aw));

    // window centre position = stream position - (NRHO+1)
    reg signed [8:0]  wk;                    // theta index of the window centre
    reg        [9:0]  wb;                    // rho bin of the window centre
    reg        [AAW:0] wcnt;
    `define AWE(i) aw[(i)*ACC_BITS +: ACC_BITS]
    wire [ACC_BITS-1:0] wv = `AWE(4);
    wire lmax = (wv >= `AWE(0)) && (wv >= `AWE(1)) && (wv >= `AWE(2)) && (wv >= `AWE(3)) &&
                (wv >= `AWE(5)) && (wv >= `AWE(6)) && (wv >= `AWE(7)) && (wv >= `AWE(8));
    `undef AWE
    wire in_l  = (wk >= K_LEFT_MIN)  && (wk <= K_LEFT_MAX);
    wire in_r  = (wk >= K_RIGHT_MIN) && (wk <= K_RIGHT_MAX);
    wire b_ok  = (wb >= 1) && (wb <= NRHO - 2);
    // votes >= 75 % of the region peak :  4*v >= 3*peak
    wire [18:0] wv4  = {8'd0, wv, 2'b00};
    wire thr_l = (wv4 >= LANE_FRAC_NUM * pv_l);
    wire thr_r = (wv4 >= LANE_FRAC_NUM * pv_r);
    wire cand  = b_ok && lmax && (wv >= MIN_VOTES) && ((in_l && thr_l) || (in_r && thr_r));

    // s1: candidate registered
    reg        s1_v, s1_l;  reg [6:0] s1_k;  reg [9:0] s1_b;  reg [ACC_BITS-1:0] s1_n;
    // s2: distance to the car  d = | (rho << 14) - YB*sin(theta) |
    wire signed [17:0] s_g;  wire signed [15:0] s_c, s_s;
    trig_lut u_lut_sel (.k(s1_k), .ginv_q15(s_g), .cos_q14(s_c), .sin_q14(s_s));
    wire signed [27:0] s_dd = (($signed({1'b0, s1_b}) - RHO_OFF) <<< 14) - YB * s_s;
    reg        s2_v, s2_l;  reg [6:0] s2_k;  reg [9:0] s2_b;  reg [ACC_BITS-1:0] s2_n;
    reg [27:0] s2_d;
    // s3: keep the closest candidate per region
    reg        hv_l, hv_r;
    reg [27:0] bd_l, bd_r;
    reg [6:0]  sk_l, sk_r;
    reg [15:0] sb_l, sb_r, sv_l, sv_r;
    reg [2:0]  s_pipe;
    reg        s3_done;

    always @(posedge clk) begin
        s3_done <= 1'b0;
        if (rst || sel_clr) begin
            wk <= -9'sd2;  wb <= NRHO - 1;  wcnt <= 0;
            s1_v <= 1'b0;  s2_v <= 1'b0;  s_pipe <= 3'd0;
            hv_l <= 1'b0;  hv_r <= 1'b0;
            // fall-back = region peak
            sk_l <= pk_l;  sb_l <= pb_l;  sv_l <= pv_l;
            sk_r <= pk_r;  sb_r <= pb_r;  sv_r <= pv_r;
        end else begin
            s1_v <= aw_valid && cand;
            s1_l <= in_l;  s1_k <= wk[6:0];  s1_b <= wb;  s1_n <= wv;
            if (aw_valid) begin
                wcnt <= wcnt + 1'b1;
                if (wb == NRHO - 1) begin wb <= 10'd0; wk <= wk + 1'b1; end
                else                  wb <= wb + 1'b1;
            end
            s2_v <= s1_v;  s2_l <= s1_l;  s2_k <= s1_k;  s2_b <= s1_b;  s2_n <= s1_n;
            s2_d <= s_dd[27] ? -s_dd : s_dd;
            if (s2_v) begin
                if (s2_l) begin
                    if (!hv_l || s2_d < bd_l) begin
                        hv_l <= 1'b1; bd_l <= s2_d; sk_l <= s2_k; sb_l <= s2_b; sv_l <= s2_n;
                    end
                end else begin
                    if (!hv_r || s2_d < bd_r) begin
                        hv_r <= 1'b1; bd_r <= s2_d; sk_r <= s2_k; sb_r <= s2_b; sv_r <= s2_n;
                    end
                end
            end
            // finished when all NACC window outputs are through s1..s3
            if (wcnt == NACC) begin
                if (s_pipe != 3'd4) s_pipe <= s_pipe + 1'b1;
                if (s_pipe == 3'd3) s3_done <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------- FSM ----
    always @(posedge clk) begin
        done     <= 1'b0;
        v0_valid <= 1'b0;
        sel_clr  <= 1'b0;
        if (rst) begin
            st <= ST_IDLE;
            d1_valid <= 1'b0;
            m1_valid <= 1'b0;
        end else begin
            case (st)
            ST_IDLE: if (start) begin
                st  <= ST_CLR;
                cnt <= {AAW{1'b0}};
            end

            ST_CLR: begin
                if (cnt == NACC - 1) begin
                    st       <= ST_SCAN;
                    sx       <= 10'd0;
                    sy       <= ROI_Y;
                    saddr    <= (ROI_Y % S) * W;
                    sbank    <= ROI_Y / S;
                    d1_valid <= 1'b0;
                    n_edge   <= 32'd0;
                end
                cnt <= cnt + 1'b1;
            end

            ST_SCAN: begin
                if (hit) begin
                    // edge pixel found -> rotate (scan pointer is held)
                    st       <= ST_ROT;
                    rx       <= ($signed({1'b0, d1_x}) - (W/2)) <<< F;
                    ry       <= ($signed({1'b0, d1_y}) - (H/2)) <<< F;
                    rk       <= 7'd0;
                    d1_valid <= 1'b0;
                    n_edge   <= n_edge + 1'b1;
                end else if (sy < H) begin
                    d1_valid <= 1'b1;
                    d1_x     <= sx;
                    d1_y     <= sy;
                    d1_bank  <= sbank;
                    if (sx == W-1) begin sx <= 10'd0; sy <= sy + 1'b1; end
                    else            sx <= sx + 1'b1;
                    if (saddr == BD-1) begin saddr <= {EAW{1'b0}}; sbank <= sbank + 1'b1; end
                    else               saddr <= saddr + 1'b1;
                end else begin
                    d1_valid <= 1'b0;
                    if (!d1_valid) begin st <= ST_DRAIN; drain <= 4'd0; end
                end
            end

            ST_ROT: begin
                v0_valid <= !((rk >= K_EXCL_LO) && (rk <= K_EXCL_HI));
                v0_k     <= rk;
                v0_x     <= rx;
                rx       <= rx + (ry >>> 5);
                ry       <= ry - (rx >>> 5);
                rk       <= rk + 1'b1;
                if (rk == NTH - 1) st <= ST_SCAN;
            end

            ST_DRAIN: begin                      // let the vote pipeline finish
                drain <= drain + 1'b1;
                if (drain == 4'd10) begin
                    st      <= ST_MAXS;
                    cnt     <= {AAW{1'b0}};
                    mk      <= 7'd0;
                    mbin    <= 16'd0;
                    pv_l <= 16'd0;  pk_l <= 7'd0;  pb_l <= 16'd0;
                    pv_r <= 16'd0;  pk_r <= 7'd0;  pb_r <= 16'd0;
                end
            end

            ST_MAXS: begin
                // issue read of cnt; compare the word read last clock
                m1_valid <= (cnt < NACC);
                m1_k     <= mk;
                m1_bin   <= mbin;
                if (cnt < NACC) begin
                    cnt <= cnt + 1'b1;
                    if (mbin == NRHO - 1) begin mbin <= 16'd0; mk <= mk + 1'b1; end
                    else                  mbin <= mbin + 1'b1;
                end
                if (m1_valid) begin
                    if (m1_k >= K_LEFT_MIN && m1_k <= K_LEFT_MAX) begin
                        if (acc_rdata > pv_l) begin
                            pv_l <= acc_rdata; pk_l <= m1_k; pb_l <= m1_bin;
                        end
                    end else if (m1_k >= K_RIGHT_MIN && m1_k <= K_RIGHT_MAX) begin
                        if (acc_rdata > pv_r) begin
                            pv_r <= acc_rdata; pk_r <= m1_k; pb_r <= m1_bin;
                        end
                    end
                end
                if (cnt == NACC && !m1_valid) begin
                    st      <= ST_SEL;
                    cnt     <= {AAW{1'b0}};
                    sel_clr <= 1'b1;
                end
            end

            ST_SEL: begin
                // stream the accumulator into the 3x3 window (see below)
                if (cnt < NACC) cnt <= cnt + 1'b1;
                if (s3_done) st <= ST_FIN;
            end

            ST_FIN: begin
                found_l <= (pv_l >= MIN_VOTES);
                found_r <= (pv_r >= MIN_VOTES);
                k_l <= sk_l;  bin_l <= sb_l;  votes_l <= sv_l;
                k_r <= sk_r;  bin_r <= sb_r;  votes_r <= sv_r;
                done    <= 1'b1;
                st      <= ST_IDLE;
            end

            default: st <= ST_IDLE;
            endcase
        end
    end
endmodule
