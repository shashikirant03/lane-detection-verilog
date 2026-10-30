// ============================================================================
// hough_transform.v  -  paper Sec. IV-B (Hough transform, local maxima,
//                       optimisation) with the CORDIC unit of Fig. 13
//
// 1) HOUGH TRANSFORM  (eq. 7:  x cos(theta) + y sin(theta) = rho)
//    For every edge pixel (x, y) of the edge list:
//       load the CORDIC with (x - 160, y - 120)      (origin = image centre)
//       for i = 0 .. 100  (theta = i * 1.79 deg, 0 .. 179 deg):
//           rho = round( x(i) )                      (cordic_rotator output)
//           Accum[i][rho] = Accum[i][rho] + 1        ("each edge pixel
//                                                     contributes +1")
//           rotate the CORDIC one step
//    One theta per clock.  The accumulator is a block RAM of
//    101 (theta) x 512 (rho, offset +256) cells of 9 bits.
//
// 2) OPTIMISATION (paper Sec. IV-B-3)
//    a) only the bottom half of the image is voted  (done in lane_detection.v)
//    b) horizontal lines are removed: 80 < theta < 100 deg is not voted
//       (i = 45..55)
//
// 3) LOCAL MAXIMA OF THE LANE (paper Sec. IV-B-2)
//    "Only the highest accumulator value is considered when several high
//     accumulator values are nearby, and adjacent values are removed ...
//     lane markings are far enough apart."
//    The accumulator is split into the two lane regions:
//       left lane  : theta = i 3..40   (5 .. 72 deg,   a "/" line)
//       right lane : theta = i 60..97  (107 .. 174 deg, a "\" line)
//    and in each region the highest cell is taken.
//
//    peak_mode = 0 (default) : the highest cell of each region is found
//        while the accumulator is scanned and cleared for the next frame
//        (every cell is read once anyway, so this costs no extra time).
//    peak_mode = 1 : Algorithm 1 (divide & conquer, local_maxima.v), as
//        written in the paper.  It reads only ~4N cells, but it returns the
//        first local peak it climbs to, which on hand-drawn images is often
//        a small side peak instead of the lane -> mode 0 is the default.
//
//    A lane is accepted when its cell has at least MIN_VOTES votes.
//
// 4) CLEAR : every cell is written back to 0 for the next frame.
// ============================================================================
module hough_transform #(
    parameter F         = 10,     // fractional bits of the CORDIC registers
    parameter NT        = 101,    // number of theta steps
    parameter SKIP_LO   = 45,     // 80.5 deg  : first theta not voted
    parameter SKIP_HI   = 55,     // 98.4 deg  : last  theta not voted
    parameter [6:0] LEFT_T0  = 3,
    parameter [6:0] LEFT_T1  = 40,
    parameter [6:0] RIGHT_T0 = 60,
    parameter [6:0] RIGHT_T1 = 97,
    parameter [8:0] RHO_LO   = 40,     // rho search range (bin = rho + 256)
    parameter [8:0] RHO_HI   = 472,
    parameter MIN_VOTES = 24
)(
    input              clk,
    input              rst,
    input              peak_mode,      // 0 = highest cell (scan), 1 = Algorithm 1
    input              start,          // pulse: edge list of a frame is ready
    input      [12:0]  n_edges,        // number of edge pixels
    output reg [12:0]  e_idx,          // edge list read address
    input      [8:0]   e_x,            // edge list data (1 clock after e_idx)
    input      [7:0]   e_y,
    output             busy,
    output reg         done,           // pulse: new lane result
    output reg         left_ok,
    output reg [6:0]   left_t,
    output reg [8:0]   left_r,
    output reg         right_ok,
    output reg [6:0]   right_t,
    output reg [8:0]   right_r
);
    localparam ACC_SIZE = NT * 512;
    localparam [6:0] T_LAST = 7'd100;        // last theta index = NT - 1

    localparam CLEAR  = 4'd0, IDLE   = 4'd1, LOAD   = 4'd2, LOAD2  = 4'd3,
               VOTE   = 4'd4, PEAK_L = 4'd5, PEAK_R = 4'd6, SCAN   = 4'd7,
               SCAN_W = 4'd8, RESULT = 4'd9;

    reg [3:0]  state;
    reg        mode;                     // peak_mode of this frame
    reg [12:0] n_r;
    reg [15:0] clr_addr;
    reg [6:0]  t;                        // theta index i

    // ------------------------------------------------ CORDIC (Fig. 13)
    // loaded in state LOAD2 with the edge pixel relative to the image centre
    wire                c_load = (state == LOAD2);
    wire signed [21:0]  c_x0   = ($signed({2'b00, e_x}) - 11'sd160) <<< F;
    wire signed [21:0]  c_y0   = ($signed({3'b000, e_y}) - 11'sd120) <<< F;
    wire signed [21:0]  cx, cy;

    cordic_rotator #(.W(22)) u_cordic (
        .clk(clk), .load(c_load), .step(state == VOTE),
        .x_in(c_x0), .y_in(c_y0), .x(cx), .y(cy)
    );

    wire signed [21:0] cx_round = cx + (22'sd1 <<< (F-1));
    wire signed [21:0] rho      = cx_round >>> F;          // round(x(i))
    wire        [8:0]  rho_bin  = rho[8:0] + 9'd256;        // -256..255 -> 0..511
    wire               skip     = (t >= SKIP_LO) && (t <= SKIP_HI);

    // ------------------------------------------------ accumulator RAM
    reg         acc_we;
    reg  [15:0] acc_waddr, acc_raddr;
    reg  [8:0]  acc_wdata;
    wire [8:0]  acc_q;

    dual_port_ram #(.DW(9), .AW(16), .DEPTH(ACC_SIZE)) u_accum (
        .wclk(clk), .we(acc_we), .waddr(acc_waddr), .wdata(acc_wdata),
        .rclk(clk), .raddr(acc_raddr), .rdata(acc_q)
    );

    // vote = read cell, next clock write cell + 1
    reg        vote_v;
    reg [15:0] vote_addr;

    // scan = read cell, next clock compare it and write 0
    reg [6:0]  st;                       // scan position (theta, rho)
    reg [8:0]  sr;
    reg        scan_v;                   // a scanned cell arrives this clock
    reg [6:0]  scan_t;
    reg [8:0]  scan_r;
    reg [8:0]  lmax_v, rmax_v;           // highest cell of each lane region
    reg [6:0]  lmax_t, rmax_t;
    reg [8:0]  lmax_r, rmax_r;

    wire in_rho   = (scan_r >= RHO_LO) && (scan_r <= RHO_HI);
    wire in_left  = in_rho && (scan_t >= LEFT_T0)  && (scan_t <= LEFT_T1);
    wire in_right = in_rho && (scan_t >= RIGHT_T0) && (scan_t <= RIGHT_T1);

    // ------------------------------------------------ Algorithm 1 (local_maxima.v)
    reg         lm_start;
    wire [15:0] lm_raddr;
    wire        lm_done;
    wire [6:0]  lm_t;
    wire [8:0]  lm_r, lm_v;

    local_maxima u_lmax (
        .clk(clk), .rst(rst), .start(lm_start),
        .t_lo(state == PEAK_R ? RIGHT_T0 : LEFT_T0),
        .t_hi(state == PEAK_R ? RIGHT_T1 : LEFT_T1),
        .r_lo(RHO_LO), .r_hi(RHO_HI),
        .raddr(lm_raddr), .rdata(acc_q),
        .done(lm_done), .max_t(lm_t), .max_r(lm_r), .max_v(lm_v)
    );

    // ------------------------------------------------ RAM port use
    always @(*) begin
        case (state)
            VOTE:    acc_raddr = {t, rho_bin};
            SCAN:    acc_raddr = {st, sr};
            default: acc_raddr = lm_raddr;
        endcase
        if (state == CLEAR) begin
            acc_we = 1'b1;  acc_waddr = clr_addr;  acc_wdata = 9'd0;
        end else if (scan_v) begin
            acc_we = 1'b1;  acc_waddr = {scan_t, scan_r};  acc_wdata = 9'd0;
        end else begin
            acc_we    = vote_v;
            acc_waddr = vote_addr;
            acc_wdata = (acc_q == 9'd511) ? 9'd511 : acc_q + 9'd1;   // +1 (saturating)
        end
    end

    assign busy = (state != IDLE);

    // ------------------------------------------------ control
    always @(posedge clk) begin
        if (rst) begin
            state    <= CLEAR;
            clr_addr <= 16'd0;
            vote_v   <= 1'b0;
            scan_v   <= 1'b0;
            lm_start <= 1'b0;
            done     <= 1'b0;
            e_idx    <= 13'd0;
            left_ok  <= 1'b0;  left_t  <= 7'd0;  left_r  <= 9'd256;
            right_ok <= 1'b0;  right_t <= 7'd0;  right_r <= 9'd256;
        end else begin
            vote_v   <= 1'b0;
            scan_v   <= 1'b0;
            lm_start <= 1'b0;
            done     <= 1'b0;

            // ---- highest cell of each lane region (value read last clock) ----
            if (scan_v && in_left && (acc_q > lmax_v)) begin
                lmax_v <= acc_q;  lmax_t <= scan_t;  lmax_r <= scan_r;
            end
            if (scan_v && in_right && (acc_q > rmax_v)) begin
                rmax_v <= acc_q;  rmax_t <= scan_t;  rmax_r <= scan_r;
            end

            case (state)
            CLEAR: begin                                  // Accum = 0 (after reset)
                clr_addr <= clr_addr + 16'd1;
                if (clr_addr == ACC_SIZE - 1) begin
                    clr_addr <= 16'd0;
                    state    <= IDLE;
                end
            end
            IDLE: begin
                if (start) begin
                    n_r   <= n_edges;
                    mode  <= peak_mode;
                    e_idx <= 13'd0;
                    state <= LOAD;
                end
            end
            LOAD: begin                                   // read edge pixel e_idx
                if (e_idx == n_r) begin                   // all edges voted
                    lmax_v <= 9'd0;  lmax_t <= LEFT_T0;   lmax_r <= 9'd256;
                    rmax_v <= 9'd0;  rmax_t <= RIGHT_T0;  rmax_r <= 9'd256;
                    st <= 7'd0;  sr <= 9'd0;
                    if (mode) begin
                        lm_start <= 1'b1;
                        state    <= PEAK_L;
                    end else
                        state    <= SCAN;
                end else
                    state <= LOAD2;
            end
            LOAD2: begin                                  // (x, y) -> CORDIC (c_load)
                t      <= 7'd0;
                state  <= VOTE;
            end
            VOTE: begin                                   // one theta per clock
                if (!skip) begin
                    vote_v    <= 1'b1;
                    vote_addr <= {t, rho_bin};
                end
                t <= t + 7'd1;
                if (t == T_LAST) begin
                    e_idx <= e_idx + 13'd1;
                    state <= LOAD;
                end
            end
            PEAK_L: begin                                 // Algorithm 1, left lane
                if (lm_done) begin
                    left_t  <= lm_t;  left_r <= lm_r;
                    left_ok <= (lm_v >= MIN_VOTES);
                    lm_start <= 1'b1;
                    state    <= PEAK_R;
                end
            end
            PEAK_R: begin                                 // Algorithm 1, right lane
                if (lm_done) begin
                    right_t  <= lm_t;  right_r <= lm_r;
                    right_ok <= (lm_v >= MIN_VOTES);
                    state    <= SCAN;                     // clear for next frame
                end
            end
            SCAN: begin                                   // read every cell once
                scan_v <= 1'b1;  scan_t <= st;  scan_r <= sr;
                if (sr == 9'd511) begin
                    sr <= 9'd0;
                    if (st == T_LAST) state <= SCAN_W;
                    else              st <= st + 7'd1;
                end else
                    sr <= sr + 9'd1;
            end
            SCAN_W: state <= RESULT;                      // last cell compared / cleared
            RESULT: begin
                if (!mode) begin                          // highest cell of each region
                    left_t   <= lmax_t;  left_r  <= lmax_r;  left_ok  <= (lmax_v >= MIN_VOTES);
                    right_t  <= rmax_t;  right_r <= rmax_r;  right_ok <= (rmax_v >= MIN_VOTES);
                end
                done  <= 1'b1;
                state <= IDLE;
            end
            default: state <= CLEAR;
            endcase
        end
    end
endmodule
