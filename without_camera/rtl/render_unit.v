// ============================================================================
// render_unit.v : image post-processing  ((rho,theta) -> (x,y), draw lines)
//
// Signed distance of pixel (xc,yc) to a lane line (Q14):
//        d = xc*cos(th) + yc*sin(th) - rho
// A pixel belongs to a drawn lane when
//   * |d| <= LINE_THICK                                (on the line)
//   * it is on the car's side of the OTHER lane and at least GAP px away from
//     it  -> the two lanes never touch / cross (no merging at the horizon)
//   * y >= top of that lane.
// How far a lane is drawn (its "top"):
//   the whole bottom half (y >= ROI_Y, the Hough voting area) always; above
//   it the lane is followed upward only while there are edge pixels on it
//   (MEAS pass: per-row evidence bits; WALK: from ROI_Y-1 upward, stop after
//   MAX_GAP rows without evidence, never above EXT_TOP).
// Passes:  MEAS (rows EXT_TOP..ROI_Y-1, no output)  ->  WALK (<= ROI_Y cycles)
//          ->  RUN (all W*H pixels, writes the output image)
// Output word per pixel (2 bit): bit1 = lane, bit0 = edge.
// The paper did this step in MATLAB; here it is done in hardware so the
// FPGA produces the final picture by itself.
// ============================================================================
module render_unit #(
    parameter W = 320, parameter H = 240, parameter T = 4,
    parameter ROI_Y = 120, parameter NRHO = 401,
    parameter EXT_TOP = 72, parameter MAX_GAP = 12, parameter YB = 120,
    parameter LINE_THICK_Q14 = 16384,
    parameter HIT_TOL_Q14 = 24576,
    parameter GAP_Q14 = 196608,
    parameter EAW = $clog2((H/T)*W),
    parameter OAW = $clog2(W*H)
)(
    input                 clk,
    input                 rst,
    input                 start,
    output reg            done,
    output     [EAW-1:0]  edge_raddr,
    input      [T-1:0]    edge_rdata,
    input                 found_l,
    input      [6:0]      k_l,
    input      [15:0]     bin_l,
    input                 found_r,
    input      [6:0]      k_r,
    input      [15:0]     bin_r,
    output reg            out_we,
    output reg [OAW-1:0]  out_addr,
    output reg [1:0]      out_data
);
    localparam S  = H / T;
    localparam BD = S * W;
    localparam RHO_OFF = (NRHO - 1) / 2;

    // ---------------------------------------------------- line constants ----
    wire signed [17:0] g0, g1;
    wire signed [15:0] cl_w, sl_w, cr_w, sr_w;
    trig_lut u_lut_l (.k(k_l), .ginv_q15(g0), .cos_q14(cl_w), .sin_q14(sl_w));
    trig_lut u_lut_r (.k(k_r), .ginv_q15(g1), .cos_q14(cr_w), .sin_q14(sr_w));

    reg signed [15:0] cl, sl, cr, sr;
    reg signed [27:0] rl, rr;          // rho << 14
    reg signed [27:0] refl, refr;      // d of the car point (0,YB) for each line
    reg               fl, fr;
    reg [9:0]         top_l, top_r;    // first row drawn

    localparam ST_IDLE = 3'd0, ST_MEAS = 3'd1, ST_MDRAIN = 3'd2, ST_WALK = 3'd3,
               ST_RUN = 3'd4, ST_DRAIN = 3'd5;
    reg [2:0] st;

    reg [9:0]      rx, ry;
    reg [EAW-1:0]  eaddr;
    reg [7:0]      ebank;
    reg [OAW-1:0]  oaddr;
    reg [3:0]      drain;
    assign edge_raddr = eaddr;

    // evidence bits (one per row above ROI_Y) and the upward walk
    reg [ROI_Y-1:0] hit_l, hit_r;
    reg [9:0]       wy;
    reg             act_l, act_r;
    reg [7:0]       gap_l, gap_r;

    // ----------------------------------------------------------- pipeline ----
    reg                 p1_valid, p1_meas;  reg [7:0] p1_bank;  reg [OAW-1:0] p1_addr;
    reg signed [10:0]   p1_xc, p1_yc;  reg [9:0] p1_row;
    reg                 p2_valid, p2_meas, p2_edge;  reg [OAW-1:0] p2_addr;  reg [9:0] p2_row;
    reg signed [27:0]   p2_xl, p2_yl, p2_xr, p2_yr;

    wire signed [27:0] dl = p2_xl + p2_yl - rl;
    wire signed [27:0] dr = p2_xr + p2_yr - rr;
    wire [27:0] adl = dl[27] ? -dl : dl;
    wire [27:0] adr = dr[27] ? -dr : dr;
    // allowed zone of each lane: car side of the other lane and >= GAP from it
    wire ok_l = fl && (!fr || (((dr >= 0) == (refr >= 0)) && (adr >= GAP_Q14)));
    wire ok_r = fr && (!fl || (((dl >= 0) == (refl >= 0)) && (adl >= GAP_Q14)));
    wire on_l = ok_l && (adl <= LINE_THICK_Q14) && (p2_row >= top_l);
    wire on_r = ok_r && (adr <= LINE_THICK_Q14) && (p2_row >= top_r);
    wire ev_l = ok_l && p2_edge && (adl <= HIT_TOL_Q14);
    wire ev_r = ok_r && p2_edge && (adr <= HIT_TOL_Q14);

    always @(posedge clk) begin
        // stage 1 -> 2 : edge bit arrives, 4 products
        p2_valid <= rst ? 1'b0 : p1_valid;
        p2_meas  <= p1_meas;
        p2_edge  <= edge_rdata[p1_bank];
        p2_addr  <= p1_addr;
        p2_row   <= p1_row;
        p2_xl    <= p1_xc * cl;
        p2_yl    <= p1_yc * sl;
        p2_xr    <= p1_xc * cr;
        p2_yr    <= p1_yc * sr;
        // stage 2 -> 3 : distance tests; output pixel (RUN) or evidence (MEAS)
        out_we   <= rst ? 1'b0 : (p2_valid && !p2_meas);
        out_addr <= p2_addr;
        out_data <= {on_l | on_r, p2_edge};
    end

    always @(posedge clk) begin
        done     <= 1'b0;
        p1_valid <= 1'b0;
        if (rst) begin
            st <= ST_IDLE;
        end else begin
            // evidence collection (from the pipeline, during MEAS / MDRAIN)
            if (p2_valid && p2_meas) begin
                if (ev_l) hit_l[p2_row] <= 1'b1;
                if (ev_r) hit_r[p2_row] <= 1'b1;
            end
            case (st)
            ST_IDLE: if (start) begin
                cl <= cl_w;  sl <= sl_w;  cr <= cr_w;  sr <= sr_w;
                rl <= ($signed({1'b0, bin_l}) - RHO_OFF) <<< 14;
                rr <= ($signed({1'b0, bin_r}) - RHO_OFF) <<< 14;
                refl <= YB * sl_w - (($signed({1'b0, bin_l}) - RHO_OFF) <<< 14);
                refr <= YB * sr_w - (($signed({1'b0, bin_r}) - RHO_OFF) <<< 14);
                fl <= found_l;  fr <= found_r;
                top_l <= ROI_Y;  top_r <= ROI_Y;
                hit_l <= {ROI_Y{1'b0}};  hit_r <= {ROI_Y{1'b0}};
                // MEAS scans rows EXT_TOP .. ROI_Y-1
                rx <= 10'd0;  ry <= EXT_TOP;
                eaddr <= (EXT_TOP % S) * W;  ebank <= EXT_TOP / S;
                oaddr <= EXT_TOP * W;
                st <= ST_MEAS;
            end

            ST_MEAS, ST_RUN: begin
                p1_valid <= 1'b1;
                p1_meas  <= (st == ST_MEAS);
                p1_bank  <= ebank;
                p1_addr  <= oaddr;
                p1_row   <= ry;
                p1_xc    <= $signed({1'b0, rx}) - (W/2);
                p1_yc    <= $signed({1'b0, ry}) - (H/2);
                oaddr    <= oaddr + 1'b1;
                if (eaddr == BD-1) begin eaddr <= {EAW{1'b0}}; ebank <= ebank + 1'b1; end
                else               eaddr <= eaddr + 1'b1;
                if (rx == W-1) begin
                    rx <= 10'd0;
                    ry <= ry + 1'b1;
                    if (st == ST_MEAS && ry == ROI_Y-1) begin st <= ST_MDRAIN; drain <= 4'd0; end
                    if (st == ST_RUN  && ry == H-1)     begin st <= ST_DRAIN;  drain <= 4'd0; end
                end else rx <= rx + 1'b1;
            end

            ST_MDRAIN: begin                     // let the last evidence bits land
                drain <= drain + 1'b1;
                if (drain == 4'd4) begin
                    st <= ST_WALK;
                    wy <= ROI_Y - 1;
                    act_l <= 1'b1;  act_r <= 1'b1;
                    gap_l <= 8'd0;  gap_r <= 8'd0;
                end
            end

            ST_WALK: begin                       // follow the evidence upward
                if (act_l) begin
                    if (hit_l[wy])                begin top_l <= wy; gap_l <= 8'd0; end
                    else if (gap_l + 1 > MAX_GAP) act_l <= 1'b0;
                    else                          gap_l <= gap_l + 1'b1;
                end
                if (act_r) begin
                    if (hit_r[wy])                begin top_r <= wy; gap_r <= 8'd0; end
                    else if (gap_r + 1 > MAX_GAP) act_r <= 1'b0;
                    else                          gap_r <= gap_r + 1'b1;
                end
                if (wy == EXT_TOP) begin
                    st <= ST_RUN;
                    rx <= 10'd0;  ry <= 10'd0;
                    eaddr <= {EAW{1'b0}};  ebank <= 8'd0;  oaddr <= {OAW{1'b0}};
                end else wy <= wy - 1'b1;
            end

            ST_DRAIN: begin
                drain <= drain + 1'b1;
                if (drain == 4'd4) begin done <= 1'b1; st <= ST_IDLE; end
            end
            default: st <= ST_IDLE;
            endcase
        end
    end
endmodule
