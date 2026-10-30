// ============================================================================
// local_maxima.v  -  paper Algorithm 1 (Local Maxima Algorithm)
//
// Finds the local maximum of the accumulator matrix Accum[theta][rho] inside
// the sub-matrix  theta = t_lo..t_hi , rho = r_lo..r_hi.
//
//  Function Local_Maximum(Accum):
//   2   middle_row    <- theta / 2
//   3   middle_column <- rho / 2
//   4   max_cell  <- Max( Max_in_Row(middle_row), Max_in_Column(middle_column) )
//   6   next_cell <- Next_Cell(max_cell)       (largest of the 4 neighbours)
//   7   if next_cell = max_cell  (no neighbour is larger) -> return max_cell
//  10   else choose the sub-matrix that contains next_cell:
//          next_cell.row    > middle_row    -> Bottom  else Top
//          next_cell.column > middle_column -> Right   else Left
//       and repeat on that sub-matrix (Top-Left / Top-Right /
//       Bottom-Left / Bottom-Right).
//  The matrix is halved every time -> about 4N cells are read, O(N).
//
// In hardware the recursion is a loop (state machine) that changes the
// sub-matrix limits.  When the sub-matrix is only 1-2 rows or columns wide,
// all its cells are checked.
//
// Note: next_cell of the previous pass is kept (g_*) and used as max_cell if
// the new middle row / column has only smaller values.  This way the search
// always keeps climbing towards the peak it was going to and cannot end on a
// small noise peak of the new sub-matrix.
//
// Memory: accumulator read port, data 1 clock after the address.
// ============================================================================
module local_maxima (
    input              clk,
    input              rst,
    input              start,
    input      [6:0]   t_lo, t_hi,      // theta range (rows)
    input      [8:0]   r_lo, r_hi,      // rho range   (columns)
    output reg [15:0]  raddr,           // {theta, rho}
    input      [8:0]   rdata,
    output reg         done,
    output reg [6:0]   max_t,
    output reg [8:0]   max_r,
    output reg [8:0]   max_v
);
    localparam IDLE  = 4'd0,  ITER  = 4'd1,  ROW   = 4'd2,  COL   = 4'd3,
               WAIT1 = 4'd4,  BEST  = 4'd5,  NEIGH = 4'd6,  WAIT2 = 4'd7,
               DECIDE= 4'd8,  SMALL = 4'd9,  WAIT3 = 4'd10, FINISH= 4'd11;

    reg [3:0] state;
    reg [6:0] lo_t, hi_t, mid_t, ct;        // current sub-matrix (rows)
    reg [8:0] lo_r, hi_r, mid_r, cr;        // current sub-matrix (columns)

    // max_cell, next_cell and "best so far" (value, row, column)
    reg       mc_f;  reg [8:0] mc_v;  reg [6:0] mc_t;  reg [8:0] mc_r;
    reg       nc_f;  reg [8:0] nc_v;  reg [6:0] nc_t;  reg [8:0] nc_r;
    reg       g_f;   reg [8:0] g_v;   reg [6:0] g_t;   reg [8:0] g_r;
    reg [1:0] nb;                          // neighbour number 0..3

    wire [7:0] sum_t = {1'b0, lo_t} + {1'b0, hi_t};
    wire [9:0] sum_r = {1'b0, lo_r} + {1'b0, hi_r};

    // cell that was read in the previous clock
    reg       rd_v;  reg rd_next;  reg [6:0] rd_t;  reg [8:0] rd_r;

    // neighbour nb of max_cell (top, bottom, left, right) and if it is inside
    reg       nb_ok;  reg [6:0] nb_t;  reg [8:0] nb_r;
    always @(*) begin
        case (nb)
            2'd0:    begin nb_ok = (mc_t > lo_t); nb_t = mc_t - 7'd1; nb_r = mc_r;        end
            2'd1:    begin nb_ok = (mc_t < hi_t); nb_t = mc_t + 7'd1; nb_r = mc_r;        end
            2'd2:    begin nb_ok = (mc_r > lo_r); nb_t = mc_t;        nb_r = mc_r - 9'd1; end
            default: begin nb_ok = (mc_r < hi_r); nb_t = mc_t;        nb_r = mc_r + 9'd1; end
        endcase
    end

    // address of the cell to read in each state
    always @(*) begin
        case (state)
            ROW:     raddr = {mid_t, cr};          // Max_in_Row(middle_row)
            COL:     raddr = {ct, mid_r};          // Max_in_Column(middle_column)
            NEIGH:   raddr = {nb_t, nb_r};         // Next_Cell(max_cell)
            SMALL:   raddr = {ct, cr};
            default: raddr = 16'd0;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done  <= 1'b0;
            rd_v  <= 1'b0;
        end else begin
            done <= 1'b0;
            rd_v <= 1'b0;

            // ---- compare the value read in the previous clock ----
            if (rd_v) begin
                if (!rd_next) begin                               // max_cell search
                    if (!mc_f || (rdata > mc_v)) begin
                        mc_f <= 1'b1; mc_v <= rdata; mc_t <= rd_t; mc_r <= rd_r;
                    end
                end else begin                                    // next_cell search
                    if (!nc_f || (rdata > nc_v)) begin
                        nc_f <= 1'b1; nc_v <= rdata; nc_t <= rd_t; nc_r <= rd_r;
                    end
                end
            end

            case (state)
            IDLE: begin
                if (start) begin
                    lo_t <= t_lo;  hi_t <= t_hi;
                    lo_r <= r_lo;  hi_r <= r_hi;
                    g_f  <= 1'b0;
                    state <= ITER;
                end
            end
            ITER: begin                                   // new sub-matrix
                mc_f <= 1'b0;  mc_v <= 9'd0;
                nc_f <= 1'b0;  nc_v <= 9'd0;
                if ((hi_t - lo_t < 7'd2) || (hi_r - lo_r < 9'd2)) begin
                    ct <= lo_t;  cr <= lo_r;               // small: check all cells
                    state <= SMALL;
                end else begin
                    mid_t <= sum_t[7:1];                  // middle_row    = (lo+hi)/2
                    mid_r <= sum_r[9:1];                  // middle_column = (lo+hi)/2
                    cr    <= lo_r;
                    state <= ROW;
                end
            end
            ROW: begin                                    // read middle row
                rd_v <= 1'b1; rd_next <= 1'b0; rd_t <= mid_t; rd_r <= cr;
                if (cr == hi_r) begin ct <= lo_t; state <= COL; end
                else            cr <= cr + 9'd1;
            end
            COL: begin                                    // read middle column
                rd_v <= 1'b1; rd_next <= 1'b0; rd_t <= ct; rd_r <= mid_r;
                if (ct == hi_t) state <= WAIT1;
                else            ct <= ct + 7'd1;
            end
            WAIT1: state <= BEST;                         // last value arriving
            BEST: begin                                   // max_cell (+ best so far)
                if (g_f && (g_v > mc_v)) begin
                    mc_v <= g_v;  mc_t <= g_t;  mc_r <= g_r;
                end
                nb    <= 2'd0;
                state <= NEIGH;
            end
            NEIGH: begin                                  // read the 4 neighbours
                if (nb_ok) begin
                    rd_v <= 1'b1; rd_next <= 1'b1; rd_t <= nb_t; rd_r <= nb_r;
                end
                nb <= nb + 2'd1;
                if (nb == 2'd3) state <= WAIT2;
            end
            WAIT2: state <= DECIDE;
            DECIDE: begin
                if (nc_f && (nc_v > mc_v)) begin          // next_cell != max_cell
                    g_f <= 1'b1;  g_v <= nc_v;  g_t <= nc_t;  g_r <= nc_r;
                    if (nc_t > mid_t) lo_t <= mid_t + 7'd1;    // Bottom sub-matrix
                    else              hi_t <= mid_t - 7'd1;    // Top
                    if (nc_r > mid_r) lo_r <= mid_r + 9'd1;    // Right
                    else              hi_r <= mid_r - 9'd1;    // Left
                    state <= ITER;
                end else
                    state <= FINISH;                      // return max_cell
            end
            SMALL: begin
                rd_v <= 1'b1; rd_next <= 1'b0; rd_t <= ct; rd_r <= cr;
                if (cr == hi_r) begin
                    cr <= lo_r;
                    if (ct == hi_t) state <= WAIT3;
                    else            ct <= ct + 7'd1;
                end else
                    cr <= cr + 9'd1;
            end
            WAIT3: state <= FINISH;
            FINISH: begin
                max_t <= mc_t;  max_r <= mc_r;  max_v <= mc_v;
                done  <= 1'b1;
                state <= IDLE;
            end
            default: state <= IDLE;
            endcase
        end
    end
endmodule
