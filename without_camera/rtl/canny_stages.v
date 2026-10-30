// ============================================================================
// canny_stages.v : the five stages of Canny edge detection (Fig. 3)
//   gray_conv  : Y = (5R + 9G + 2B) >> 4                 (paper eq. 8)
//   gauss5x5   : 5x5 Gaussian, sigma=1, /273              (paper eq. 2)
//   sobel3x3   : Gx, Gy, |G| = sqrt(Gx^2+Gy^2), direction (eq. 3-6, Fig. 12)
//   nms3x3     : non-maximum suppression (4 directions)
//   hyst3x3    : double threshold + 8-neighbour hysteresis
// All stages are fully pipelined, 1 pixel / clock, 1 output per input.
// ============================================================================

// ---------------------------------------------------------------------------
module gray_conv (
    input             clk,
    input             rst,
    input             in_valid,
    input      [23:0] in_rgb,          // {R,G,B}
    output reg        out_valid,
    output reg [7:0]  out_y
);
    wire [11:0] sum = 12'd5 * in_rgb[23:16] + 12'd9 * in_rgb[15:8] + 12'd2 * in_rgb[7:0];
    always @(posedge clk) begin
        out_valid <= rst ? 1'b0 : in_valid;
        out_y     <= sum[11:4];                       // 12-bit -> 8-bit
    end
endmodule

// ---------------------------------------------------------------------------
module gauss5x5 #(parameter W = 320, parameter AW = 9) (
    input             clk,
    input             rst,
    input             in_valid,
    input      [7:0]  in_data,
    output            out_valid,
    output     [7:0]  out_data
);
    wire          wv;
    wire [199:0]  win;
    window_gen #(.DW(8), .K(5), .W(W), .AW(AW)) u_win (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data(in_data),
        .out_valid(wv), .win(win));

    // p(r,c) = window element
    `define GP(r,c) {9'd0, win[((r)*5+(c))*8 +: 8]}

    reg [16:0] rs0, rs1, rs2, rs3, rs4;   // row sums
    reg [16:0] tot;
    reg [28:0] prod;
    reg [3:0]  v;
    always @(posedge clk) begin
        // P1 : weighted row sums   [1 4 7 4 1] [4 16 26 16 4] [7 26 41 26 7]
        rs0 <=      `GP(0,0) + 17'd4 *`GP(0,1) + 17'd7 *`GP(0,2) + 17'd4 *`GP(0,3) +       `GP(0,4);
        rs1 <= 17'd4*`GP(1,0) + 17'd16*`GP(1,1) + 17'd26*`GP(1,2) + 17'd16*`GP(1,3) + 17'd4*`GP(1,4);
        rs2 <= 17'd7*`GP(2,0) + 17'd26*`GP(2,1) + 17'd41*`GP(2,2) + 17'd26*`GP(2,3) + 17'd7*`GP(2,4);
        rs3 <= 17'd4*`GP(3,0) + 17'd16*`GP(3,1) + 17'd26*`GP(3,2) + 17'd16*`GP(3,3) + 17'd4*`GP(3,4);
        rs4 <=      `GP(4,0) + 17'd4 *`GP(4,1) + 17'd7 *`GP(4,2) + 17'd4 *`GP(4,3) +       `GP(4,4);
        // P2 : total (max 273*255 = 69615)
        tot  <= rs0 + rs1 + rs2 + rs3 + rs4;
        // P3 : multiply by round(2^20/273) = 3841  ( == divide by 273 )
        prod <= tot * 29'd3841;
        v    <= rst ? 4'd0 : {v[2:0], wv};
    end
    `undef GP
    assign out_data  = prod[27:20];
    assign out_valid = v[2];
endmodule

// ---------------------------------------------------------------------------
// Pipelined integer square root (digit-by-digit, 1 result bit per stage)
// root = floor(sqrt(rad)).  Latency = OW clocks.  SBW-bit side band follows.
// ---------------------------------------------------------------------------
module isqrt_pipe #(parameter IW = 22, parameter OW = 11, parameter SBW = 2) (
    input                clk,
    input                rst,
    input                in_valid,
    input  [IW-1:0]      in_rad,
    input  [SBW-1:0]     in_sb,
    output               out_valid,
    output [OW-1:0]      out_root,
    output [SBW-1:0]     out_sb
);
    localparam RW = OW + 3;
    // flattened stage buses: stage s input at [s*X +: X]
    wire [(OW+1)*IW-1:0]  rad_w;
    wire [(OW+1)*RW-1:0]  rem_w;
    wire [(OW+1)*OW-1:0]  root_w;
    wire [(OW+1)*SBW-1:0] sb_w;
    wire [OW:0]           vld_w;

    assign rad_w [0 +: IW]  = in_rad;
    assign rem_w [0 +: RW]  = {RW{1'b0}};
    assign root_w[0 +: OW]  = {OW{1'b0}};
    assign sb_w  [0 +: SBW] = in_sb;
    assign vld_w [0]        = in_valid;

    genvar s;
    generate
        for (s = 0; s < OW; s = s + 1) begin : g_st
            wire [IW-1:0]  rad_i  = rad_w [s*IW  +: IW];
            wire [RW-1:0]  rem_i  = rem_w [s*RW  +: RW];
            wire [OW-1:0]  root_i = root_w[s*OW  +: OW];
            wire [RW-1:0]  rem_t  = {rem_i[RW-3:0], rad_i[IW-1:IW-2]};
            wire [RW-1:0]  trial  = {{(RW-OW-2){1'b0}}, root_i, 2'b01};
            reg  [IW-1:0]  rad_r;
            reg  [RW-1:0]  rem_r;
            reg  [OW-1:0]  root_r;
            reg  [SBW-1:0] sb_r;
            reg            vld_r;
            always @(posedge clk) begin
                rad_r <= rad_i << 2;
                sb_r  <= sb_w[s*SBW +: SBW];
                vld_r <= rst ? 1'b0 : vld_w[s];
                if (rem_t >= trial) begin
                    rem_r  <= rem_t - trial;
                    root_r <= {root_i[OW-2:0], 1'b1};
                end else begin
                    rem_r  <= rem_t;
                    root_r <= {root_i[OW-2:0], 1'b0};
                end
            end
            assign rad_w [(s+1)*IW  +: IW]  = rad_r;
            assign rem_w [(s+1)*RW  +: RW]  = rem_r;
            assign root_w[(s+1)*OW  +: OW]  = root_r;
            assign sb_w  [(s+1)*SBW +: SBW] = sb_r;
            assign vld_w [s+1]              = vld_r;
        end
    endgenerate

    assign out_valid = vld_w[OW];
    assign out_root  = root_w[OW*OW +: OW];
    assign out_sb    = sb_w[OW*SBW +: SBW];
endmodule

// ---------------------------------------------------------------------------
// Sobel: Gx = right - left column, Gy = bottom - top row (1,2,1 weights)
// magnitude = floor(sqrt(Gx^2+Gy^2)) (0..1442), direction quantised to
//   0 : ~0 deg (horizontal gradient)   1 : ~45 deg (Gx,Gy same sign)
//   2 : ~90 deg (vertical gradient)    3 : ~135 deg (opposite signs)
// ---------------------------------------------------------------------------
module sobel3x3 #(parameter W = 320, parameter AW = 9) (
    input             clk,
    input             rst,
    input             in_valid,
    input      [7:0]  in_data,
    output            out_valid,
    output     [10:0] out_mag,
    output     [1:0]  out_dir
);
    wire         wv;
    wire [71:0]  win;
    window_gen #(.DW(8), .K(3), .W(W), .AW(AW)) u_win (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data(in_data),
        .out_valid(wv), .win(win));

    `define SP(r,c) $signed({3'b000, win[((r)*3+(c))*8 +: 8]})

    // P1 : gradients
    reg signed [10:0] gx, gy;
    reg               v1;
    always @(posedge clk) begin
        gx <= (`SP(0,2) + (`SP(1,2) <<< 1) + `SP(2,2)) - (`SP(0,0) + (`SP(1,0) <<< 1) + `SP(2,0));
        gy <= (`SP(2,0) + (`SP(2,1) <<< 1) + `SP(2,2)) - (`SP(0,0) + (`SP(0,1) <<< 1) + `SP(0,2));
        v1 <= rst ? 1'b0 : wv;
    end
    `undef SP

    // P2 : |Gx|,|Gy|, Gx^2 + Gy^2, sign relation
    reg [9:0]  ax, ay;
    reg [20:0] sq;
    reg        same, v2;
    always @(posedge clk) begin
        ax   <= gx[10] ? -gx : gx;
        ay   <= gy[10] ? -gy : gy;
        sq   <= gx * gx + gy * gy;
        same <= (gx[10] == gy[10]);
        v2   <= rst ? 1'b0 : v1;
    end

    // P3 : direction quantisation (tan 22.5 = 106/256, tan 67.5 = 618/256)
    reg [1:0]  dir;
    reg [20:0] sq3;
    reg        v3;
    wire [19:0] ay256 = {ay, 8'd0};
    wire [19:0] ax106 = ax * 20'd106;
    wire [19:0] ax618 = ax * 20'd618;
    always @(posedge clk) begin
        if (ay256 <= ax106)      dir <= 2'd0;
        else if (ay256 >= ax618) dir <= 2'd2;
        else if (same)           dir <= 2'd1;
        else                     dir <= 2'd3;
        sq3 <= sq;
        v3  <= rst ? 1'b0 : v2;
    end

    // P4.. : pipelined square root (Fig. 12 "sqrt" block)
    isqrt_pipe #(.IW(22), .OW(11), .SBW(2)) u_sqrt (
        .clk(clk), .rst(rst), .in_valid(v3), .in_rad({1'b0, sq3}), .in_sb(dir),
        .out_valid(out_valid), .out_root(out_mag), .out_sb(out_dir));
endmodule

// ---------------------------------------------------------------------------
// Non-maximum suppression: keep centre if  mag > nA  and  mag >= nB
//   dir0: nA=right      nB=left        dir1: nA=down-right nB=up-left
//   dir2: nA=down       nB=up          dir3: nA=up-right   nB=down-left
// ---------------------------------------------------------------------------
module nms3x3 #(parameter W = 320, parameter AW = 9) (
    input             clk,
    input             rst,
    input             in_valid,
    input      [10:0] in_mag,
    input      [1:0]  in_dir,
    output reg        out_valid,
    output reg [10:0] out_mag
);
    wire         wv;
    wire [116:0] win;               // 9 x 13 bit {dir, mag}
    window_gen #(.DW(13), .K(3), .W(W), .AW(AW)) u_win (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data({in_dir, in_mag}),
        .out_valid(wv), .win(win));

    `define NM(r,c) win[((r)*3+(c))*13 +: 11]
    wire [1:0] cdir = win[(1*3+1)*13 + 11 +: 2];

    reg [10:0] m, na, nb;
    reg        v1;
    always @(posedge clk) begin
        m  <= `NM(1,1);
        case (cdir)
            2'd0: begin na <= `NM(1,2); nb <= `NM(1,0); end
            2'd1: begin na <= `NM(2,2); nb <= `NM(0,0); end
            2'd2: begin na <= `NM(2,1); nb <= `NM(0,1); end
            default: begin na <= `NM(0,2); nb <= `NM(2,0); end
        endcase
        v1 <= rst ? 1'b0 : wv;
        out_mag   <= (m > na && m >= nb) ? m : 11'd0;
        out_valid <= rst ? 1'b0 : v1;
    end
    `undef NM
endmodule

// ---------------------------------------------------------------------------
// Hysteresis: strong = mag >= TH_HIGH, weak = TH_LOW <= mag < TH_HIGH
//   edge = strong  OR  (weak AND at least one of the 8 neighbours strong)
// ---------------------------------------------------------------------------
module hyst3x3 #(parameter W = 320, parameter AW = 9,
                 parameter TH_HIGH = 160, parameter TH_LOW = 80) (
    input             clk,
    input             rst,
    input             in_valid,
    input      [10:0] in_mag,
    output reg        out_valid,
    output reg        out_edge
);
    reg [1:0] cls;          // bit1 strong, bit0 weak
    reg       cv;
    always @(posedge clk) begin
        cls <= (in_mag >= TH_HIGH) ? 2'b10 : (in_mag >= TH_LOW) ? 2'b01 : 2'b00;
        cv  <= rst ? 1'b0 : in_valid;
    end

    wire        wv;
    wire [17:0] win;
    window_gen #(.DW(2), .K(3), .W(W), .AW(AW)) u_win (
        .clk(clk), .rst(rst), .in_valid(cv), .in_data(cls),
        .out_valid(wv), .win(win));

    wire any_strong = win[1] | win[3] | win[5] | win[7] |            // (0,0)(0,1)(0,2)(1,0)
                      win[11] | win[13] | win[15] | win[17];       // (1,2)(2,0)(2,1)(2,2)
    wire c_strong = win[9];
    wire c_weak   = win[8];
    always @(posedge clk) begin
        out_edge  <= c_strong | (c_weak & any_strong);
        out_valid <= rst ? 1'b0 : wv;
    end
endmodule

// ---------------------------------------------------------------------------
// One complete Canny engine ("thread" of the paper).  Streams RGB pixels in,
// streams edge bits out.  The spatial offset between input and output is
// (5 rows + 5 columns): gauss 2 + sobel 1 + nms 1 + hysteresis 1.
// ---------------------------------------------------------------------------
module canny_engine #(parameter W = 320, parameter AW = 9,
                      parameter TH_HIGH = 160, parameter TH_LOW = 80) (
    input         clk,
    input         rst,
    input         in_valid,
    input  [23:0] in_rgb,
    output        out_valid,
    output        out_edge
);
    wire       gv;  wire [7:0]  gy;
    wire       bv;  wire [7:0]  bl;
    wire       sv;  wire [10:0] sm;  wire [1:0] sd;
    wire       nv;  wire [10:0] nm;

    gray_conv u_gray (.clk(clk), .rst(rst), .in_valid(in_valid), .in_rgb(in_rgb),
                      .out_valid(gv), .out_y(gy));
    gauss5x5 #(.W(W), .AW(AW)) u_gauss (.clk(clk), .rst(rst), .in_valid(gv), .in_data(gy),
                      .out_valid(bv), .out_data(bl));
    sobel3x3 #(.W(W), .AW(AW)) u_sobel (.clk(clk), .rst(rst), .in_valid(bv), .in_data(bl),
                      .out_valid(sv), .out_mag(sm), .out_dir(sd));
    nms3x3   #(.W(W), .AW(AW)) u_nms (.clk(clk), .rst(rst), .in_valid(sv), .in_mag(sm),
                      .in_dir(sd), .out_valid(nv), .out_mag(nm));
    hyst3x3  #(.W(W), .AW(AW), .TH_HIGH(TH_HIGH), .TH_LOW(TH_LOW)) u_hyst (
                      .clk(clk), .rst(rst), .in_valid(nv), .in_mag(nm),
                      .out_valid(out_valid), .out_edge(out_edge));
endmodule
