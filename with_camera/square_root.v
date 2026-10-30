// ============================================================================
// square_root.v  -  the "sqrt" block of paper Fig. 12
//
// Integer square root of a 16-bit number (result 8 bit), pipelined:
// one result bit per stage (digit-by-digit method), 8 stages.
//   for each bit (MSB first):
//       rem   = rem*4 + next two input bits
//       trial = root*4 + 1
//       if rem >= trial : rem = rem - trial, root = root*2 + 1
//       else            :                    root = root*2
// "tag" carries the pixel coordinate / direction along with the data.
// Latency: 8 clocks.
// ============================================================================
module square_root #(
    parameter TW = 19
)(
    input               clk,
    input               rst,
    input               in_valid,
    input      [15:0]   in_val,
    input      [TW-1:0] in_tag,
    output              out_valid,
    output     [7:0]    out_root,
    output     [TW-1:0] out_tag
);
    // bus slot s = input of stage s, slot 8 = output
    wire [8:0]      vb;
    wire [16*9-1:0] radb;
    wire [10*9-1:0] remb;
    wire [8*9-1:0]  rootb;
    wire [TW*9-1:0] tagb;

    assign vb[0]        = in_valid;
    assign radb[15:0]   = in_val;
    assign remb[9:0]    = 10'd0;
    assign rootb[7:0]   = 8'd0;
    assign tagb[TW-1:0] = in_tag;

    genvar s;
    generate
        for (s = 0; s < 8; s = s + 1) begin : STAGE
            wire [15:0] rad   = radb [s*16 +: 16];
            wire [9:0]  rem   = remb [s*10 +: 10];
            wire [7:0]  root  = rootb[s*8  +: 8];
            wire [11:0] rem4  = {rem, rad[15:14]};       // rem*4 + 2 bits
            wire [11:0] trial = {2'b00, root, 2'b01};    // root*4 + 1
            wire        ge    = (rem4 >= trial);
            wire [11:0] diff  = rem4 - trial;

            reg          v_q;
            reg [15:0]   rad_q;
            reg [9:0]    rem_q;
            reg [7:0]    root_q;
            reg [TW-1:0] tag_q;

            always @(posedge clk) begin
                if (rst) v_q <= 1'b0; else v_q <= vb[s];
                rad_q  <= {rad[13:0], 2'b00};
                rem_q  <= ge ? diff[9:0] : rem4[9:0];
                root_q <= {root[6:0], ge};
                tag_q  <= tagb[s*TW +: TW];
            end

            assign vb[s+1]              = v_q;
            assign radb [(s+1)*16 +: 16] = rad_q;
            assign remb [(s+1)*10 +: 10] = rem_q;
            assign rootb[(s+1)*8  +: 8]  = root_q;
            assign tagb [(s+1)*TW +: TW] = tag_q;
        end
    endgenerate

    assign out_valid = vb[8];
    assign out_root  = rootb[8*8 +: 8];
    assign out_tag   = tagb[8*TW +: TW];
endmodule
