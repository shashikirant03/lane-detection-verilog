// ============================================================================
// lane_detection.v  -  lane detection unit (paper Fig. 1 / Fig. 10)
//
//   gray pixels -> Canny edge detection -> edge memory -> Hough transform
//               -> local maxima -> (theta, rho) of left and right lane
//
// Edge memory: the Hough transform needs the edges of a whole frame, so the
// edge pixels (x, y) are written into an on-chip memory while Canny runs.
// Only the bottom half of the image (y >= ROI_Y, below the horizon) is
// stored -> paper optimisation 1 ("only the bottom half of the edge
// detection image will be passed").
// The memory has two halves (banks): while the Hough transform reads the
// edges of frame N, Canny writes the edges of frame N+1 into the other half.
//
// freeze = 1 : nothing new is recorded (picture and lanes stay on screen).
// ============================================================================
module lane_detection #(
    parameter IMG_W    = 320,
    parameter ROI_Y    = 120,       // first image row used for the Hough transform
    parameter MAX_EDGE = 8191,      // edge pixels per frame (memory size)
    parameter END_WAIT = 1023       // clocks after the frame for the pipeline to empty
)(
    input              clk,
    input              rst,
    input      [7:0]   th,             // hysteresis threshold
    input              peak_mode,      // 0 = highest cell per lane region, 1 = Algorithm 1
    input              freeze,
    // gray image (320 x 240)
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_gray,
    input              frame_end,      // pulse after the last pixel of a frame
    // outputs
    output reg         record,         // 1 = this frame is being recorded
    output             edge_valid,     // edge image (for the display)
    output     [8:0]   edge_x,
    output     [7:0]   edge_y,
    output             edge_bit,
    output             lanes_valid,    // pulse: new result
    output             left_ok,
    output     [6:0]   left_t,
    output     [8:0]   left_r,
    output             right_ok,
    output     [6:0]   right_t,
    output     [8:0]   right_r,
    output             hough_busy
);
    // ------------------------------------------------ Canny edge detection
    canny_edge_detection #(.IMG_W(IMG_W)) u_canny (
        .clk(clk), .rst(rst), .th(th),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_gray(in_gray),
        .out_valid(edge_valid), .out_x(edge_x), .out_y(edge_y), .out_edge(edge_bit)
    );

    // ------------------------------------------------ edge memory (2 banks)
    reg         wbank;                 // bank written by Canny
    reg  [12:0] wcount;                // edges stored in this frame
    wire        store = record && edge_valid && edge_bit &&
                        (edge_y >= ROI_Y) && (wcount < MAX_EDGE);

    reg         rbank;                 // bank read by the Hough transform
    wire [12:0] e_idx;
    wire [16:0] e_data;

    dual_port_ram #(.DW(17), .AW(14), .DEPTH(16384)) u_edge_mem (
        .wclk(clk), .we(store), .waddr({wbank, wcount}), .wdata({edge_x, edge_y}),
        .rclk(clk), .raddr({rbank, e_idx}), .rdata(e_data)
    );

    // ------------------------------------------------ frame control
    reg  [10:0] wait_cnt;
    reg         waiting;
    reg         h_start;
    reg  [12:0] h_count;

    always @(posedge clk) begin
        if (rst) begin
            waiting <= 1'b0;  wait_cnt <= 11'd0;
            wbank   <= 1'b0;  rbank    <= 1'b1;  wcount <= 13'd0;
            record  <= 1'b1;
            h_start <= 1'b0;  h_count  <= 13'd0;
        end else begin
            h_start <= 1'b0;
            if (store) wcount <= wcount + 13'd1;

            if (frame_end) begin
                waiting  <= 1'b1;
                wait_cnt <= 11'd0;
            end else if (waiting) begin
                wait_cnt <= wait_cnt + 11'd1;
                if (wait_cnt == END_WAIT) begin          // last edges are written
                    waiting <= 1'b0;
                    if (record && !hough_busy) begin      // give this frame to Hough
                        h_start <= 1'b1;
                        h_count <= wcount;
                        rbank   <= wbank;
                        wbank   <= ~wbank;
                    end
                    wcount <= 13'd0;
                    record <= ~freeze;                    // record the next frame?
                end
            end
        end
    end

    // ------------------------------------------------ Hough + local maxima
    hough_transform u_hough (
        .clk(clk), .rst(rst), .peak_mode(peak_mode),
        .start(h_start), .n_edges(h_count),
        .e_idx(e_idx), .e_x(e_data[16:8]), .e_y(e_data[7:0]),
        .busy(hough_busy), .done(lanes_valid),
        .left_ok(left_ok),   .left_t(left_t),   .left_r(left_r),
        .right_ok(right_ok), .right_t(right_t), .right_r(right_r)
    );
endmodule
