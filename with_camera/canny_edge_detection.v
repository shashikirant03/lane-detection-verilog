// ============================================================================
// canny_edge_detection.v  -  paper Fig. 3 and Algorithm 3 (Edge_detection)
//
//   gray -> Gaussian smoothing -> Sobel operator -> Non-maximum suppression
//        -> Hysteresis -> edge (1 = edge pixel)
//
// PARALLEL PROCESSING (paper Sec. IV-C, Fig. 7):
// In the paper the image is cut into slices and each thread processes its
// slice.  In hardware all four stages below work AT THE SAME TIME on
// different rows of the image (a pipeline that "slides" down the image),
// and inside every stage all window multiplications are done in parallel,
// so one pixel is finished every clock.  The line memories of each stage
// correspond to the rows each thread keeps (Fig. 11).
//
// The coordinates travel together with the data.  Border pixels that a
// window cannot cover are dropped: edges exist for x = 4..315, y = 4..235.
// ============================================================================
module canny_edge_detection #(
    parameter IMG_W = 320
)(
    input              clk,
    input              rst,
    input      [7:0]   th,            // hysteresis threshold
    input              in_valid,
    input      [8:0]   in_x,
    input      [7:0]   in_y,
    input      [7:0]   in_gray,
    output             out_valid,
    output     [8:0]   out_x,
    output     [7:0]   out_y,
    output             out_edge
);
    // ---- Gaussian smoothing (output starts at (2,2)) ----
    wire g_v;  wire [8:0] g_x;  wire [7:0] g_y;  wire [7:0] g_pix;
    gaussian_smoothing #(.IMG_W(IMG_W), .X0(0), .Y0(0)) u_gauss (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_x(in_x), .in_y(in_y), .in_pix(in_gray),
        .out_valid(g_v), .out_x(g_x), .out_y(g_y), .out_pix(g_pix)
    );

    // ---- Sobel operator (output starts at (3,3)) ----
    wire s_v;  wire [8:0] s_x;  wire [7:0] s_y;  wire [7:0] s_mag;  wire [1:0] s_dir;
    sobel_operator #(.IMG_W(IMG_W), .X0(2), .Y0(2)) u_sobel (
        .clk(clk), .rst(rst),
        .in_valid(g_v), .in_x(g_x), .in_y(g_y), .in_pix(g_pix),
        .out_valid(s_v), .out_x(s_x), .out_y(s_y), .out_mag(s_mag), .out_dir(s_dir)
    );

    // ---- Non-maximum suppression (output starts at (4,4)) ----
    wire n_v;  wire [8:0] n_x;  wire [7:0] n_y;  wire [7:0] n_mag;
    non_max_suppression #(.IMG_W(IMG_W), .X0(3), .Y0(3)) u_nms (
        .clk(clk), .rst(rst),
        .in_valid(s_v), .in_x(s_x), .in_y(s_y), .in_mag(s_mag), .in_dir(s_dir),
        .out_valid(n_v), .out_x(n_x), .out_y(n_y), .out_mag(n_mag)
    );

    // ---- Hysteresis ----
    hysteresis u_hyst (
        .clk(clk), .rst(rst), .th(th),
        .in_valid(n_v), .in_x(n_x), .in_y(n_y), .in_mag(n_mag),
        .out_valid(out_valid), .out_x(out_x), .out_y(out_y), .out_edge(out_edge)
    );
endmodule
