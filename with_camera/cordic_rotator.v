// ============================================================================
// cordic_rotator.v  -  paper Fig. 13 (CORDIC Hough operation register) and eq. (9)
//
//   x(i+1) = x(i) + y(i) * 2^-5        ("Shifter >> 5" + Adder)
//   y(i+1) = y(i) - x(i) * 2^-5        ("Shifter >> 5" + Subtractor)
//
// Every step rotates the vector (x, y) by the constant angle
//   phi = atan(2^-5) = 1.79 deg        (paper: "tan(phi) = 0.031 = 2^-5,
//                                        therefore 5 bits should be shifted")
// After i steps:
//   x(i) = K_i * ( x cos(i*phi) + y sin(i*phi) ) = K_i * rho(i*phi)    eq. (7)
//   K_i  = (1 + 2^-10)^(i/2)   (CORDIC gain, 1.000 ... 1.050 for i = 0..100)
// So one step per clock gives rho for the next theta.  The gain K_i is the
// same for all pixels at one theta, so the Hough votes are not affected;
// it is taken into account again when the line is drawn (vga_display.v uses
// the same rotator to get K_i*cos and K_i*sin).
//
//   load : x <= x_in, y <= y_in        step : one rotation by phi
// ============================================================================
module cordic_rotator #(
    parameter W = 22                      // register width (fixed point)
)(
    input                     clk,
    input                     load,
    input                     step,
    input  signed [W-1:0]     x_in,
    input  signed [W-1:0]     y_in,
    output reg signed [W-1:0] x,
    output reg signed [W-1:0] y
);
    always @(posedge clk) begin
        if (load) begin
            x <= x_in;
            y <= y_in;
        end else if (step) begin
            x <= x + (y >>> 5);           // adder
            y <= y - (x >>> 5);           // subtractor
        end
    end
endmodule
