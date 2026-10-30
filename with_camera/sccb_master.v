// ============================================================================
// sccb_master.v  -  SCCB write of one OV7670 register  (3-phase write)
//
// SCCB (Serial Camera Control Bus) is the OmniVision version of I2C:
//
//   START | ID address 0x42 (write) X | register address X | data X | STOP
//
//   X = 9th bit ("don't care" bit in SCCB, the OV7670 normally pulls it low
//       like an I2C ACK; if it stays high, 'nack' is set as a warning only)
//
// SIOC is driven by the FPGA.  SIOD is open drain:
//   sda_oe = 1 pulls SIOD low, sda_oe = 0 releases it (pull-up -> 1).
// Every bit is split into 4 quarter periods:
//   q0: change SIOD (SIOC low)   q1: SIOC high   q2: sample   q3: SIOC low
// ============================================================================
module sccb_master #(
    parameter CLK_HZ  = 50_000_000,
    parameter SCCB_HZ = 100_000
)(
    input             clk,
    input             rst,
    input             go,             // pulse: start a write
    input      [7:0]  dev_addr,       // 0x42 for the OV7670
    input      [7:0]  reg_addr,
    input      [7:0]  data,
    output reg        busy,
    output reg        nack,           // 1 = the camera did not answer
    output reg        scl,            // SIOC
    output reg        sda_oe,         // SIOD pull low
    input             sda_in          // SIOD read back
);
    localparam QUARTER = CLK_HZ / (SCCB_HZ * 4);

    reg [15:0] qcnt;
    reg [1:0]  q;
    reg [4:0]  bitn;     // 0 = START, 1..27 = 3 bytes x (8 bits + X bit), 28 = STOP
    reg [26:0] sh;       // bits to send (X positions = 1 = released)

    wire tick = (qcnt == QUARTER - 1);

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; nack <= 1'b0; scl <= 1'b1; sda_oe <= 1'b0;
            qcnt <= 16'd0; q <= 2'd0; bitn <= 5'd0; sh <= 27'd0;
        end else if (!busy) begin
            scl <= 1'b1; sda_oe <= 1'b0;                   // bus idle (both high)
            if (go) begin
                busy <= 1'b1;
                nack <= 1'b0;
                sh   <= {dev_addr, 1'b1, reg_addr, 1'b1, data, 1'b1};
                bitn <= 5'd0;
                q    <= 2'd0;
                qcnt <= 16'd0;
            end
        end else begin
            qcnt <= tick ? 16'd0 : qcnt + 16'd1;
            if (tick) begin
                q <= q + 2'd1;
                if (bitn == 5'd0) begin                    // START: SIOD falls while SIOC high
                    case (q)
                        2'd0: begin scl <= 1'b1; sda_oe <= 1'b0; end
                        2'd1: begin scl <= 1'b1; sda_oe <= 1'b1; end
                        2'd2: begin scl <= 1'b0; sda_oe <= 1'b1; end
                        2'd3: begin scl <= 1'b0; bitn <= 5'd1;   end
                    endcase
                end else if (bitn <= 5'd27) begin          // data bits and X bits
                    case (q)
                        2'd0: begin scl <= 1'b0; sda_oe <= ~sh[26]; end
                        2'd1: begin scl <= 1'b1; end
                        2'd2: begin
                                  scl <= 1'b1;
                                  if ((bitn == 5'd9 || bitn == 5'd18 || bitn == 5'd27) && sda_in)
                                      nack <= 1'b1;        // camera did not pull X low
                              end
                        2'd3: begin scl <= 1'b0; sh <= {sh[25:0], 1'b1}; bitn <= bitn + 5'd1; end
                    endcase
                end else begin                             // STOP: SIOD rises while SIOC high
                    case (q)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; end
                        2'd1: begin scl <= 1'b1; sda_oe <= 1'b1; end
                        2'd2: begin scl <= 1'b1; sda_oe <= 1'b0; end
                        2'd3: begin busy <= 1'b0; end
                    endcase
                end
            end
        end
    end
endmodule
