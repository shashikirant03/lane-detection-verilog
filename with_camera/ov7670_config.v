// ============================================================================
// ov7670_config.v  -  start-up of the OV7670 camera
//
//   1. RESET pin low for 1 ms, then high, wait 10 ms
//   2. write COM7 = 0x80 (software reset of all registers), wait 10 ms
//   3. write the register table below over SCCB (ID address 0x42)
//   4. write the whole table again when the mirror / flip switches change
//      or when KEY1 (reconfig) is pressed
//
// Output format set by the table:
//   VGA 640 x 480, RGB565 (2 bytes per pixel, full range 0..255)
//   automatic exposure (AEC), automatic gain (AGC, max 4x to keep the noise
//   low) and automatic white balance (AWB) are ON.
//   PCLK: CLKRC = 0x01 with the PLL x4 (DBLV = 0x4A) -> PCLK ~ XCLK
//   (25 MHz XCLK -> about 30 frames/s).
// Most values are the recommended settings from the OV7670 implementation
// guide (also used by the Linux ov7670 driver).  Registers marked "reserved"
// have no documented meaning, the guide just says to write these values.
// ============================================================================
module ov7670_config #(
    parameter CLK_HZ = 50_000_000
)(
    input             clk,
    input             rst,
    input             mirror,         // 1 = mirror left/right
    input             vflip,          // 1 = flip upside down
    input             reconfig,       // pulse
    output reg        cam_reset_n,    // OV7670 RESET pin (active low)
    output            cam_pwdn,       // OV7670 PWDN pin  (1 = power down)
    output            sccb_scl,
    output            sccb_sda_oe,
    input             sccb_sda_in,
    output reg        config_done,
    output reg        ack_error
);
    localparam NREG  = 114;
    localparam T1MS  = CLK_HZ / 1000;
    localparam T10MS = CLK_HZ / 100;

    assign cam_pwdn = 1'b0;                       // camera always on

    // ---- register table {address, value} ----
    reg [6:0] idx;
    reg [7:0] reg_addr, reg_data;
    always @(*) begin
        case (idx)
        // -------- reset, clock, output format
        7'd0:   {reg_addr, reg_data} = 16'h12_80;  // COM7 : software reset
        7'd1:   {reg_addr, reg_data} = 16'h11_01;  // CLKRC: pre-scaler
        7'd2:   {reg_addr, reg_data} = 16'h6B_4A;  // DBLV : PLL x4
        7'd3:   {reg_addr, reg_data} = 16'h12_04;  // COM7 : VGA, RGB output
        7'd4:   {reg_addr, reg_data} = 16'h8C_00;  // RGB444 off
        7'd5:   {reg_addr, reg_data} = 16'h04_00;  // COM1
        7'd6:   {reg_addr, reg_data} = 16'h40_D0;  // COM15: RGB565, range 00..FF
        7'd7:   {reg_addr, reg_data} = 16'h3A_04;  // TSLB
        7'd8:   {reg_addr, reg_data} = 16'h0C_00;  // COM3 : no scaling
        7'd9:   {reg_addr, reg_data} = 16'h3E_00;  // COM14: no PCLK divider
        7'd10:  {reg_addr, reg_data} = 16'h70_3A;  // SCALING_XSC
        7'd11:  {reg_addr, reg_data} = 16'h71_35;  // SCALING_YSC
        7'd12:  {reg_addr, reg_data} = 16'h72_11;  // SCALING_DCWCTR
        7'd13:  {reg_addr, reg_data} = 16'h73_F0;  // SCALING_PCLK_DIV
        7'd14:  {reg_addr, reg_data} = 16'hA2_02;  // SCALING_PCLK_DELAY
        7'd15:  {reg_addr, reg_data} = 16'h15_00;  // COM10: VSYNC positive, PCLK free running
        // -------- 640 x 480 window
        7'd16:  {reg_addr, reg_data} = 16'h17_13;  // HSTART
        7'd17:  {reg_addr, reg_data} = 16'h18_01;  // HSTOP
        7'd18:  {reg_addr, reg_data} = 16'h32_B6;  // HREF
        7'd19:  {reg_addr, reg_data} = 16'h19_02;  // VSTART
        7'd20:  {reg_addr, reg_data} = 16'h1A_7A;  // VSTOP
        7'd21:  {reg_addr, reg_data} = 16'h03_0A;  // VREF
        7'd22:  {reg_addr, reg_data} = {8'h1E, 2'b00, mirror, vflip, 4'b0111}; // MVFP
        // -------- gamma curve
        7'd23:  {reg_addr, reg_data} = 16'h7A_20;
        7'd24:  {reg_addr, reg_data} = 16'h7B_10;
        7'd25:  {reg_addr, reg_data} = 16'h7C_1E;
        7'd26:  {reg_addr, reg_data} = 16'h7D_35;
        7'd27:  {reg_addr, reg_data} = 16'h7E_5A;
        7'd28:  {reg_addr, reg_data} = 16'h7F_69;
        7'd29:  {reg_addr, reg_data} = 16'h80_76;
        7'd30:  {reg_addr, reg_data} = 16'h81_80;
        7'd31:  {reg_addr, reg_data} = 16'h82_88;
        7'd32:  {reg_addr, reg_data} = 16'h83_8F;
        7'd33:  {reg_addr, reg_data} = 16'h84_96;
        7'd34:  {reg_addr, reg_data} = 16'h85_A3;
        7'd35:  {reg_addr, reg_data} = 16'h86_AF;
        7'd36:  {reg_addr, reg_data} = 16'h87_C4;
        7'd37:  {reg_addr, reg_data} = 16'h88_D7;
        7'd38:  {reg_addr, reg_data} = 16'h89_E8;
        // -------- automatic exposure / gain (set up while they are off)
        7'd39:  {reg_addr, reg_data} = 16'h13_E0;  // COM8 : AGC, AEC, AWB off
        7'd40:  {reg_addr, reg_data} = 16'h00_00;  // GAIN
        7'd41:  {reg_addr, reg_data} = 16'h10_00;  // AECH
        7'd42:  {reg_addr, reg_data} = 16'h0D_40;  // COM4
        7'd43:  {reg_addr, reg_data} = 16'h14_18;  // COM9 : max gain 4x
        7'd44:  {reg_addr, reg_data} = 16'hA5_05;  // BD50MAX
        7'd45:  {reg_addr, reg_data} = 16'hAB_07;  // BD60MAX
        7'd46:  {reg_addr, reg_data} = 16'h24_95;  // AEW : AEC upper limit
        7'd47:  {reg_addr, reg_data} = 16'h25_33;  // AEB : AEC lower limit
        7'd48:  {reg_addr, reg_data} = 16'h26_E3;  // VPT
        7'd49:  {reg_addr, reg_data} = 16'h9F_78;  // HAECC1
        7'd50:  {reg_addr, reg_data} = 16'hA0_68;  // HAECC2
        7'd51:  {reg_addr, reg_data} = 16'hA1_03;  // reserved
        7'd52:  {reg_addr, reg_data} = 16'hA6_D8;  // HAECC3
        7'd53:  {reg_addr, reg_data} = 16'hA7_D8;  // HAECC4
        7'd54:  {reg_addr, reg_data} = 16'hA8_F0;  // HAECC5
        7'd55:  {reg_addr, reg_data} = 16'hA9_90;  // HAECC6
        7'd56:  {reg_addr, reg_data} = 16'hAA_94;  // HAECC7
        // -------- reserved values from the implementation guide
        7'd57:  {reg_addr, reg_data} = 16'h0E_61;  // COM5
        7'd58:  {reg_addr, reg_data} = 16'h0F_4B;  // COM6
        7'd59:  {reg_addr, reg_data} = 16'h16_02;
        7'd60:  {reg_addr, reg_data} = 16'h21_02;
        7'd61:  {reg_addr, reg_data} = 16'h22_91;
        7'd62:  {reg_addr, reg_data} = 16'h29_07;
        7'd63:  {reg_addr, reg_data} = 16'h33_0B;
        7'd64:  {reg_addr, reg_data} = 16'h35_0B;
        7'd65:  {reg_addr, reg_data} = 16'h37_1D;
        7'd66:  {reg_addr, reg_data} = 16'h38_71;
        7'd67:  {reg_addr, reg_data} = 16'h39_2A;
        7'd68:  {reg_addr, reg_data} = 16'h3C_78;  // COM12
        7'd69:  {reg_addr, reg_data} = 16'h4D_40;
        7'd70:  {reg_addr, reg_data} = 16'h4E_20;
        7'd71:  {reg_addr, reg_data} = 16'h69_00;  // GFIX
        7'd72:  {reg_addr, reg_data} = 16'h74_10;
        7'd73:  {reg_addr, reg_data} = 16'h8D_4F;
        7'd74:  {reg_addr, reg_data} = 16'h8E_00;
        7'd75:  {reg_addr, reg_data} = 16'h8F_00;
        7'd76:  {reg_addr, reg_data} = 16'h90_00;
        7'd77:  {reg_addr, reg_data} = 16'h91_00;
        7'd78:  {reg_addr, reg_data} = 16'h96_00;
        7'd79:  {reg_addr, reg_data} = 16'h9A_00;
        7'd80:  {reg_addr, reg_data} = 16'hB0_84;
        7'd81:  {reg_addr, reg_data} = 16'hB1_0C;
        7'd82:  {reg_addr, reg_data} = 16'hB2_0E;
        7'd83:  {reg_addr, reg_data} = 16'hB3_82;
        7'd84:  {reg_addr, reg_data} = 16'hB8_0A;
        // -------- automatic white balance
        7'd85:  {reg_addr, reg_data} = 16'h43_0A;
        7'd86:  {reg_addr, reg_data} = 16'h44_F0;
        7'd87:  {reg_addr, reg_data} = 16'h45_34;
        7'd88:  {reg_addr, reg_data} = 16'h46_58;
        7'd89:  {reg_addr, reg_data} = 16'h47_28;
        7'd90:  {reg_addr, reg_data} = 16'h48_3A;
        7'd91:  {reg_addr, reg_data} = 16'h59_88;
        7'd92:  {reg_addr, reg_data} = 16'h5A_88;
        7'd93:  {reg_addr, reg_data} = 16'h5B_44;
        7'd94:  {reg_addr, reg_data} = 16'h5C_67;
        7'd95:  {reg_addr, reg_data} = 16'h5D_49;
        7'd96:  {reg_addr, reg_data} = 16'h5E_0E;
        7'd97:  {reg_addr, reg_data} = 16'h6C_0A;
        7'd98:  {reg_addr, reg_data} = 16'h6D_55;
        7'd99:  {reg_addr, reg_data} = 16'h6E_11;
        7'd100: {reg_addr, reg_data} = 16'h6F_9F;
        7'd101: {reg_addr, reg_data} = 16'h6A_40;  // G channel AWB gain
        7'd102: {reg_addr, reg_data} = 16'h01_40;  // BLUE gain
        7'd103: {reg_addr, reg_data} = 16'h02_60;  // RED gain
        // -------- colour matrix for RGB565
        7'd104: {reg_addr, reg_data} = 16'h4F_B3;
        7'd105: {reg_addr, reg_data} = 16'h50_B3;
        7'd106: {reg_addr, reg_data} = 16'h51_00;
        7'd107: {reg_addr, reg_data} = 16'h52_3D;
        7'd108: {reg_addr, reg_data} = 16'h53_A7;
        7'd109: {reg_addr, reg_data} = 16'h54_E4;
        7'd110: {reg_addr, reg_data} = 16'h58_9E;  // MTXS
        7'd111: {reg_addr, reg_data} = 16'h3D_C0;  // COM13: gamma on, UV auto
        7'd112: {reg_addr, reg_data} = 16'h41_38;  // COM16: auto de-noise / edge, AWB gain
        // -------- finally switch AGC, AEC and AWB on
        default:{reg_addr, reg_data} = 16'h13_E7;  // COM8 : fast AEC, AGC, AWB, AEC on
        endcase
    end

    // ---- SCCB ----
    reg  sccb_go;
    wire sccb_busy, sccb_nack;

    sccb_master #(.CLK_HZ(CLK_HZ), .SCCB_HZ(100_000)) u_sccb (
        .clk(clk), .rst(rst),
        .go(sccb_go), .dev_addr(8'h42), .reg_addr(reg_addr), .data(reg_data),
        .busy(sccb_busy), .nack(sccb_nack),
        .scl(sccb_scl), .sda_oe(sccb_sda_oe), .sda_in(sccb_sda_in)
    );

    // ---- sequence ----
    localparam S_RESET = 3'd0, S_WAIT = 3'd1, S_SEND = 3'd2, S_BUSY = 3'd3,
               S_NEXT  = 3'd4, S_IDLE = 3'd5;

    reg [2:0]  state;
    reg [23:0] timer;
    reg        mir_done, flip_done;
    reg        again;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_RESET;
            timer       <= 24'd0;
            cam_reset_n <= 1'b0;
            idx         <= 7'd0;
            sccb_go     <= 1'b0;
            config_done <= 1'b0;
            ack_error   <= 1'b0;
            again       <= 1'b0;
            mir_done    <= 1'b0;
            flip_done   <= 1'b0;
        end else begin
            sccb_go <= 1'b0;
            if (reconfig) again <= 1'b1;
            case (state)
                S_RESET: begin                            // camera held in reset
                    cam_reset_n <= 1'b0;
                    timer <= timer + 24'd1;
                    if (timer == T1MS) begin timer <= 24'd0; state <= S_WAIT; end
                end
                S_WAIT: begin                             // camera starting
                    cam_reset_n <= 1'b1;
                    timer <= timer + 24'd1;
                    if (timer == T10MS) begin timer <= 24'd0; idx <= 7'd0; state <= S_SEND; end
                end
                S_SEND: begin                             // write register idx
                    if (idx == NREG - 1) begin
                        mir_done  <= mirror;
                        flip_done <= vflip;
                    end
                    sccb_go <= 1'b1;
                    state   <= S_BUSY;
                end
                S_BUSY: begin
                    if (!sccb_go && !sccb_busy) begin
                        if (sccb_nack) ack_error <= 1'b1;
                        state <= S_NEXT;
                    end
                end
                S_NEXT: begin                             // pause, then next register
                    timer <= timer + 24'd1;
                    // after the software reset (idx 0) wait 10 ms, else 10 us
                    if ((idx == 7'd0) ? (timer == T10MS) : (timer == 24'd500)) begin
                        timer <= 24'd0;
                        if (idx == NREG - 1) begin
                            config_done <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            idx   <= idx + 7'd1;
                            state <= S_SEND;
                        end
                    end
                end
                S_IDLE: begin                             // settings changed?
                    if (again || (mirror != mir_done) || (vflip != flip_done)) begin
                        again       <= 1'b0;
                        ack_error   <= 1'b0;
                        config_done <= 1'b0;
                        idx         <= 7'd0;
                        state       <= S_SEND;
                    end
                end
                default: state <= S_RESET;
            endcase
        end
    end
endmodule
