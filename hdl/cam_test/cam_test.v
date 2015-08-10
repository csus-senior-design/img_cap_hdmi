/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

ADV7513 Driver
Authors:  Greg M. Crist, Jr. (gmcrist@gmail.com)
          Padraic Hagerty    (guitarisrockin@hotmail.com)

Description:
  Initializes and drives the ADV7513 IC on the Terasic Cyclone V Starter GX
  board
*/

`ifdef IVERILOG
    `define SIM
`endif

`ifdef VCS
    `define SIM
`endif

module cam_test #(
	// for testbenching
	`ifdef SIM
		parameter ADV7513_INIT_DELAY = 29'd250, // 250ms
	`else
		parameter ADV7513_INIT_DELAY = 29'd50000000, // 1s with 50MHz clock
	`endif

	parameter ADV7513_CHIP_ADDR = 7'h39,    // 0x72 >> 1

	parameter I2C_CLKDIV = 12'd125,			// This value is multiplied by four
											// somewhere, so 125 divides by 500
	parameter I2C_TXN_DELAY = 29'd30000     // 600µs with 50MHz clock
)(
	// Clock signals
	(*
	  chip_pin = "U12"
	*)
	input   CLOCK_125_p,
	(*
	  chip_pin = "R20"
	*)
	input   CLOCK_50_B5B,
	(*
	  chip_pin = "N20"
	*)
	input   CLOCK_50_B6A,
	(*
	  chip_pin = "H12"
	*)
	input   CLOCK_50_B7A,
	(*
	  chip_pin = "M10"
	*)
	input   CLOCK_50_B8A,

	(*
	  chip_pin = "P11"
	*)
	input   RESET,

	// HDMI-TX via ADV7513
	(*
	  chip_pin = "Y25"
	*)
	output  HDMI_TX_CLK,
	(*
	  chip_pin = "Y26"
	*)
	output  HDMI_TX_DE,
	(*
	  chip_pin = "U26"
	*)
	output  HDMI_TX_HS,
	(*
	  chip_pin = "U25"
	*)
	output  HDMI_TX_VS,
	(*
	  chip_pin = "AD25, AC25, AB25, AA24, AB26, R26, R24, P21, P26, N25, P23, P22, R25, R23, T26, T24, T23, U24, V25, V24, W26, W25, AA26, V23"
	*)
	output  [23:0] HDMI_TX_D,
	(*
	  chip_pin = "T12"
	*)
	input   HDMI_TX_INT,

	// Internal i2c bus for HDMI-TX
	(*
	  chip_pin = "B7"
	*)
	inout  I2C_SCL,
	(*
	  chip_pin = "G11"
	*)
	inout  I2C_SDA,

	(*
	  chip_pin = "AC10, V10, AB10, W11, AC8, AD13, AE10, AC9"
	*)
	input  [7:0]  I2C_REG,
	(*
	  chip_pin = "W20, W21, V20, V22, U20, AD6, AD7, AF24, AC19, AE25, AE26, AB19, AD26, AA18, Y18, Y19, Y20, W18, V17, V18, V19"
	*)
	output [20:0] SSEG_OUT,

	// GPIO outputs for snooping the internal i2c bus
	(*
	  chip_pin = "D26, T21"
	*)
	output  [1:0] i2c_snoop,

	// LED Status Indicators
	(*
	  chip_pin = "J10, H7, K8, K10, J7, J8, G7, G6, F6, F7"
	*)
	output reg [9:0] LEDR,
	(*
	  chip_pin = "H9, H8, B6, A5, E9, D8, K6, L7"
	*)
	output reg [7:0] LEDG,

	// User interfaces
	(*
	  chip_pin = "Y16"
	*)
	input I2C_REG_READ
);

    wire de;
    wire vs;
    wire hs;
    wire field = 1'b0;
    wire vs_out;
    wire hs_out;
    wire de_out;

    reg adv7513_init_start;
    wire adv7513_init_done;

    reg adv7513_reg_read_start;
    wire adv7513_reg_read_done;

    reg sseg_en;

    wire [11:0] x_out;
    wire [12:0] y_out;
    wire [7:0] r_out;
    wire [7:0] g_out;
    wire [7:0] b_out;

    wire [7:0] I2C_REG_DATA;
    
    wire clk_1us;

    wire pll_locked;
    
    wire reg_read;

    localparam  [3:0] s_idle                   = 0,
                      s_startup                = 1,
                      s_adv7513_init_start     = 2,
                      s_adv7513_init_wait      = 3,
                      s_adv7513_reg_read_start = 4,
                      s_adv7513_reg_read_wait  = 5;

    (* syn_encoding = "safe" *)
    reg [3:0] state;

    reg [28:0] delay_tick;
    reg delay_done;

    assign i2c_snoop[0] = I2C_SCL;
    assign i2c_snoop[1] = I2C_SDA;

    `ifdef SIM
        assign HDMI_TX_CLK = CLOCK_50_B5B;
        assign pll_locked = 1'b1;

        clk_1us #(
            .CLKDIV(50)
        ) clk_1us_inst (
            .reset(RESET),
            .clk_in(~CLOCK_50_B5B),
            .clk_out(clk_1us)
        );
    `else
        //reg pll_rst;
        
        pll pll_inst (
            .refclk(CLOCK_50_B5B),
            .rst(1'b0),             // Active high
            .outclk_0(HDMI_TX_CLK),
            .outclk_1(clk_1us),
            .locked(pll_locked)
    	);
        
        /*always @(posedge CLOCK_50_B5B)
            if (~pll_locked || ~RESET)
                pll_rst <= 1'b0;
            else
                pll_rst <= 1'b1;*/
    `endif

    assign HDMI_TX_D  = {r_out, g_out, b_out};
    assign HDMI_TX_DE = de_out;
    assign HDMI_TX_HS = hs_out;
    assign HDMI_TX_VS = vs_out;

    localparam VIDEO_FORMAT = 8'd1; // Format 1 (640 x 480p @ 60Hz)
    
    /* ********************* */
    wire INTERLACED;
    wire [11:0] V_TOTAL_0;
    wire [11:0] V_FP_0;
    wire [11:0] V_BP_0;
    wire [11:0] V_SYNC_0;
    wire [11:0] V_TOTAL_1;
    wire [11:0] V_FP_1;
    wire [11:0] V_BP_1;
    wire [11:0] V_SYNC_1;
    wire [11:0] H_TOTAL;
    wire [11:0] H_FP;
    wire [11:0] H_BP;
    wire [11:0] H_SYNC;
    wire [11:0] HV_OFFSET_0;
    wire [11:0] HV_OFFSET_1;
    wire [19:0] PATTERN_RAMP_STEP;
    wire  [7:0] PATTERN_TYPE;

    assign PATTERN_TYPE = 8'd6;
    
    format_vg format_vg (
        .FORMAT(VIDEO_FORMAT),
        .INTERLACED(INTERLACED),
        .V_TOTAL_0(V_TOTAL_0),
        .V_FP_0(V_FP_0),
        .V_BP_0(V_BP_0),
        .V_SYNC_0(V_SYNC_0),
        .V_TOTAL_1(V_TOTAL_1),
        .V_FP_1(V_FP_1),
        .V_BP_1(V_BP_1),
        .V_SYNC_1(V_SYNC_1),
        .H_TOTAL(H_TOTAL),
        .H_FP(H_FP),
        .H_BP(H_BP),
        .H_SYNC(H_SYNC),
        .HV_OFFSET_0(HV_OFFSET_0),
        .HV_OFFSET_1(HV_OFFSET_1),
        .PATTERN_RAMP_STEP(PATTERN_RAMP_STEP)
    );

    sync_vg #(.X_BITS(12), .Y_BITS(12)) sync_vg (
        .clk(HDMI_TX_CLK),
        .reset(RESET),
        .interlaced(INTERLACED),
        .clk_out(), // inverted output clock - unconnected
        .v_total_0(V_TOTAL_0),
        .v_fp_0(V_FP_0),
        .v_bp_0(V_BP_0),
        .v_sync_0(V_SYNC_0),
        .v_total_1(V_TOTAL_1),
        .v_fp_1(V_FP_1),
        .v_bp_1(V_BP_1),
        .v_sync_1(V_SYNC_1),
        .h_total(H_TOTAL),
        .h_fp(H_FP),
        .h_bp(H_BP),
        .h_sync(H_SYNC),
        .hv_offset_0(HV_OFFSET_0),
        .hv_offset_1(HV_OFFSET_1),
        .de_out(de),
        .vs_out(vs),
        .v_count_out(),
        .h_count_out(),
        .x_out(x_out),
        .y_out(y_out),
        .hs_out(hs),
        .field_out(field)
    );

    pattern_vg #(
        .B(8), // Bits per channel
        .X_BITS(12),
        .Y_BITS(12),
        .FRACTIONAL_BITS(12)) // Number of fractional bits for ramp pattern
    pattern_vg (
        .reset(RESET),
        .clk_in(HDMI_TX_CLK),
        .x(x_out),
        .y(y_out[11:0]),
        .vn_in(vs),
        .hn_in(hs),
        .dn_in(de),
        .r_in(8'h0), // default red channel value
        .g_in(8'h0), // default green channel value
        .b_in(8'h0), // default blue channel value
        .vn_out(vs_out),
        .hn_out(hs_out),
        .den_out(de_out),
        .r_out(r_out),
        .g_out(g_out),
        .b_out(b_out),
        .total_active_pix(H_TOTAL - (H_FP + H_BP + H_SYNC)), // (1920) // h_total - (h_fp+h_bp+h_sync)
        .total_active_lines(INTERLACED ?
            (V_TOTAL_0 - (V_FP_0 + V_BP_0 + V_SYNC_0)) + (V_TOTAL_1 - (V_FP_1 + V_BP_1 + V_SYNC_1)) :
            (V_TOTAL_0 - (V_FP_0 + V_BP_0 + V_SYNC_0)) - 1), // originally: 13'd480
        .pattern(PATTERN_TYPE),
        .ramp_step(PATTERN_RAMP_STEP)
    );

    adv7513_init #(
        .CHIP_ADDR(ADV7513_CHIP_ADDR),
        .I2C_CLKDIV(I2C_CLKDIV)
    ) adv7513_init (
        .clk(CLOCK_50_B5B),
        .reset(RESET),
        .scl(I2C_SCL),
        .sda(I2C_SDA),
        .start(adv7513_init_start),
        .done(adv7513_init_done)
    );

    adv7513_reg_read #(
        .CHIP_ADDR(ADV7513_CHIP_ADDR),
        .I2C_CLKDIV(I2C_CLKDIV)
    ) adv7513_reg_read (
        .clk(CLOCK_50_B5B),
        .reset(RESET),
        .scl(I2C_SCL),
        .sda(I2C_SDA),
        .reg_addr_in(I2C_REG),
        .reg_data(I2C_REG_DATA),
        .start(adv7513_reg_read_start),
        .done(adv7513_reg_read_done)
    );

    led_7seg sseg_l(
        .en(sseg_en),
        .dp(1'b0),
        .hex(I2C_REG_DATA[3:0]),
        .sseg(SSEG_OUT[6:0])
    );

    led_7seg sseg_h(
        .en(sseg_en),
        .dp(1'b0),
        .hex(I2C_REG_DATA[7:4]),
        .sseg(SSEG_OUT[13:7])
    );

    led_7seg sseg_state(
        .en(sseg_en),
        .dp(1'b0),
        .hex(state[3:0]),
        .sseg(SSEG_OUT[20:14])
    );
    
    slow_input_flop slow_input_flop (
        .clk(CLOCK_50_B5B),
        .rst(RESET),
        .in(I2C_REG_READ),
        .out(reg_read)
    );

    always @ (posedge /*clk_1us*/CLOCK_50_B5B) begin
        if (~RESET) begin
            delay_done <= 1'b0;
            delay_tick <= 29'd0;
        end
        else begin
            delay_done <= (
                (state == s_idle) ||
                (state == s_startup                && delay_tick == ADV7513_INIT_DELAY) ||
                //(state == s_adv7513_init_start/*     && delay_tick == 1*/) ||
                (state == s_adv7513_init_wait      && delay_tick == ADV7513_INIT_DELAY) ||
                //(state == s_adv7513_reg_read_start/* && delay_tick == 1*/) ||
                (state == s_adv7513_reg_read_wait  && delay_tick == I2C_TXN_DELAY)
            ) ? 1'b1 : 1'b0;

            delay_tick <= (delay_done == 1'b1) ? 29'd0 : delay_tick + 29'b1;
        end
    end

    always @ (posedge CLOCK_50_B5B) begin
        if (~RESET) begin
            state <= s_startup;

            sseg_en <= 1'b0;

            adv7513_init_start     <= 1'b0;
            adv7513_reg_read_start <= 1'b0;

            LEDR <= 10'b10_0000_0000;
            LEDG <= 8'h00;
        end
        else begin
            sseg_en <= 1'b1;

            LEDR[9] <= 1'b0;
            LEDR[8] <= pll_locked;
            LEDR[7] <= HDMI_TX_INT;

            LEDG[0] <= adv7513_init_start;
            LEDG[1] <= adv7513_init_done;

            LEDG[2] <= adv7513_reg_read_start;
            LEDG[3] <= adv7513_reg_read_done;

            case (state)
                s_idle: begin
                    if (~reg_read) begin
                        state <= s_adv7513_reg_read_start;
                    end
                    else begin
                        state <= s_idle;
                    end
                end

                s_startup: begin
                    state <= delay_done ? s_adv7513_init_start : s_startup;
                end

                s_adv7513_init_start: begin
                    adv7513_init_start <= 1'b1;
                    state <= s_adv7513_init_wait;
                end

                s_adv7513_init_wait: begin
                    adv7513_init_start <= 1'b0;
                    state <= (delay_done /*&& adv7513_init_done*/) ? s_idle : s_adv7513_init_wait;
                end

                s_adv7513_reg_read_start: begin
                    adv7513_reg_read_start <= 1'b1;
                    state <= s_adv7513_reg_read_wait;
                end

                s_adv7513_reg_read_wait: begin
                    adv7513_reg_read_start <= 1'b0;
                    state <= (delay_done /*&& adv7513_reg_read_done*/) ? s_idle : s_adv7513_reg_read_wait;
                end
            endcase
        end
    end
endmodule