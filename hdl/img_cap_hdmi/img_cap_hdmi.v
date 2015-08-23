/*
-------------------------------------------------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
-------------------------------------------------------------------------------

ADV7513 Driver
Authors:	Padraic Hagerty (guitarisrockin@hotmail.com)
				Greg M. Crist, Jr. (gmcrist@gmail.com)

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

module img_cap_hdmi #(
	// for testbenching
	`ifdef SIM
		parameter ADV7513_INIT_DELAY = 29'd250, // 250ms
	`else
		//parameter ADV7513_INIT_DELAY = 29'd25000000, // 500ms with 50MHz clock
		parameter ADV7513_INIT_DELAY = 29'd12600000, // 500ms with 25.2MHz clock
		//parameter ADV7513_INIT_DELAY = 29'd37125000, // 500ms with 74.25MHz clock
	`endif

	parameter ADV7513_CHIP_ADDR = 7'h39,    // 0x72 >> 1

	//parameter I2C_CLKDIV = 12'd125,			// This value is multiplied by four
																// somewhere, so 125 divides by 500
																// for a 100kHz clock based on a 50MHz
																// reference clock
	parameter I2C_CLKDIV = 12'd63,			// Gives a 100kHz clock based on a 25.2MHz
																// clock
	//parameter I2C_CLKDIV = 12'd186,			// Gives a 100kHz clock based on a 74.25MHz
																// clock
	//parameter I2C_TXN_DELAY = 29'd30000		// 600µs with 50MHz clock
	parameter I2C_TXN_DELAY = 29'd15120		// 600µs with 25.2MHz clock
	//parameter I2C_TXN_DELAY = 29'd44544		// 600µs with 74.25MHz clock
)(
	// Clock signal
	input					pix_clk,	// 25.2MHz for 640x480p, 74.25MHz for 1280x720p
	
	// Synchronous reset
	input					reset,

	// HDMI-TX via ADV7513
	output				HDMI_TX_DE,
	output				HDMI_TX_HS,
	output				HDMI_TX_VS,

	// External I2C bus for HDMI-TX
	inout					I2C_SCL,
	inout					I2C_SDA,

	// I2C read register control signal, register address, and output data
	input					i2c_reg_read,
	input		[7:0]		i2c_reg_addr,
	output		[7:0]		i2c_reg_data
	/*(*
	  chip_pin = "W20, W21, V20, V22, U20, AD6, AD7, AF24, AC19, AE25, AE26, AB19, AD26, AA18, Y18, Y19, Y20, W18, V17, V18, V19"
	*)
	output [20:0] SSEG_OUT,

	// LED Status Indicators
	(*
	  chip_pin = "J10, H7, K8, K10, J7, J8, G7, G6, F6, F7"
	*)
	output reg [9:0] LEDR,
	(*
	  chip_pin = "H9, H8, B6, A5, E9, D8, K6, L7"
	*)
	output reg [7:0] LEDG*/
	
);

	wire de;
	wire vs;
	wire hs;
	wire field = 1'b0;

	reg adv7513_init_start;
	wire adv7513_init_done;

	reg adv7513_reg_read_start;
	wire adv7513_reg_read_done;

	//reg sseg_en;

	wire [11:0] x_out;
	wire [12:0] y_out;
	wire [7:0] r_out;
	wire [7:0] g_out;
	wire [7:0] b_out;

	localparam	[3:0]
		s_idle = 4'h0,
		s_startup = 4'h1,
		s_adv7513_init_start = 4'h2,
		s_adv7513_init_wait = 4'h3,
		s_adv7513_reg_read_start = 4'h4,
		s_adv7513_reg_read_wait= 4'h5;

	(* syn_encoding = "safe" *)
	reg [3:0] state;

	reg [28:0] delay_tick;
	reg delay_done;

	`ifdef SIM
		assign HDMI_TX_CLK = pix_clk;
		assign pll_locked = 1'b1;
	`endif

	//assign HDMI_TX_D  = {r_out, g_out, b_out};
	//assign HDMI_TX_DE = de_out;
	//assign HDMI_TX_HS = hs_out;
	//assign HDMI_TX_VS = vs_out;

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
		.clk(pix_clk),
		.reset(reset),
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
		.de_out(HDMI_TX_DE),
		.vs_out(HDMI_TX_VS),
		.v_count_out(),
		.h_count_out(),
		.x_out(x_out),
		.y_out(y_out),
		.hs_out(HDMI_TX_HS),
		.field_out(field)
	);

	/*pattern_vg #(
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
	);*/

	adv7513_init #(
		.CHIP_ADDR(ADV7513_CHIP_ADDR),
		.I2C_CLKDIV(I2C_CLKDIV)
	) adv7513_init (
		.clk(pix_clk),
		.reset(reset),
		.scl(I2C_SCL),
		.sda(I2C_SDA),
		.start(adv7513_init_start),
		.done(adv7513_init_done)
	);

	adv7513_reg_read #(
		.CHIP_ADDR(ADV7513_CHIP_ADDR),
		.I2C_CLKDIV(I2C_CLKDIV)
	) adv7513_reg_read (
		.clk(pix_clk),
		.reset(reset),
		.scl(I2C_SCL),
		.sda(I2C_SDA),
		.reg_addr_in(i2c_reg_addr),
		.reg_data(i2c_reg_data),
		.start(adv7513_reg_read_start),
		.done(adv7513_reg_read_done)
	);

	/*led_7seg sseg_l(
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
	);*/

	always @ (posedge pix_clk) begin
		if (~reset) begin
			delay_done <= 1'b0;
			delay_tick <= 29'd0;
		end
		else begin
			delay_done <= (
				(state == s_idle) ||
				(state == s_startup                && delay_tick == ADV7513_INIT_DELAY) ||
				(state == s_adv7513_init_wait      && delay_tick == ADV7513_INIT_DELAY) ||
				(state == s_adv7513_reg_read_wait  && delay_tick == I2C_TXN_DELAY)
			) ? 1'b1 : 1'b0;

			delay_tick <= (delay_done == 1'b1) ? 29'd0 : delay_tick + 29'b1;
		end
	end

	always @ (posedge pix_clk) begin
		if (~reset) begin
			state <= s_startup;

			//sseg_en <= 1'b0;

			adv7513_init_start     <= 1'b0;
			adv7513_reg_read_start <= 1'b0;

			//LEDR <= 10'b10_0000_0000;
			//LEDG <= 8'h00;
		end
		else begin
			/*sseg_en <= 1'b1;

			LEDR[9] <= 1'b0;
			LEDR[8] <= pll_locked;
			LEDR[7] <= HDMI_TX_INT;

			LEDG[0] <= adv7513_init_start;
			LEDG[1] <= adv7513_init_done;

			LEDG[2] <= adv7513_reg_read_start;
			LEDG[3] <= adv7513_reg_read_done;*/

			case (state)
				s_idle: begin
					if (i2c_reg_read) begin
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
					state <= (delay_done) ? s_idle : s_adv7513_init_wait;
				end

				s_adv7513_reg_read_start: begin
					adv7513_reg_read_start <= 1'b1;
					state <= s_adv7513_reg_read_wait;
				end

				s_adv7513_reg_read_wait: begin
					adv7513_reg_read_start <= 1'b0;
					state <= (delay_done) ? s_idle : s_adv7513_reg_read_wait;
				end
			endcase
		end
	end
endmodule