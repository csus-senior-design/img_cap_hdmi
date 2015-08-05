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

        parameter I2C_CLKDIV = 12'd125,
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

        // i2c for HDMI-TX
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

        // GPIO for Camera Interfaces
        (*
          chip_pin = "D26, T21"
        *)
        output  [1:0] camGPIO,

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

    //assign camGPIO[0] = I2C_SCL;
    //assign camGPIO[1] = I2C_SDA;

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

    //`define MODE_1080p
    //`define MODE_1080i
    `define MODE_720p

    `ifdef MODE_1080p /* FORMAT 16 */
        localparam INTERLACED = 1'b0;
        localparam V_TOTAL_0 = 12'd1125;
        localparam V_FP_0 = 12'd4;
        localparam V_BP_0 = 12'd36;
        localparam V_SYNC_0 = 12'd5;
        localparam V_TOTAL_1 = 12'd0;
        localparam V_FP_1 = 12'd0;
        localparam V_BP_1 = 12'd0;
        localparam V_SYNC_1 = 12'd0;
        localparam H_TOTAL = 12'd2200;
        localparam H_FP = 12'd88;
        localparam H_BP = 12'd148;
        localparam H_SYNC = 12'd44;
        localparam HV_OFFSET_0 = 12'd0;
        localparam HV_OFFSET_1 = 12'd0;
        localparam PATTERN_RAMP_STEP = 20'h0222;
        localparam PATTERN_TYPE = 8'd4; // RAMP
        //localparam PATTERN_TYPE = 8'd1; // OUTLINE
    `endif

    `ifdef MODE_1080i /* FORMAT 5 */
        localparam INTERLACED = 1'b1;
        localparam V_TOTAL_0 = 12'd562;
        localparam V_FP_0 = 12'd2;
        localparam V_BP_0 = 12'd15;
        localparam V_SYNC_0 = 12'd5;
        localparam V_TOTAL_1 = 12'd563;
        localparam V_FP_1 = 12'd2;
        localparam V_BP_1 = 12'd16;
        localparam V_SYNC_1 = 12'd5;
        localparam H_TOTAL = 12'd2200;
        localparam H_FP = 12'd88;
        localparam H_BP = 12'd148;
        localparam H_SYNC = 12'd44;
        localparam HV_OFFSET_0 = 12'd0;
        localparam HV_OFFSET_1 = 12'd1100;
        localparam PATTERN_RAMP_STEP = 20'h0222; // 20'hFFFFF / 1920 act_pixels per line = 20'h0222
        localparam PATTERN_TYPE = 8'd4; // RAMP
        //localparam PATTERN_TYPE = 8'd1; // OUTLINE
    `endif

    `ifdef MODE_720p /* FORMAT 4 */
        localparam INTERLACED = 1'b0;
        localparam V_TOTAL_0 = 12'd750;
        localparam V_FP_0 = 12'd5;
        localparam V_BP_0 = 12'd20;
        localparam V_SYNC_0 = 12'd5;
        localparam V_TOTAL_1 = 12'd0;
        localparam V_FP_1 = 12'd0;
        localparam V_BP_1 = 12'd0;
        localparam V_SYNC_1 = 12'd0;
        localparam H_TOTAL = 12'd1650;
        localparam H_FP = 12'd110;
        localparam H_BP = 12'd220;
        localparam H_SYNC = 12'd40;
        localparam HV_OFFSET_0 = 12'd0;
        localparam HV_OFFSET_1 = 12'd0;
        localparam PATTERN_RAMP_STEP = 20'h0333; // 20'hFFFFF / 1280 act_pixels per line = 20'h0333
        //localparam PATTERN_TYPE = 8'd1; // BORDER.
        localparam PATTERN_TYPE = 8'd4; // RAMP
    `endif

    /* ********************* */
    sync_vg #(.X_BITS(12), .Y_BITS(12)) sync_vg (
        .clk(CLOCK_50_B5B),
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
        .clk_in(CLOCK_50_B5B),
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