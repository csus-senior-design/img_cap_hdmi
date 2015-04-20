module cam_test(
    // Clock signals
    input   CLOCK_125_p,
    input   CLOCK_50_B5B,
    input   CLOCK_50_B6A,
    input   CLOCK_50_B7A,
    input   CLOCK_50_B8A,

    input   reset,

    // HDMI-TX via ADV7513
	output  HDMI_TX_CLK,
	output  HDMI_TX_DE,
	output  HDMI_TX_HS,
	output  HDMI_TX_VS,
	output  [23:0] HDMI_TX_D,
	input   HDMI_TX_INT,

    // i2c for HDMI-TX
    inout  I2C_SCL,
    inout  I2C_SDA,

    input  [7:0]  I2C_REG,
    output [15:0] SSEG_OUT,

//    inout [35:0] camGPIO


    // User interface
    input i2c_reg_read

);
    localparam I2C_CLKDIV = 206;

    wire de;
    wire vs;
    wire hs;
    wire field = 1'b0;
    wire vs_out;
    wire hs_out;
    wire de_out;

    wire clk_in;

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
    
    assign clk_in      = ~CLOCK_125_p;
    assign HDMI_TX_CLK = ~CLOCK_125_p;

    assign HDMI_TX_D   = {r_out, g_out, b_out};
    assign HDMI_TX_DE  = de_out;
    assign HDMI_TX_HS  = hs_out;
    assign HDMI_TX_VS = vs_out;



    always @(posedge clk_in) begin
        // Do something
    end

    //`define MODE_1080p
    //`define MODE_1080i
    `define MODE_720p

    `ifdef MODE_1080p /* FORMAT 16 */
        parameter INTERLACED = 1'b0;
        parameter V_TOTAL_0 = 12'd1125;
        parameter V_FP_0 = 12'd4;
        parameter V_BP_0 = 12'd36;
        parameter V_SYNC_0 = 12'd5;
        parameter V_TOTAL_1 = 12'd0;
        parameter V_FP_1 = 12'd0;
        parameter V_BP_1 = 12'd0;
        parameter V_SYNC_1 = 12'd0;
        parameter H_TOTAL = 12'd2200;
        parameter H_FP = 12'd88;
        parameter H_BP = 12'd148;
        parameter H_SYNC = 12'd44;
        parameter HV_OFFSET_0 = 12'd0;
        parameter HV_OFFSET_1 = 12'd0;
        parameter PATTERN_RAMP_STEP = 20'h0222;
        parameter PATTERN_TYPE = 8'd4; // RAMP
        //parameter PATTERN_TYPE = 8'd1; // OUTLINE
    `endif

    `ifdef MODE_1080i /* FORMAT 5 */
        parameter INTERLACED = 1'b1;
        parameter V_TOTAL_0 = 12'd562;
        parameter V_FP_0 = 12'd2;
        parameter V_BP_0 = 12'd15;
        parameter V_SYNC_0 = 12'd5;
        parameter V_TOTAL_1 = 12'd563;
        parameter V_FP_1 = 12'd2;
        parameter V_BP_1 = 12'd16;
        parameter V_SYNC_1 = 12'd5;
        parameter H_TOTAL = 12'd2200;
        parameter H_FP = 12'd88;
        parameter H_BP = 12'd148;
        parameter H_SYNC = 12'd44;
        parameter HV_OFFSET_0 = 12'd0;
        parameter HV_OFFSET_1 = 12'd1100;
        parameter PATTERN_RAMP_STEP = 20'h0222; // 20'hFFFFF / 1920 act_pixels per line = 20'h0222
        parameter PATTERN_TYPE = 8'd4; // RAMP
        //parameter PATTERN_TYPE = 8'd1; // OUTLINE
    `endif

    `ifdef MODE_720p /* FORMAT 4 */
        parameter INTERLACED = 1'b0;
        parameter V_TOTAL_0 = 12'd750;
        parameter V_FP_0 = 12'd5;
        parameter V_BP_0 = 12'd20;
        parameter V_SYNC_0 = 12'd5;
        parameter V_TOTAL_1 = 12'd0;
        parameter V_FP_1 = 12'd0;
        parameter V_BP_1 = 12'd0;
        parameter V_SYNC_1 = 12'd0;
        parameter H_TOTAL = 12'd1650;
        parameter H_FP = 12'd110;
        parameter H_BP = 12'd220;
        parameter H_SYNC = 12'd40;
        parameter HV_OFFSET_0 = 12'd0;
        parameter HV_OFFSET_1 = 12'd0;
        parameter PATTERN_RAMP_STEP = 20'h0333; // 20'hFFFFF / 1280 act_pixels per line = 20'h0333
        //parameter PATTERN_TYPE = 8'd1; // BORDER.
        parameter PATTERN_TYPE = 8'd4; // RAMP
    `endif

    /* ********************* */
    sync_vg #(.X_BITS(12), .Y_BITS(12)) sync_vg (
        .clk(clk_in),
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
        .reset(reset),
        .clk_in(clk_in),
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
        .total_active_lines(INTERLACED ? (V_TOTAL_0 - (V_FP_0 + V_BP_0 + V_SYNC_0)) + (V_TOTAL_1 - (V_FP_1 + V_BP_1 + V_SYNC_1)) : (V_TOTAL_0 - (V_FP_0 + V_BP_0 + V_SYNC_0))), // originally: 13'd480
        .pattern(PATTERN_TYPE),
        .ramp_step(PATTERN_RAMP_STEP)
    );

    adv7513_init adv7513_init(
        .clk(clk_in),
        .reset(reset),
        .clk_div(I2C_CLKDIV),
        .scl(I2C_SCL),
        .sda(I2C_SDA),
        .start(adv7513_init_start),
        .done(adv7513_init_done)
    );

    adv7513_reg_read adv7513_reg_read(
        .clk(clk_in),
        .reset(reset),
        .clk_div(I2C_CLKDIV),
        .scl(I2C_SCL),
        .sda(I2C_SDA),
        .reg_addr(I2C_REG),
        .reg_data(I2C_REG_DATA),
        .start(adv7513_reg_read_start),
        .done(adv7513_reg_read_done)
    );

    led_7seg sseg_h(
        .en(sseg_en),
        .dp(1'b0),
        .hex(I2C_REG_DATA[7:4]),
        .sseg(SSEG_OUT[15:8])
    );

    led_7seg sseg_l(
        .en(sseg_en),
        .dp(1'b0),
        .hex(I2C_REG_DATA[3:0]),
        .sseg(SSEG_OUT[7:0])
    );

    
    localparam s_idle             = 0, 
               s_startup          = 1,
               s_adv7513_init     = 2,
               s_adv7513_reg_read = 3;

    reg [3:0] state;
    
    
    wire clk_1us;
    reg [31 :0] delay_tick;
    wire delay_done;
    
    // Clock dividier to get 1us clock
    clk_1us clk_1us_inst (clk_in, clk_1us);

    assign delay_done = (
        state == s_adv7513_init && delay_tick == 32'd200000 // 200ms
    ) ? 1'b1 : 1'b0;
    
    always @ (posedge clk_1us, negedge reset) begin
        if (~reset) begin
            delay_tick <= 32'd0;
        end else begin
            delay_tick <= delay_done == 1'b1 ? 32'd0 : delay_tick + 1'b1;
        end
    end
    
    always @ (posedge clk_in, negedge reset) begin
        if (~reset) begin
            state <= s_startup;

            sseg_en <= 1'b0;

            adv7513_init_start     <= 1'b0;
            adv7513_reg_read_start <= 1'b0;
            
        end
        else begin
            sseg_en <= 1'b1;
            
            case (state)
                s_idle: begin
                    if (i2c_reg_read) begin
                        state <= s_adv7513_reg_read;
                    end
                    else begin
                        state <= s_idle;
                    end
                end
                
                s_startup: begin
                    state <= delay_done ? s_adv7513_init : s_startup;
                end
                
                s_adv7513_init: begin
                    adv7513_init_start <= 1'b1;
                    
                    state <= adv7513_init_done ? s_idle : s_adv7513_init;
                end
                
                s_adv7513_reg_read: begin
                    adv7513_reg_read_start <= 1'b1;

                    state <= adv7513_reg_read_done ? s_idle : s_adv7513_reg_read;
                end
            endcase
        end
    end
endmodule

