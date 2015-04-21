`timescale 1ns / 1ps
`include "../hdl/i2c/i2c_master.v"
`include "../hdl/i2c/i2c_slave.v"

`include "../hdl/adv7513/adv7513_init.v"
`include "../hdl/adv7513/adv7513_reg_read.v"
`include "../hdl/videogen/sync_vg.v"
`include "../hdl/videogen/pattern_vg.v"

`include "../hdl/cam_test/clk_1us.v"
`include "../hdl/cam_test/led_7seg.v"

`include "../hdl/cam_test/cam_test.v"

//`include "adv7513_mock.v"

/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

Cam Test Testbench

Authors:  Greg M. Crist, <Jr. (gmcrist@gmail.com)

Description:
  testbench for cam test
*/
module cam_test_tb ();
    parameter CLOCKPERIOD = 20;

    reg CLOCK_125_p,
        CLOCK_50_B5B,
        CLOCK_50_B6A,
        CLOCK_50_B7A,
        CLOCK_50_B8A;

    reg reset;

    wire HDMI_TX_CLK,
         HDMI_TX_DE,
         HDMI_TX_HS,
         HDMI_TX_VS;

    wire [23:0] HDMI_TX_D;

    reg HDMI_TX_INT;

    wire I2C_SCL, I2C_SDA;

    reg [7:0] I2C_REG;
    wire [23:0] SSEG_OUT;
    reg i2c_reg_read;



    // for counting the cycles
    reg [15:0] cycle;

    reg clock;

//    pullup(I2C_SDA);
//    pullup(I2C_SCL);

    adv7513_mock adv7513_mock_inst (
        .clock(clock),
        .reset(reset),
        .SDA(I2C_SDA),
        .SCL(I2C_SCL)
    );

    cam_test uut (
        .CLOCK_125_p(clock),
        .CLOCK_50_B5B(CLOCK_50_B5B),
        .CLOCK_50_B6A(CLOCK_50_B6A),
        .CLOCK_50_B7A(CLOCK_50_B7A),
        .CLOCK_50_B8A(CLOCK_50_B8A),

        .reset(reset),

        .HDMI_TX_CLK(HDMI_TX_CLK),
        .HDMI_TX_DE(HDMI_TX_DE),
        .HDMI_TX_HS(HDMI_TX_HS),
        .HDMI_TX_VS(HDMI_TX_VS),

        .HDMI_TX_D(HDMI_TX_D),
        .HDMI_TX_INT(HDMI_TX_INT),

        .I2C_SCL(I2C_SCL),
        .I2C_SDA(I2C_SDA),

        .I2C_REG(I2C_REG),
        .SSEG_OUT(SSEG_OUT),
        .i2c_reg_read(i2c_reg_read)
    );


    // Initial conditions; setup
    initial begin
        $timeformat(-9,1, "ns", 12);
        $monitor("%t, Tick: %d, Init: %b, State: %d", $realtime, uut.delay_tick, uut.adv7513_init_done, uut.state);

        // Initial Conditions
        cycle <= 0;
        reset <= 1'b0;

        #2 clock <= 0;

        #5 reset <= 1'b1;
        $display("De-asserting reset");
    end

    always @ (posedge clock) begin
        if (uut.adv7513_init_done == 1) begin
            #100 I2C_REG <= 8'h0E;
            #100 i2c_reg_read <= 1'b1;
            #100;
        end

        if (uut.state == 0) begin
            #100 $finish;
        end
    end


/**************************************************************/
/* The following can be left as-is unless necessary to change */
/**************************************************************/

    // Cycle Counter
    always @ (posedge clock)
        cycle <= cycle + 1;

    // Clock generation
    always #(CLOCKPERIOD / 2) clock <= ~clock;

/*
  Conditional Environment Settings for the following:
    - Icarus Verilog
    - VCS
    - Altera Modelsim
    - Xilinx ISIM
*/
// Icarus Verilog
`ifdef IVERILOG
    initial $dumpfile("vcdbasic.vcd");
    initial $dumpvars();
`endif

// VCS
`ifdef VCS
    initial $vcdpluson;
`endif

// Altera Modelsim
`ifdef MODEL_TECH
`endif

// Xilinx ISIM
`ifdef XILINX_ISIM
`endif
endmodule
