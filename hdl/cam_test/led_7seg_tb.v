`timescale 1ns / 1ps
/*
----------------------------------------
Stereoscopic Vision System
Senior Design Project - Team 11
California State University, Sacramento
Spring 2015 / Fall 2015
----------------------------------------

Testbench for 7 Segment Interface

Authors:  Greg M. Crist, Jr. (gmcrist@gmail.com)

Description:

*/
module led_7seg_tb ();
    parameter CLOCKPERIOD = 10;

    reg reset;
    reg clock;

    reg [3:0] hex;
    reg en;

    wire [7:0] sseg;

    // for counting the cycles
    reg [15:0] cycle;

    led_7seg uut (.en(en), .dp(dp), .hex(hex), .sseg(sseg));

    // Initial conditions; setup
    initial begin
        $timeformat(-9,1, "ns", 12);
        $monitor("%t, HEX: %h, SSEG: %b", $realtime, hex, sseg);

		// Initial Conditions
		clock <= 0;
        cycle <= 0;
        reset <= 1'b0;

        en <= 1'b0;
        hex <= 4'h0;


        // Initialize clock
        #2
        clock <= 1'b0;

		// Deassert reset
        #100
        reset <= 1'b1;

        en <= 1'b1;

        #5

        while (hex < 4'hf) begin
            #20 hex <= hex + 1'b1;
        end


        #100 $finish;
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
