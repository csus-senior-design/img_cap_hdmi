`include "slow_input_flop.v"
`timescale 1ns / 1ns

module slow_input_flop_tb;

    reg     clk,
            rst,
            in;
    wire    out;

    slow_input_flop slow_input_flop (
        .clk(clk),
        .rst(rst),
        .in(in),
        .out(out)
    );

    always #10 clk = ~clk;

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars();
    end

    initial begin
        clk = 1'b0;
        rst = 1'b1;
        in = 1'b1;
        
        #20 rst = 1'b0;
        
        #20 in = 1'b0;
        
        #160 in = 1'b1;
        
        #40 $finish;
    end

endmodule