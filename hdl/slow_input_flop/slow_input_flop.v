module slow_input_flop #(
    parameter   DEF_VAL = 1'b1,
                `ifdef SIM
                    WAIT_TIME = 20'd10,
                `else
                    WAIT_TIME = 20'd1000000,
                `endif
                HOLD_CYCLES = 20'd2
)(
    input       clk,
                rst,
                in,
    output  reg out = DEF_VAL
);

    reg [19:0]  cnt = 20'd0;
    
    always @(posedge clk) begin
        out <= DEF_VAL;
        
        if (~rst || cnt == WAIT_TIME) begin
            cnt <= 19'd0;
        end else if (cnt <= HOLD_CYCLES && in == ~DEF_VAL) begin
            out <= in;
            cnt <= cnt + 20'd1;
        end else if (in == ~DEF_VAL)
            cnt <= cnt + 20'd1;
    end

endmodule