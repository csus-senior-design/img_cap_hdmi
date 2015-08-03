module slow_input_flop #(
    parameter   DEF_VAL = 1'b1,
                `ifdef SIM
                    WAIT_TIME = 26'd10,
                `else
                    WAIT_TIME = 26'd50000000,
                `endif
                HOLD_CYCLES = 26'd2
)(
    input       clk,
                rst,
                in,
    output  reg out = DEF_VAL
);

    reg [25:0]  cnt = 26'd0;
    
    always @(posedge clk) begin
        out <= DEF_VAL;
        
        if (~rst || cnt == WAIT_TIME || in == DEF_VAL)
            cnt <= 26'd0;
        else if (cnt <= HOLD_CYCLES && in == ~DEF_VAL) begin
            out <= in;
            cnt <= cnt + 26'd1;
        end else if (in == ~DEF_VAL)
            cnt <= cnt + 26'd1;
    end

endmodule