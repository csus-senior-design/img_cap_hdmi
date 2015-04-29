module clk_1us # (
        parameter CLKDIV = 50
    )(
        input clk_in,
        input reset,
        output reg clk_out
    );

    reg [7:0] counter;

    always @ (posedge clk_in, negedge reset) begin
        if (~reset) begin
            clk_out <= 1'b0;
            counter <= 8'h00;
        end
        else begin
            counter <= counter + 1'b1;

            if (counter == CLKDIV) begin
                counter <= 8'h00;
                clk_out <= ~clk_out;
            end
        end
    end
endmodule
