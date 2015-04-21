module clk_1us(clk_in, reset, clk_out);
    input clk_in;
    input reset;
    output reg clk_out;

    reg [7:0] counter;

    always @ (posedge clk_in, negedge reset) begin
        if (~reset) begin
            clk_out <= 1'b0;
            counter <= 8'h00;
        end
        else begin
            counter <= counter + 1'b1;

            if (counter == 8'b01111101) begin
                counter <= 8'h00;
                clk_out <= ~clk_out;
            end
        end
    end
endmodule
