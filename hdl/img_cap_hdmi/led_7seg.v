module led_7seg
    (
        input en,
        input [3:0] hex,
        input dp,
        output reg [7:0] sseg
    );

    always @ (*) begin
        if (en == 1'b1) begin
            case (hex)
                 4'h0: sseg[6:0] = ~7'b0111111;
                 4'h1: sseg[6:0] = ~7'b0000110;
                 4'h2: sseg[6:0] = ~7'b1011011;
                 4'h3: sseg[6:0] = ~7'b1001111;
                 4'h4: sseg[6:0] = ~7'b1100110;
                 4'h5: sseg[6:0] = ~7'b1101101;
                 4'h6: sseg[6:0] = ~7'b1111101;
                 4'h7: sseg[6:0] = ~7'b0000111;
                 4'h8: sseg[6:0] = ~7'b1111111;
                 4'h9: sseg[6:0] = ~7'b1100111;
                 4'hA: sseg[6:0] = ~7'b1110111;
                 4'hB: sseg[6:0] = ~7'b1111100;
                 4'hC: sseg[6:0] = ~7'b0111001;
                 4'hD: sseg[6:0] = ~7'b1011110;
                 4'hE: sseg[6:0] = ~7'b1111001;
                 4'hF: sseg[6:0] = ~7'b1110001;
            endcase

            sseg[7] = dp;
        end
        else begin
            sseg = 8'hff;
        end
    end
endmodule
