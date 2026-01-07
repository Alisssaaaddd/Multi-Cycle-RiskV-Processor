module Mux3to1 (
    input [15:0] a, b, c,
    input [1:0] sel,
    output reg [15:0] out
);
    always @(*) begin
        case (sel)
            2'b00: out = a;
            2'b01: out = b;
            2'b10: out = c;
        endcase
    end        
endmodule