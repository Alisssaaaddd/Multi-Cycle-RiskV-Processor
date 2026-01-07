module ALU (
    input [15:0] SrcA, SrcB,
    input [2:0] ALUControl,
    output zero,
    output reg [15:0] ALUResult
);
    always @(*) begin 
        case (ALUControl) 
           3'b000: ALUResult = SrcA + SrcB;
           3'b001: ALUResult = SrcA - SrcB;
           3'b010: ALUResult = SrcA & SrcB;
           3'b011: ALUResult = SrcA | SrcB;
           3'b100: ALUResult = ~SrcA;
           3'b101: ALUResult = SrcA;
           3'b110: ALUResult = SrcB;
           default: ALUResult = 16'b0;
        endcase
    end
    assign zero = (ALUResult == 0);
endmodule