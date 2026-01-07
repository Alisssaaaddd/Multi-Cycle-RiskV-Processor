module alu_control (
    input [1:0] aluop,      // Control signal from Main Control
    input [3:0] opcode,     // Opcode field from Instruction Register
    input [8:0] func,       // Function field (9-bit) for Type-C instructions
    output reg [2:0] alucontrol // Final 3-bit control signal for the ALU
);
    always @(*) begin
        case (aluop)
            // Force ADD for PC increment and Memory Address calculation
            // Special cases: Jump and BranchZ use pass-through (SrcB) to pass immediate
            2'b00: alucontrol = ((opcode == 4'b0010) || (opcode == 4'b0100)) ? 3'b110 : 3'b000; 

            // Force SUB for zero-flag comparison (BranchZ)
            2'b01: alucontrol = 3'b001; 

            // Type-C Instructions: Decode operation based on Func bits
            2'b10: begin                
                if (func[2])      alucontrol = 3'b000; // Add
                else if (func[3]) alucontrol = 3'b001; // Sub
                else if (func[4]) alucontrol = 3'b010; // And
                else if (func[5]) alucontrol = 3'b011; // Or
                else if (func[6]) alucontrol = 3'b100; // Not
                else if (func[1]) alucontrol = 3'b110; // MoveFrom (Pass B)
                else if (func[0]) alucontrol = 3'b101; // MoveTo (Pass A)
                else              alucontrol = 3'b000; // Default: Add
            end

            // Type-D Instructions: Decode operation based on Opcode
            2'b11: begin                
                if (opcode == 4'b1100)      alucontrol = 3'b000; // Addi
                else if (opcode == 4'b1101) alucontrol = 3'b001; // Subi
                else if (opcode == 4'b1110) alucontrol = 3'b010; // Andi
                else if (opcode == 4'b1111) alucontrol = 3'b011; // Ori
                else                        alucontrol = 3'b000; // Default: Add
            end

            default: alucontrol = 3'b000;
        endcase
    end
endmodule