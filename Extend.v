module extend (
    input  [15:0] instr,      
    output reg [15:0] imm_ext 
);
    always @(*) begin
        case (instr[15:12])
            4'b0100: imm_ext = {{7{instr[8]}}, instr[8:0]};   // BranchZ (9-bit)
            4'b0010: imm_ext = {{4{instr[11]}}, instr[11:0]}; // Jump (12-bit)
            default: imm_ext = {{4{instr[11]}}, instr[11:0]}; // Others
        endcase
    end
endmodule

/*
module extend (
    input  [15:0] instr,      
    output reg [15:0] imm_ext 
);
    wire [3:0] opcode = instr[15:12];

    always @(*) begin
        case (opcode)
            4'b0100: 
                imm_ext = {{7{instr[8]}}, instr[8:0]}; 

            4'b0000, 4'b0001, 4'b0010, 4'b1100, 4'b1101, 4'b1110, 4'b1111: 
                imm_ext = {{4{instr[11]}}, instr[11:0]};

            default: 
                imm_ext = 16'b0;
        endcase
    end
endmodule
*/