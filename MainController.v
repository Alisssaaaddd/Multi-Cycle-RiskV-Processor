module main_control (
    input clk, rst,
    input [3:0] opcode,
    input [8:0] func,
    input zero,
    output reg pwrite, iwrite, regwrite, memwrite,
    output reg adrsrc, memtoreg, alusrca, regdest,
    output reg [1:0] alusrcb,
    output reg [1:0] aluop
);
    reg [3:0] state, next_state;

    parameter FETCH = 4'd0, PCINC = 4'd8, DECODE = 4'd1, EXECUTE = 4'd2, 
              MEM_RD = 4'd3, MEM_WR = 4'd4, WRITEBACK = 4'd5, 
              BRANCH = 4'd6, JUMP = 4'd7;

    always @(posedge clk or posedge rst) begin
        if (rst) state <= FETCH;
        else state <= next_state;
    end

    always @(*) begin
        {pwrite, iwrite, regwrite, memwrite, adrsrc, memtoreg, alusrca, regdest} = 8'b0;
        alusrcb = 2'b00; aluop = 2'b00; next_state = FETCH;

        case (state)
            FETCH: begin
                adrsrc = 0; iwrite = 1; alusrca = 0; alusrcb = 2'b01; aluop = 2'b00;
                next_state = PCINC;
            end

            PCINC: begin
                pwrite = 1; alusrca = 0; alusrcb = 2'b01; // PC+1
                next_state = DECODE;
            end

            DECODE: begin
                case (opcode)
                    4'b0000, 4'b0001: next_state = EXECUTE; // Load/Store
                    4'b0010: next_state = JUMP;            // Jump 
                    4'b0100: next_state = EXECUTE;         // BranchZ (comparation in Execute)
                    4'b1000: next_state = EXECUTE;         // Type-C
                    4'b1100, 4'b1101, 4'b1110, 4'b1111: next_state = EXECUTE; // Type-D
                    default: next_state = FETCH;
                endcase
            end

            EXECUTE: begin
                case (opcode)
                    4'b0100: begin // BranchZ: compare R0 و Ri
                        alusrca = 1; alusrcb = 2'b00; aluop = 2'b01; // Sub
                        next_state = BRANCH;
                    end
                    4'b0000, 4'b0001: begin // Load/Store address
                        alusrca = 1; alusrcb = 2'b10; aluop = 2'b00;
                        next_state = (opcode == 4'b0000) ? MEM_RD : MEM_WR;
                    end
                    4'b1000: begin // Type C
                        alusrca = 1; alusrcb = 2'b00; aluop = 2'b10;
                        next_state = WRITEBACK;
                    end
                    default: begin // Type D
                        alusrca = 1; alusrcb = 2'b10; aluop = 2'b11;
                        next_state = WRITEBACK;
                    end
                endcase
            end

            BRANCH: begin
                if (zero) begin // R0 == Ri
                    alusrca = 0; alusrcb = 2'b10; aluop = 2'b00; // PC = Imm calculation
                    pwrite = 1;
                end
                next_state = FETCH;
            end

            JUMP: begin
                alusrca = 0; alusrcb = 2'b10; aluop = 2'b00; // PC = Imm
                pwrite = 1;
                next_state = FETCH;
            end

            MEM_RD: begin adrsrc = 1; next_state = WRITEBACK; end
            MEM_WR: begin adrsrc = 1; memwrite = 1; next_state = FETCH; end

            WRITEBACK: begin
                regwrite = 1;
                memtoreg = (opcode == 4'b0000);
                // برای MoveFrom (func[1]) مقصد R0 (regdest=0) است
                regdest = (opcode == 4'b1000) ? (func[1] == 1'b0) : 1'b1;
                next_state = FETCH;
            end
        endcase
    end
endmodule


/*
module main_control (
    input clk, rst,
    input [3:0] opcode,
    input [8:0] func,
    input zero,
    output reg pwrite, iwrite, regwrite, memwrite,
    output reg adrsrc, memtoreg, alusrca, regdest,
    output reg [1:0] alusrcb,
    output reg [1:0] immsrc,
    output reg [1:0] aluop
);
    reg [3:0] state, next_state;

    parameter FETCH  = 4'd0, PCINC  = 4'd8, DECODE = 4'd1, EXECUTE = 4'd2, 
              MEM_RD = 4'd3, MEM_WR = 4'd4, WRITEBACK = 4'd5, 
              BRANCH = 4'd6, JUMP   = 4'd7;

    always @(posedge clk or posedge rst) begin
        if (rst) state <= FETCH;
        else state <= next_state;
    end

    always @(*) begin
        {pwrite, iwrite, regwrite, memwrite, adrsrc, memtoreg, alusrca, regdest} = 8'b0;
        alusrcb = 2'b00; 
        aluop = 2'b00;
        next_state = FETCH;

        case (state)
            FETCH: begin
                // First cycle of fetch: compute PC+1 into ALUOut (do not write PC yet)
                adrsrc = 0; iwrite = 1; alusrca = 0; alusrcb = 2'b01;
                aluop = 2'b00;
                pwrite = 0; // delay PC write to next microcycle
                next_state = PCINC;
            end

            PCINC: begin
                // Second cycle of fetch: write computed PC+1 into PC, then go to DECODE
                pwrite = 1;
                iwrite = 0;
                next_state = DECODE;
            end

            DECODE: begin
                case (opcode)
                    4'b0000, 4'b0001: next_state = EXECUTE; // Load, Store
                    4'b0010: next_state = EXECUTE;         // Jump (needs to compute address)
                    4'b0100: next_state = EXECUTE;         // BranchZ (needs to compute branch target first)
                    4'b1000: next_state = EXECUTE;         // Type_C
                    4'b1100, 4'b1101, 4'b1110, 4'b1111: next_state = EXECUTE; // Type_D (Addi, Subi, Andi, Ori)
                    default: next_state = FETCH;
                endcase
            end

            EXECUTE: begin
                case (opcode)
                    4'b0010: begin // Jump: compute PC = immediate
                        alusrca = 0; // doesn't matter
                        alusrcb = 2'b10; // select immediate
                        aluop = 2'b00; // ALU controller will use pass-through for Jump
                        next_state = JUMP;
                    end
                    4'b0100: begin // BranchZ: compute branch target first (will conditionally use it)
                        // Compute branch target = immediate (sign-extended 9-bit address)
                        // We compute target first, then check condition in BRANCH state
                        alusrca = 0; // doesn't matter for pass-through
                        alusrcb = 2'b10; // select immediate
                        aluop = 2'b00; // Pass-through immediate (ALU controller handles BranchZ)
                        next_state = BRANCH;
                    end
                    4'b0000, 4'b0001: begin // Load/Store
                        alusrca = 1;
                        alusrcb = 2'b10;
                        aluop = 2'b00;
                        if (opcode == 4'b0000) next_state = MEM_RD;
                        else next_state = MEM_WR;
                    end
                    4'b1000: begin // Type C
                        alusrca = 1;
                        alusrcb = 2'b00;
                        aluop = 2'b10;
                        next_state = WRITEBACK;
                    end
                    4'b1100, 4'b1101, 4'b1110, 4'b1111: begin // Type D (Addi, Subi, Andi, Ori)
                        alusrca = 1;
                        alusrcb = 2'b10;
                        aluop = 2'b11;
                        next_state = WRITEBACK;
                    end
                endcase
            end

            MEM_RD: begin
                adrsrc = 1;
                next_state = WRITEBACK;
            end

            MEM_WR: begin
                adrsrc = 1;
                memwrite = 1;
                next_state = FETCH;
            end

            WRITEBACK: begin
                regwrite = 1;
                memtoreg = (opcode == 4'b0000);
                // Destination register selection:
                // - For Type-C MoveFrom (func[1]==1) write to R0
                // - For other instructions that write back (Load, Type-C non-MoveFrom, Type-D) write to Ri
                regdest = (opcode == 4'b1000) ? (func[1] == 1'b0) : 1'b1;
                next_state = FETCH;
            end

            BRANCH: begin
                // BranchZ: If (R0=Ri) PC[8:0] ← adr-9
                // Branch target (immediate) was computed in EXECUTE state, stored in ALUOut
                // Now we need to compare R0 with Ri, but we've already used ALUOut for the target
                // This is a limitation: we need both comparison and target, but only one ALU operation per cycle
                // Simplified: Compare R0 with Ri, and if equal, the target is already in ALUOut from EXECUTE
                // But ALUOut gets overwritten when we do the comparison...
                // For now, do comparison and note that proper implementation needs two cycles or storage
                alusrca = 1; 
                alusrcb = 2'b00; // Compare R0 (A) with Ri (B)
                aluop = 2'b01; // SUB for comparison
                // Note: Proper implementation would need to preserve comparison result or target
                // This simplified version may not work correctly for all cases
                // If test program doesn't use branches, this is acceptable
                next_state = FETCH;
            end

            JUMP: begin
                // PC = immediate (computed in previous EXECUTE state, now in alu_out_val)
                pwrite = 1;
                next_state = FETCH;
            end
        endcase
    end
endmodule
*/