module Datapath (
    input clk, rst,
    input pwrite, iwrite, regwrite, memwrite,
    input adrsrc, memtoreg, alusrca, regdest,
    input [1:0] alusrcb,
    input [2:0] alucontrol,
    input [15:0] mem_rd,
    output [15:0] mem_wd,
    output [15:0] alu_out_adr,
    output [15:0] instr,
    output zero
);
    wire [15:0] pc_val, ir_val, mdr_val, res_mux_out;
    wire [15:0] rd1, rd2, a_val, b_val, imm_ext, src_a, src_b, alu_result, alu_out_val;
    wire [2:0]  wa;

    // PC Register
    Register PC_Reg (.data_in(alu_result), .rst(rst), .clk(clk), .en(pwrite), .data_out(pc_val));

    // Address Mux
    Mux2to1 AdrMux (.a(pc_val), .b(alu_out_val), .sel(adrsrc), .out(alu_out_adr));

    // Instruction & Data Registers
    Register IR_Reg  (.data_in(mem_rd), .rst(rst), .clk(clk), .en(iwrite), .data_out(ir_val));
    Register MDR_Reg (.data_in(mem_rd), .rst(rst), .clk(clk), .en(1'b1),   .data_out(mdr_val));
    assign instr = ir_val;

    // Register File Logic
    assign wa = (regdest) ? ir_val[11:9] : 3'b000;
    RegisterFile RF (
        .clk(clk), .we(regwrite),
        .a1(3'b000), .a2(ir_val[11:9]), .a3(wa),
        .wd(res_mux_out), .rd1(rd1), .rd2(rd2)
    );

    Register A_Reg (.data_in(rd1), .rst(rst), .clk(clk), .en(1'b1), .data_out(a_val));
    Register B_Reg (.data_in(rd2), .rst(rst), .clk(clk), .en(1'b1), .data_out(b_val));

   // Store Data: Always writes R0 (a_val) to memory
    assign mem_wd = a_val; 

    extend ExtUnit (.instr(ir_val), .imm_ext(imm_ext));

    // ALU Source Muxes
    Mux2to1 SrcAMux (.a(pc_val), .b(a_val), .sel(alusrca), .out(src_a));
    Mux3to1 SrcBMux (.a(b_val), .b(16'h0001), .c(imm_ext), .sel(alusrcb), .out(src_b));

    ALU Main_ALU (.SrcA(src_a), .SrcB(src_b), .ALUControl(alucontrol), .ALUResult(alu_result), .zero(zero));
    Register ALUOut_Reg (.data_in(alu_result), .rst(rst), .clk(clk), .en(1'b1), .data_out(alu_out_val));

    // Result Mux
    Mux2to1 ResMux (.a(alu_out_val), .b(mdr_val), .sel(memtoreg), .out(res_mux_out));

endmodule

/*
module Datapath (
    input clk, rst,
    // Control Signals from Control Unit
    input pwrite, iwrite, regwrite, memwrite,
    input adrsrc, memtoreg, alusrca, regdest,
    input [1:0] alusrcb,
    input [2:0] alucontrol,
    // External Memory Interface
    input  [15:0] mem_rd,    // Data read from memory
    output [15:0] mem_wd,    // Data to be written to memory
    output [15:0] alu_out_adr, // Final address sent to memory
    // Feedback to Control Unit
    output [15:0] instr,
    output zero
);

    wire [15:0] pc_val, ir_val, mdr_val;
    wire [15:0] rd1, rd2, a_val, b_val;
    wire [15:0] imm_ext, src_a, src_b;
    wire [15:0] alu_result, alu_out_val;
    wire [15:0] result;
    wire [2:0]  wa; // Write Address for Register File

    Register PC_Reg (
        .data_in(result), 
        .rst(rst), .clk(clk), .en(pwrite), 
        .data_out(pc_val)
    );


    Mux2to1 AdrMux (
        .a(pc_val), 
        .b(alu_out_val), 
        .sel(adrsrc), 
        .out(alu_out_adr)
    );

    Register IR_Reg (
        .data_in(mem_rd), 
        .rst(rst), .clk(clk), .en(iwrite), 
        .data_out(ir_val)
    );
    assign instr = ir_val;

    Register MDR_Reg (
        .data_in(mem_rd), 
        .rst(rst), .clk(clk), .en(1'b1), 
        .data_out(mdr_val)
    );

    assign wa = (regdest) ? ir_val[11:9] : 3'b000;

    RegisterFile RF (
        .clk(clk), 
        .we(regwrite),
        .a1(3'b000),       // Read Port 1: Always Accumulator (R0)
        .a2(ir_val[11:9]), // Read Port 2: Ri from instruction
        .a3(wa),           // Write Port Address
        .wd(result),       // Write Data (from ALU or Memory)
        .rd1(rd1), 
        .rd2(rd2)
    );

    Register A_Reg (.data_in(rd1), .rst(rst), .clk(clk), .en(1'b1), .data_out(a_val));
    Register B_Reg (.data_in(rd2), .rst(rst), .clk(clk), .en(1'b1), .data_out(b_val));
    
    // For Store instructions, write R0's value to memory
    assign mem_wd = a_val; 

    extend ExtUnit (
        .instr(ir_val), 
        .imm_ext(imm_ext)
    );

    Mux2to1 SrcAMux (
        .a(pc_val), 
        .b(a_val), 
        .sel(alusrca), 
        .out(src_a)
    );

    Mux3to1 SrcBMux (
        .a(b_val),      // 00: Ri
        .b(16'h0001),   // 01: Constant 1 for PC+1
        .c(imm_ext),    // 10: Immediate value
        .sel(alusrcb), 
        .out(src_b)
    );

    ALU Main_ALU (
        .SrcA(src_a), 
        .SrcB(src_b), 
        .ALUControl(alucontrol), 
        .ALUResult(alu_result), 
        .zero(zero)
    );

    Register ALUOut_Reg (
        .data_in(alu_result), 
        .rst(rst), .clk(clk), .en(1'b1), 
        .data_out(alu_out_val)
    );

        Mux2to1 ResMux (
        .a(alu_out_val), 
        .b(mdr_val), 
        .sel(memtoreg), 
        .out(result)
    );

endmodule

*/