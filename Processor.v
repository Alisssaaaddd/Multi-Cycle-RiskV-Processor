module Processor (
    input clk,
    input rst,
    // External Memory Interface (Connects to System Memory)
    input  [15:0] mem_rd,
    output [15:0] mem_wd,
    output [15:0] mem_addr,
    output        mem_we
);

    // Internal signals for interconnection
    wire [15:0] instr;
    wire zero;
    wire pwrite, iwrite, regwrite, memwrite_internal;
    wire adrsrc, memtoreg, alusrca, regdest;
    wire [1:0] alusrcb, aluop;
    wire [2:0] alucontrol;

    // Output mem_we directly from Control Unit
    assign mem_we = memwrite_internal;

    // --- 1. Control Unit Instance ---
    // Contains both Main Control (FSM) and ALU Control
    control_unit CU (
        .clk(clk),
        .rst(rst),
        .opcode(instr[15:12]),
        .func(instr[8:0]),     // 9-bit field for Type-C instructions
        .zero(zero),
        .pwrite(pwrite),
        .iwrite(iwrite),
        .regwrite(regwrite),
        .memwrite(memwrite_internal),
        .adrsrc(adrsrc),
        .memtoreg(memtoreg),
        .alusrca(alusrca),
        .regdest(regdest),
        .alusrcb(alusrcb),
        .alucontrol(alucontrol)
    );

    // --- 2. Datapath Instance ---
    Datapath DP (
        .clk(clk),
        .rst(rst),
        .pwrite(pwrite),
        .iwrite(iwrite),
        .regwrite(regwrite),
        .memwrite(memwrite_internal),
        .adrsrc(adrsrc),
        .memtoreg(memtoreg),
        .alusrca(alusrca),
        .regdest(regdest),
        .alusrcb(alusrcb),
        .alucontrol(alucontrol),
        .instr(instr),
        .zero(zero),
        .mem_rd(mem_rd),
        .mem_wd(mem_wd),
        .alu_out_adr(mem_addr)
    );

endmodule