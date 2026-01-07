module control_unit (
    input clk, rst,
    input [3:0] opcode,
    input [8:0] func,
    input zero,
    output pwrite, iwrite, regwrite, memwrite,
    output adrsrc, memtoreg, alusrca, regdest,
    output [1:0] alusrcb,
    output [2:0] alucontrol
);

    wire [1:0] aluop_internal;

    main_control MC (
        .clk(clk), .rst(rst), .opcode(opcode), .func(func), .zero(zero),
        .pwrite(pwrite), .iwrite(iwrite), .regwrite(regwrite), .memwrite(memwrite),
        .adrsrc(adrsrc), .memtoreg(memtoreg), .alusrca(alusrca), .regdest(regdest),
        .alusrcb(alusrcb), .aluop(aluop_internal)
    );

    alu_control AC (
        .aluop(aluop_internal),
        .opcode(opcode),
        .func(func),
        .alucontrol(alucontrol)
    );

endmodule