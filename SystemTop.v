module System_Top (
    input clk,
    input rst
);
    wire [15:0] m_rd, m_wd, m_addr;
    wire m_we;

    Processor CPU (
        .clk(clk),
        .rst(rst),
        .mem_rd(m_rd),
        .mem_wd(m_wd),
        .mem_addr(m_addr),
        .mem_we(m_we)
    );

    memory Data_Instruction_Mem (
        .clk(clk),
        .we(m_we),
        .adr(m_addr[11:0]), // Use lower 12 bits for 4KW memory
        .wd(m_wd),
        .rd(m_rd)
    );

endmodule