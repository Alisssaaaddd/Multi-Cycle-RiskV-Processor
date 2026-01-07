module memory (
    input  clk, we,
    input  [11:0] adr, 
    input  [15:0] wd,
    output [15:0] rd
);
    reg [15:0] mem [4095:0];

    initial begin
        $readmemh("mem.txt", mem);
    end

    assign rd = mem[adr];

    always @(posedge clk) begin
        if (we) mem[adr] <= wd;
    end
endmodule