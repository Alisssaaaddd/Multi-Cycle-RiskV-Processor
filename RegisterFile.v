module RegisterFile (
    input clk, we,
    input [2:0] a1, a2, a3,
    input [15:0] wd,
    output [15:0] rd1, rd2
);
    reg [15:0] rf [7:0];
    assign rd1 = rf[a1];
    assign rd2 = rf[a2];
    always @(posedge clk) if (we) rf[a3] <= wd;
endmodule

/*
module RegisterFile (
    input         clk, we,
    input  [2:0]  a1, a2, a3, 
    input  [15:0] wd,         
    output [15:0] rd1, rd2    
);
    reg [15:0] rf [7:0];

    integer i;
    initial begin
        for (i=0; i<8; i=i+1) rf[i] = 16'b0;
    end

    assign rd1 = rf[a1];
    assign rd2 = rf[a2];

    always @(posedge clk) begin
        if (we) rf[a3] <= wd;
    end
endmodule
*/