module Register (
    input [15:0] data_in,
    input rst, clk, en, 
    output reg [15:0] data_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) data_out <= 16'b0;
        else if (en) 
            data_out <= data_in;
    end
endmodule