`timescale 1ns/1ps

module tb_SystemTop;

    // Clock and Reset
    reg clk;
    reg rst;
    integer i;

    // Instantiate DUT
    System_Top uut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generator (10 ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test sequence
    initial begin
        // Waveform dump (for GTKWave / ModelSim)
        $dumpfile("tb_SystemTop.vcd");
        $dumpvars(0, uut);

        // Apply reset
        rst = 1;
        #20;

        // Try to load program_sum_array.hex if present (hex format, one 16-bit word per line)
        if (!$test$plusargs("skip_readmemh")) begin
            $display("[TB] Loading program_sum_array.hex into memory (if available)");
            $readmemh("program_sum_array.hex", uut.Data_Instruction_Mem.mem);
        end else begin
            // Default memory initialization (fallback / example data)
            for (i = 0; i < 32; i = i + 1) begin
                uut.Data_Instruction_Mem.mem[i] = i; // store simple data values for observation
            end
        end

        // Ensure data array (addresses 0x64..0x6D) is initialized with values 1..10
        uut.Data_Instruction_Mem.mem[16'h0064] = 16'h0001;
        uut.Data_Instruction_Mem.mem[16'h0065] = 16'h0002;
        uut.Data_Instruction_Mem.mem[16'h0066] = 16'h0003;
        uut.Data_Instruction_Mem.mem[16'h0067] = 16'h0004;
        uut.Data_Instruction_Mem.mem[16'h0068] = 16'h0005;
        uut.Data_Instruction_Mem.mem[16'h0069] = 16'h0006;
        uut.Data_Instruction_Mem.mem[16'h006a] = 16'h0007;
        uut.Data_Instruction_Mem.mem[16'h006b] = 16'h0008;
        uut.Data_Instruction_Mem.mem[16'h006c] = 16'h0009;
        uut.Data_Instruction_Mem.mem[16'h006d] = 16'h000a;

        $display("[TB] Initialized data array at 0x64..0x6d with values 1..10");

        // Release reset and run
        rst = 0;

        // Run simulation for a fixed time then finish
        #2000;

        $display("[TB] Dumping memory 0..127:");
        for (i = 0; i < 128; i = i + 1) begin
            $display("mem[%0d] = %h", i, uut.Data_Instruction_Mem.mem[i]);
        end

        // Print register file entries and datapath debug values
        $display("[TB] R0 (Accumulator) = %h", uut.CPU.DP.RF.rf[0]);
        $display("[TB] R1 = %h", uut.CPU.DP.RF.rf[1]);
        $display("[TB] R2 (Sum Result) = %h", uut.CPU.DP.RF.rf[2]);
        $display("[TB] R3 = %h", uut.CPU.DP.RF.rf[3]);
        $display("[TB] R4 = %h", uut.CPU.DP.RF.rf[4]);
        $display("[TB] MDR  = %h", uut.CPU.DP.mdr_val);
        $display("[TB] ALU ADR = %h", uut.CPU.DP.alu_out_adr);
        $display("[TB] MEM RD = %h", uut.m_rd);
        $display("[TB] Expected sum = %h (decimal 55)", 16'h0037);

        if (uut.CPU.DP.RF.rf[2] === 16'h0037) $display("[TB] PASS: Sum is correct");
        else $display("[TB] FAIL: Sum incorrect");

        $stop;
    end

    // Monitor memory writes initiated by the processor
    always @(posedge clk) begin
        if (uut.m_we) begin
            $display("%0t: MEM WRITE at addr=%h data=%h", $time, uut.m_addr, uut.m_wd);
        end
    end

    // Monitor register write-back events for debugging
    reg [3:0] prev_state;
    reg [15:0] prev_pc;
    
    // Helper wires for easier waveform viewing in ModelSim
    wire [15:0] r0_value = uut.CPU.DP.RF.rf[0];  // Accumulator
    wire [15:0] r1_value = uut.CPU.DP.RF.rf[1];
    wire [15:0] r2_value = uut.CPU.DP.RF.rf[2];  // Expected sum location
    wire [15:0] r3_value = uut.CPU.DP.RF.rf[3];
    wire [15:0] r4_value = uut.CPU.DP.RF.rf[4];
    wire [15:0] r5_value = uut.CPU.DP.RF.rf[5];
    wire [15:0] r6_value = uut.CPU.DP.RF.rf[6];
    wire [15:0] r7_value = uut.CPU.DP.RF.rf[7];
    wire [3:0] current_state = uut.CPU.CU.MC.state;
    wire [15:0] pc_value = uut.CPU.DP.pc_val;
    wire [15:0] instruction = uut.CPU.instr;
    wire [15:0] alu_result_val = uut.CPU.DP.alu_out_val;
    wire [15:0] result_bus = uut.CPU.DP.result;
    wire [15:0] accumulator_reg = uut.CPU.DP.a_val;

    always @(posedge clk) begin
        if (uut.CPU.regwrite) begin
            $display("%0t: WB regwrite=1 regdest=%b instr[11:9]=%0d result=%h", $time, uut.CPU.regdest, uut.CPU.instr[11:9], uut.CPU.DP.result);
        end

        // Detect state changes in main_control
        if (uut.CPU.CU.MC.state !== prev_state) begin
            $display("%0t: MC STATE change %0d -> %0d", $time, prev_state, uut.CPU.CU.MC.state);
            prev_state <= uut.CPU.CU.MC.state;
        end

        // Detect PC changes when pwrite asserted
        if (uut.CPU.pwrite && (uut.CPU.DP.pc_val !== prev_pc)) begin
            $display("%0t: PC update (pwrite=1) old=%h new=%h", $time, prev_pc, uut.CPU.DP.pc_val);
            prev_pc <= uut.CPU.DP.pc_val;
        end

        // Control signals monitor for debugging (compact)
        $display("[CTRL] %0t opcode=%b regwrite=%b regdest=%b alusrca=%b alusrcb=%b aluop=%b pwrite=%b iwrite=%b", $time, uut.CPU.instr[15:12], uut.CPU.regwrite, uut.CPU.regdest, uut.CPU.alusrca, uut.CPU.alusrcb, uut.CPU.aluop, uut.CPU.pwrite, uut.CPU.iwrite);
        $display("%0t: PC=%h IR=%h", $time, uut.CPU.DP.pc_val, uut.CPU.instr);
        $display("%0t: ADR=%h MDR=%h MEM_RD=%h", $time, uut.CPU.DP.alu_out_adr, uut.CPU.DP.mdr_val, uut.m_rd);
    end

endmodule
