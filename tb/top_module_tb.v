`timescale 1ns / 1ps

module Top_Level_Testbench;
    // Inputs
    reg clk;
    reg reset;

    // Instantiate the Top_Level module
    Top_Level uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Reset, Memory, and Register File Initialization
    integer i;
    initial begin
        // Apply reset
        reset = 1;
        #20;
        reset = 0;
        
        // Initialize memories
        $readmemh("instr_mem.mem", uut.if_stage.IMem);
        $readmemh("data_mem.mem", uut.memory_stage.dataMemory);
        $readmemh("regfile.mem", uut.reg_file.registers);
        
        // Ensure x0 is zero
        uut.reg_file.registers[0] = 32'b0;
    end

    // Simulation runtime
    initial begin
        #200;
        $finish;
    end

    // Memory monitoring parameters
    parameter INSTR_MEM_DISPLAY_SIZE = 20;  // Number of instruction memory locations to display
    parameter DATA_MEM_DISPLAY_SIZE = 20;   // Number of data memory locations to display

    // Comprehensive signal and memory monitoring
    always @(posedge clk) begin
        $display("\nTime: %0dns", $time);
        
        // Instruction Memory State
        $display("\n=== INSTRUCTION MEMORY STATE ===");
        $display("Current Fetch Address: %h", uut.PCF);
        $display("Current Instruction: %h", uut.InstrF);
        $display("Instruction Memory Contents (First %0d words):", INSTR_MEM_DISPLAY_SIZE);
        for (i = 0; i < INSTR_MEM_DISPLAY_SIZE; i = i + 1) begin
            if (uut.if_stage.IMem[i] != 0) begin  // Only show non-zero instructions
                $display("IMem[%0h]: %h (PC: %0h)", i, uut.if_stage.IMem[i], i << 2);
            end
        end
        
        // Data Memory State
        $display("\n=== DATA MEMORY STATE ===");
        $display("Current Memory Access Address: %h", uut.ALUResultM);
        $display("Memory Write Enable: %b", uut.MemWriteM);
        if (uut.MemWriteM) begin
            $display("Writing Data: %h", uut.WriteDataM);
        end
        if (uut.ResultSrcM == 2'b01) begin  // If load instruction
            $display("Reading Data: %h", uut.ReadDataM);
        end
        $display("Data Memory Contents (First %0d words):", DATA_MEM_DISPLAY_SIZE);
        for (i = 0; i < DATA_MEM_DISPLAY_SIZE; i = i + 1) begin
            if (uut.memory_stage.dataMemory[i] != 0) begin  // Only show non-zero data
                $display("DMem[%0h]: %h (%0d)", i, uut.memory_stage.dataMemory[i], 
                        $signed(uut.memory_stage.dataMemory[i]));
            end
        end
        
        // Fetch Stage Signals
        $display("\n=== FETCH STAGE ===");
        $display("PCF: %h", uut.PCF);
        $display("InstrF: %h", uut.InstrF);
        $display("PCPlus4F: %h", uut.PCPlus4F);
        $display("StallF: %b", uut.StallF);
        
        // Decode Stage Signals
        $display("\n=== DECODE STAGE ===");
        $display("PCD: %h", uut.PCD);
        $display("InstrD: %h", uut.InstrD);
        $display("PCPlus4D: %h", uut.PCPlus4D);
        $display("Rs1D: x%0d, Rs2D: x%0d, RdD: x%0d", uut.Rs1D, uut.Rs2D, uut.RdD);
        $display("RD1D: %h, RD2D: %h", uut.RD1D, uut.RD2D);
        $display("ImmExtD: %h", uut.ImmExtD);
        $display("Control Signals:");
        $display("  RegWriteD: %b, MemWriteD: %b, ResultSrcD: %b", 
                 uut.RegWriteD, uut.MemWriteD, uut.ResultSrcD);
        $display("  ALUControlD: %b, ALUSrcD: %b, BranchD: %b, JumpD: %b",
                 uut.ALUControlD, uut.ALUSrcD, uut.BranchD, uut.JumpD);
        $display("  ImmSrcD: %b", uut.ImmSrcD);
        $display("StallD: %b, FlushD: %b", uut.StallD, uut.FlushD);
        
        // Execute Stage Signals
        $display("\n=== EXECUTE STAGE ===");
        $display("PCE: %h", uut.PCE);
        $display("Rs1E: x%0d, Rs2E: x%0d, RdE: x%0d", uut.Rs1E, uut.Rs2E, uut.RdE);
        $display("RD1E: %h, RD2E: %h", uut.RD1E, uut.RD2E);
        $display("ImmExtE: %h", uut.ImmExtE);
        $display("PCPlus4E: %h", uut.PCPlus4E);
        $display("ALUResultE: %h", uut.ALUResultE);
        $display("WriteDataE: %h", uut.ex_stage.WriteDataE);
        $display("PCTargetE: %h", uut.PCTargetE);
        $display("Control Signals:");
        $display("  RegWriteE: %b, MemWriteE: %b, ResultSrcE: %b",
                 uut.RegWriteE, uut.MemWriteE, uut.ResultSrcE);
        $display("  ALUControlE: %b, ALUSrcE: %b, BranchE: %b, JumpE: %b",
                 uut.ALUControlE, uut.ALUSrcE, uut.BranchE, uut.JumpE);
        $display("  ForwardAE: %b, ForwardBE: %b",
                 uut.hazard_unit.ForwardAE, uut.hazard_unit.ForwardBE);
        $display("ZeroE: %b, PCSrcE: %b", uut.ZeroE, uut.PCSrcE);
        $display("FlushE: %b", uut.FlushE);
        
        // Memory Stage Signals
        $display("\n=== MEMORY STAGE ===");
        $display("ALUResultM: %h", uut.ALUResultM);
        $display("WriteDataM: %h", uut.WriteDataM);
        $display("ReadDataM: %h", uut.ReadDataM);
        $display("PCPlus4M: %h", uut.PCPlus4M);
        $display("RdM: x%0d", uut.RdM);
        $display("Control Signals:");
        $display("  RegWriteM: %b, MemWriteM: %b, ResultSrcM: %b",
                 uut.RegWriteM, uut.MemWriteM, uut.ResultSrcM);
        
        // Writeback Stage Signals
        $display("\n=== WRITEBACK STAGE ===");
        $display("ALUResultW: %h", uut.ALUResultW);
        $display("ReadDataW: %h", uut.ReadDataW);
        $display("PCPlus4W: %h", uut.PCPlus4W);
        $display("ResultW: %h", uut.ResultW);
        $display("RdW: x%0d", uut.RdW);
        $display("Control Signals:");
        $display("  RegWriteW: %b, ResultSrcW: %b",
                 uut.RegWriteW, uut.ResultSrcW);
        
        // Register File State
        $display("\n=== REGISTER FILE STATE ===");
        for (i = 0; i < 32; i = i + 1) begin
            if (uut.reg_file.registers[i] != 0)
                $display("x%0d: %h (%0d)", i, uut.reg_file.registers[i], $signed(uut.reg_file.registers[i]));
        end
        
        // Memory Access Summary
        $display("\n=== MEMORY ACCESS SUMMARY ===");
        if (uut.MemWriteM) begin
            $display("Memory Write Operation:");
            $display("  Address: %h", uut.ALUResultM);
            $display("  Data: %h", uut.WriteDataM);
        end
        if (uut.ResultSrcM == 2'b01) begin
            $display("Memory Read Operation:");
            $display("  Address: %h", uut.ALUResultM);
            $display("  Data: %h", uut.ReadDataM);
        end
        
        $display("\n------------------------------------------------\n");
    end

endmodule// tb file goes here
