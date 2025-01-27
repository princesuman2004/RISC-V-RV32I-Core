module IF_Stage(
    input wire clk,              // Clock signal
    input wire StallF,           // Active low StallF signal (stalls when low)
    input wire reset,            // Reset signal
    input wire PCSrcE,           // Branch control signal
    input wire [31:0] PCTargetE, // Branch target address

    // Outputs
    output wire [31:0] PCF,      // Program Counter (current)
    output wire [31:0] InstrF,   // Instruction fetched
    output wire [31:0] PCPlus4F  // PC + 4 (next sequential address)
);

    reg [31:0] currentPC;        // Program Counter register

    // Internal Instruction Memory (ROM - word-addressable)
    parameter Instr_MEM_SIZE = 8192; // 8KB RAM
    reg [31:0] IMem [0:Instr_MEM_SIZE-1]; // Instruction memory with 64 entries (256 bytes)

    // Initialize instruction memory (for simulation/debugging)
    integer i;
    initial begin
        // Default to NOP instructions (if not overwritten externally)
        for (i = 0; i < Instr_MEM_SIZE; i = i + 1) begin
            IMem[i] = 32'h00000000;
        end
    end

    // Fetch instruction based on PC
    assign InstrF = IMem[currentPC[31:2]]; // Word-aligned address (ignores lower 2 bits)

    // Compute PC+4
    assign PCPlus4F = currentPC + 4;

    // Compute the next PC value (branch or sequential)
    wire [31:0] PCNext;
    assign PCNext = PCSrcE ? PCTargetE : PCPlus4F; // Branch or sequential address

    // Sequential logic for updating the PC
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            currentPC <= 32'b0; // Reset PC to 0 on reset
        end else if (StallF) begin  // Stall when enable is low (active low)
            currentPC <= currentPC; // Hold current value (do not update)
        end else begin
            currentPC <= PCNext; // Update PC if enable is high (no stall)
        end
    end

    // Assign current PC to output
    assign PCF = currentPC;

endmodule
