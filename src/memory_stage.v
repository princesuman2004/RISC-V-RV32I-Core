module MemoryStage(
    input wire clk,
    input wire MemWriteM,
    input wire [31:0] ALUResultM,
    input wire [31:0] WriteDataM,
    output reg [31:0] ReadDataM
);

    parameter MEM_SIZE = 16384;
    reg [31:0] dataMemory [0:MEM_SIZE-1];
    wire [13:0] alignedAddress = ALUResultM[15:2];

    // Separate always block for reads
    always @(*) begin
        ReadDataM = dataMemory[alignedAddress];
    end

    // Separate always block for writes
    always @(posedge clk) begin  // Changed to negedge to avoid race conditions
        if (MemWriteM) begin
            dataMemory[alignedAddress] <= WriteDataM;
        end
    end

endmodule
