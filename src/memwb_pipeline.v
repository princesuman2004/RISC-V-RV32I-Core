// MEM/WB Pipeline Register
module MEM_WB_Pipeline (
    input wire clk,                  // Clock signal
    input wire RegWriteM,            // Register Write Enable from MEM stage
    input wire [1:0] ResultSrcM,     // Result selection signal from MEM stage
    input wire [31:0] ReadDataM,     // Data read from memory
    input wire [31:0] ALUResultM,    // ALU result from MEM stage
    input wire [31:0] PCPlus4M,      // PC+4 from MEM stage
    input wire [4:0] RdM,            // Destination register from MEM stage

    output reg RegWriteW,            // Register Write Enable for WB stage
    output reg [1:0] ResultSrcW,     // Result selection signal for WB stage
    output reg [31:0] ReadDataW,     // Data read for WB stage
    output reg [31:0] ALUResultW,    // ALU result for WB stage
    output reg [31:0] PCPlus4W,      // PC+4 for WB stage
    output reg [4:0] RdW             // Destination register for WB stage
);

    // Sequential logic to store pipeline values
    always @(posedge clk) begin
        RegWriteW   <= RegWriteM;      // Pass RegWrite signal to WB stage
        ResultSrcW  <= ResultSrcM;     // Pass ResultSrc signal to WB stage
        ReadDataW   <= ReadDataM;      // Pass ReadData to WB stage
        ALUResultW  <= ALUResultM;     // Pass ALUResult to WB stage
        PCPlus4W    <= PCPlus4M;       // Pass PCPlus4 to WB stage
        RdW         <= RdM;            // Pass destination register to WB stage
    end

endmodule

