module ID_EX_Pipeline_Reg (
    input wire clk,
    input wire reset,
    input wire FlushE,

    // Inputs from Decode Stage (ID)
    input wire [31:0] RD1,        // Register 1 value
    input wire [31:0] RD2,        // Register 2 value
    input wire [31:0] ImmExtD,    // Sign-extended immediate
    input wire [31:0] PCPlus4D,   // PC+4 value from Decode stage
    input wire [4:0] Rs1D,        // Source register 1 address
    input wire [4:0] Rs2D,        // Source register 2 address
    input wire [4:0] RdD,         // Destination register address
    input wire [2:0] ALUControlD, // ALU operation signal
    input wire ALUSrcD,           // ALU source selection signal
    input wire MemWriteD,         // Memory write enable
    input wire RegWriteD,         // Register write enable
    input wire [1:0] ResultSrcD,  // Result selection signal
    input wire BranchD,           // Branch signal
    input wire JumpD,             // Jump signal
    input wire [31:0] PCD,
    // Outputs to Execute Stage (EX)
    output reg [31:0] RD1E,       // Stored value of rd1
    output reg [31:0] RD2E,       // Stored value of rd2
    output reg [31:0] ImmExtE,    // Stored value of imm_ext
    output reg [31:0] PCPlus4E,   // Stored PC+4 value
    output reg [4:0] Rs1E,        // Stored source register 1 address
    output reg [4:0] Rs2E,        // Stored source register 2 address
    output reg [4:0] RdE,         // Stored destination register address
    output reg [2:0] ALUControlE, // Stored ALU operation signal
    output reg ALUSrcE,           // Stored ALU source signal
    output reg MemWriteE,         // Stored memory write enable
    output reg RegWriteE,         // Stored register write enable
    output reg [1:0] ResultSrcE,  // Stored result selection signal
    output reg BranchE,           // Stored Branch signal
    output reg JumpE,              // Stored Jump signal
    output reg [31:0] PCE
);

    always @(posedge clk or posedge reset) begin
        if (reset||FlushE) begin
            // Reset all outputs to default values
            RD1E <= 32'b0;
            RD2E <= 32'b0;
            ImmExtE <= 32'b0;
            PCPlus4E <= 32'b0;
            PCE <= 32'b0;
            Rs1E <= 5'b0;
            Rs2E <= 5'b0;
            RdE <= 5'b0;
            ALUControlE <= 3'b0;
            ALUSrcE <= 1'b0;
            MemWriteE <= 1'b0;
            RegWriteE <= 1'b0;
            ResultSrcE <= 2'b0;
            BranchE <= 1'b0;     // Reset Branch signal
            JumpE <= 1'b0;       // Reset Jump signal
             
        end else begin
            // Store inputs into corresponding outputs
            RD1E <= RD1;
            RD2E <= RD2;
            ImmExtE <= ImmExtD;
            PCPlus4E <= PCPlus4D;
            Rs1E <= Rs1D;
            Rs2E <= Rs2D;
            RdE <= RdD;
            ALUControlE <= ALUControlD;
            ALUSrcE <= ALUSrcD;
            MemWriteE <= MemWriteD;
            RegWriteE <= RegWriteD;
            ResultSrcE <= ResultSrcD;
            BranchE <= BranchD;   // Store Branch signal
            JumpE <= JumpD;       // Store Jump signal
            PCE <= PCD;
        end
    end

endmodule
