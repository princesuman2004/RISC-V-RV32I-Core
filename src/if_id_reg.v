module IF_ID_Pipeline_Reg (
    input wire clk,
    input wire reset,
    input wire FlushD,
    input wire StallD,           // Active low enable signal for the pipeline register
    input wire [31:0] PCF,       // PC value from IF stage
    input wire [31:0] PCPlus4F,  // PC+4 value from IF stage
    input wire [31:0] InstrF,    // Instruction from IF stage

    // Outputs
    output reg [31:0] PCD,       // Stored PC value for ID stage
    output reg [31:0] PCPlus4D,  // Stored PC+4 value for ID stage
    output reg [31:0] InstrD     // Stored instruction for ID stage
);

    // Sequential logic for pipeline register
    always @(posedge clk or posedge reset) begin
        if (reset||FlushD) begin
            PCD <= 32'b0;         // Reset PC to 0
            PCPlus4D <= 32'b0;    // Reset PC+4 to 0
            InstrD <= 32'b0;      // Reset instruction to 0
        end else if (StallD) begin // Stall when enable is low (active low)
            PCD <= PCD;           // Hold current PC value
            PCPlus4D <= PCPlus4D; // Hold current PC+4 value
            InstrD <= InstrD;     // Hold current instruction value
        end else begin
            PCD <= PCF;           // Store new PC value
            PCPlus4D <= PCPlus4F; // Store new PC+4 value
            InstrD <= InstrF;     // Store new instruction value
        end
    end

endmodule
