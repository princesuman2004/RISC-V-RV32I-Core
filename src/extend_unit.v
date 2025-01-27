module Extend_Unit (
    input wire [31:0] InstrD,    // Instruction from ID stage
    input wire [1:0] ImmSrc,     // Immediate Source signal
    output reg [31:0] ImmExtD    // Extended immediate value
);

    // Extract immediate fields based on instruction type
    wire [11:0] immI = InstrD[31:20];                     // I-type immediate
    wire [11:0] immS = {InstrD[31:25], InstrD[11:7]};     // S-type immediate
    wire [12:0] immB = {InstrD[31], InstrD[7], InstrD[30:25], InstrD[11:8], 1'b0}; // B-type immediate
    wire [20:0] immJ = {InstrD[31], InstrD[19:12], InstrD[20], InstrD[30:21], 1'b0}; // J-type immediate

    always @(*) begin
        case (ImmSrc)
            2'b00: ImmExtD = {{20{immI[11]}}, immI};       // I-type: 12-bit signed immediate
            2'b01: ImmExtD = {{20{immS[11]}}, immS};       // S-type: 12-bit signed immediate
            2'b10: ImmExtD = {{19{immB[12]}}, immB};       // B-type: 13-bit signed immediate
            2'b11: ImmExtD = {{11{immJ[20]}}, immJ};       // J-type: 21-bit signed immediate
            default: ImmExtD = 32'b0;                      // Default case
        endcase
    end

endmodule
