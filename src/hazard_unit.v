module Hazard_Unit (
    input wire [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    input wire RegWriteM, RegWriteW,
    input wire [1:0] ResultSrcE,
    input wire PCSrcE,

    output wire StallF, StallD, FlushD, FlushE,  // Changed to wire
    output reg [1:0] ForwardAE, ForwardBE
);

    // Detect load instruction in Execute stage
    wire lwStall;
    assign lwStall = ((Rs1D == RdE) || (Rs2D == RdE)) &&
                     (ResultSrcE == 2'b01) &&
                     (RdE != 0);

    // Control signal assignments
    assign StallF = lwStall;
    assign StallD = lwStall;
    assign FlushE = lwStall || PCSrcE;
    assign FlushD = PCSrcE;

    // Forwarding logic
    always @(*) begin
        // Forward for ALU operand A
        if ((Rs1E != 0) && (Rs1E == RdM) && RegWriteM)
            ForwardAE = 2'b10;  // Forward from MEM stage
        else if ((Rs1E != 0) && (Rs1E == RdW) && RegWriteW)
            ForwardAE = 2'b01;  // Forward from WB stage
        else
            ForwardAE = 2'b00;  // No forwarding

        // Forward for ALU operand B
        if ((Rs2E != 0) && (Rs2E == RdM) && RegWriteM)
            ForwardBE = 2'b10;  // Forward from MEM stage
        else if ((Rs2E != 0) && (Rs2E == RdW) && RegWriteW)
            ForwardBE = 2'b01;  // Forward from WB stage
        else
            ForwardBE = 2'b00;  // No forwarding
    end

endmodule


