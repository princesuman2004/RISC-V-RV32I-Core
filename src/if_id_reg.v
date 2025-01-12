`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.12.2024 14:00:00
// Design Name: 
// Module Name: IF_ID_Pipeline_Reg
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Pipeline register between IF and ID stages.
//              This register stores the instruction, PC, and PC+4 values, with an enable signal.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module IF_ID_Pipeline_Reg (
    input wire clk,
    input wire reset,
    input wire enable,           // Enable signal for the pipeline register
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
        if (reset) begin
            PCD <= 32'b0;        // Reset PC to 0
            PCPlus4D <= 32'b0;  // Reset PC+4 to 0
            InstrD <= 32'b0;    // Reset instruction to 0
        end else if (!enable) begin
            PCD <= PCF;         // Store PC value
            PCPlus4D <= PCPlus4F; // Store PC+4 value
            InstrD <= InstrF;     // Store instruction value
        end
    end

endmodule

