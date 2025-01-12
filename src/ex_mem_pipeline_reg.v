`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.12.2024 12:45:00
// Design Name: 
// Module Name: EX_MEM_Pipeline_Reg
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: EX/MEM pipeline register module based on the datapath diagram.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module EX_MEM_Pipeline_Reg (
    input wire clk,
    input wire reset,

    // Inputs from Execute Stage (EX)
    input wire [31:0] ALUResultE,   // ALU result
    input wire [31:0] WriteDataE,   // Data to write to memory
    input wire [31:0] PCPlus4E,     // PC+4 value
    input wire [4:0] RdE,           // Destination register address
    input wire MemWriteE,           // Memory write enable
   
    input wire RegWriteE,           // Register write enable
    input wire [1:0] ResultSrcE,    // Result selection signal

    // Outputs to Memory Stage (MEM)
    output reg [31:0] ALUResultM,   // Stored ALU result
    output reg [31:0] WriteDataM,   // Stored data to write to memory
    output reg [31:0] PCPlus4M,     // Stored PC+4 value
    output reg [4:0] RdM,           // Stored destination register address
    output reg MemWriteM,           // Stored memory write enable
   
    output reg RegWriteM,           // Stored register write enable
    output reg [1:0] ResultSrcM     // Stored result selection signal
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all outputs
            ALUResultM <= 32'b0;
            WriteDataM <= 32'b0;
            PCPlus4M <= 32'b0;
            RdM <= 5'b0;
            MemWriteM <= 1'b0;
            
            RegWriteM <= 1'b0;
            ResultSrcM <= 2'b0;
        end else begin
            // Store inputs into outputs
            ALUResultM <= ALUResultE;
            WriteDataM <= WriteDataE;
            PCPlus4M <= PCPlus4E;
            RdM <= RdE;
            MemWriteM <= MemWriteE;
           
            RegWriteM <= RegWriteE;
            ResultSrcM <= ResultSrcE;
        end
    end

endmodule

