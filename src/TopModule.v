`timescale 1ns /1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.12.2024 18:00:00
// Design Name: 
// Module Name: Top_Level
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Top-level integration of the RISC-V processor pipeline, combining all modules.
// 
// Dependencies: Pipeline stages, hazard detection, forwarding unit, control unit, and others.
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Top_Level(
    input wire clk,             // Clock signal
    input wire reset            // Reset signal
);

    // Internal signals
    wire [31:0] PCF, PCE, ImmExtE, InstrF, PCD, InstrD, ImmExtD, SrcAE, SrcBE, ALUResultE, MemDataM, ALUResultM,ALUResultW, ReadDataW,ReadDataM, ResultW, PCTargetE, PCPlus4F, PCPlus4D, PCPlus4E, PCPlus4M,WriteDataM,WriteDataE, PCPlus4W;
    wire [31:0] RD1E, RD2E, RD1D, RD2D; // Updated naming for RD1D, RD2D
    wire [2:0] ALUControlD, ALUControlE;
    wire [1:0] ImmSrc, ResultSrcD, ResultSrcE, ResultSrcM, ResultSrcW, ForwardAE, ForwardBE;
    wire [4:0] Rs1D, Rs2D, RdD, RdE, RdM, RdW, rs1, rs2;
    wire [4:0]Rs1E, Rs2E;
    wire StallF, StallD, FlushD, FlushE;
    wire RegWriteD, RegWriteE, RegWriteM, RegWriteW, MemWriteD, MemWriteE, MemWriteM;
    wire ALUSrcD, ALUSrcE, BranchD, BranchE, JumpD, JumpE, ZeroE, PCSrcE;

    // IF Stage
    IF_Stage if_stage (
        .clk(clk),
        .enable(StallF),
        .reset(reset),
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),
        .InstrF(InstrF),
        .PCF(PCF),
        .PCPlus4F(PCPlus4F)
    );

    // IF/ID Pipeline Register
    IF_ID_Pipeline_Reg if_id_reg (
        .clk(clk),
        .reset(reset | FlushD),
        .enable(StallD),
        .PCF(PCF),
        .InstrF(InstrF),
        .PCPlus4F(PCPlus4F),
        .PCD(PCD),
        .InstrD(InstrD),
        .PCPlus4D(PCPlus4D)
    );

    // Control Unit
    ControlUnit control_unit (
        .InstrD(InstrD),
        .RegWriteD(RegWriteD),
        .MemWriteD(MemWriteD),
        .ResultSrcD(ResultSrcD),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        .BranchD(BranchD),
        .JumpD(JumpD),
        .ImmSrcD(ImmSrc)
    );

    // ID Stage
    Extend_Unit extend_unit (
        .InstrD(InstrD),
        .ImmSrc(ImmSrc),
        .ImmExtD(ImmExtD)
    );

    assign rs1 = InstrD[19:15];
    assign rs2 = InstrD[24:20];

    Register_File reg_file (
        .clk(clk),
        .reset(reset),
        .RegWriteW(RegWriteW),
        .rs1(InstrD[19:15] ),
        .rs2(InstrD[24:20]),
        .rd(RdW),
        .ResultW(ResultW),
        .RD1(RD1D),  // Connected RD1D
        .RD2(RD2D)   // Connected RD2D
    );

    // Hazard Unit
    Hazard_Unit hazard_unit (
        .Rs1D(rs1),
        .Rs2D(rs2),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdE(RdE),
        .RdM(RdM),
        .RdW(RdW),
       
        .RegWriteM(RegWriteM),
        .RegWriteW(RegWriteW),
        .ResultSrcE(ResultSrcE),
        .PCSrcE(PCSrcE),
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE)
    );
    

    // ID/EX Pipeline Register
    ID_EX_Pipeline_Reg id_ex_pipeline (
        .clk(clk),
        .reset(FlushE),
        .RD1(RD1D),      // Input RD1D
        .RD2(RD2D),      // Input RD2D
        .ImmExtD(ImmExtD),
        .PCPlus4D(PCPlus4D),
        .Rs1D(InstrD[19:15]),
        .Rs2D(InstrD[24:20]),
        .RdD(InstrD[11:7]),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        .MemWriteD(MemWriteD),
        .RegWriteD(RegWriteD),
        .ResultSrcD(ResultSrcD),
        .BranchD(BranchD),
        .JumpD(JumpD),
        .RD1E(RD1E),      // Output RD1E
        .RD2E(RD2E),      // Output RD2E
        .ImmExtE(ImmExtE),
        .PCPlus4E(PCPlus4E),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdE(RdE),
        .ALUControlE(ALUControlE),
        .ALUSrcE(ALUSrcE),
        .MemWriteE(MemWriteE),
        .RegWriteE(RegWriteE),
        .ResultSrcE(ResultSrcE),
        .BranchE(BranchE),
        .JumpE(JumpE),
        .PCD(PCD),
        .PCE(PCE)
    );

    // EX Stage
    assign PCSrcE = (BranchE & ZeroE) | JumpE;

    EX_Stage ex_stage (
        .RD1E(RD1E),        // Connected RD1E
        .RD2E(RD2E),        // Connected RD2E
        .PCE(PCE),
        .ImmExtE(ImmExtE),
        .ALUControlE(ALUControlE),
        .ALUSrcE(ALUSrcE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE),
        .ResultW(ResultW),
        .ALUResultM(ALUResultM),
        .ALUResultE(ALUResultE),
        .ZeroE(ZeroE),
        .PCTargetE(PCTargetE),
        .WriteDataE(SrcBE)
    );

    // EX/MEM Pipeline Register
    EX_MEM_Pipeline_Reg ex_mem_pipeline_reg (
        .clk(clk),
        .reset(reset),
        .ALUResultE(ALUResultE),
        .WriteDataE(SrcBE),
        .PCPlus4E(PCPlus4E),
        .RdE(RdE),
        .MemWriteE(MemWriteE),
        .RegWriteE(RegWriteE),
        .ResultSrcE(ResultSrcE),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .PCPlus4M(PCPlus4M),
        .RdM(RdM),
        .MemWriteM(MemWriteM),
        .RegWriteM(RegWriteM),
        .ResultSrcM(ResultSrcM)
    );

    // MEM Stage
    MemoryStage memory_stage (
        .clk(clk),
        .MemWriteM(MemWriteM),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .ReadDataM(ReadDataM)
    );

    // MEM/WB Pipeline Register
    MEM_WB_Pipeline memwb_pipeline (
        .clk(clk),
        .RegWriteM(RegWriteM),
        .ResultSrcM(ResultSrcM),
        .ReadDataM(ReadDataM),
        .ALUResultM(ALUResultM),
        .PCPlus4M(PCPlus4M),
        .RdM(RdM),
        .RegWriteW(RegWriteW),
        .ResultSrcW(ResultSrcW),
        .ReadDataW(ReadDataW),
        .ALUResultW(ALUResultW),
        .PCPlus4W(PCPlus4W),
        .RdW(RdW)
    );

    // WB Stage
    WBStage wb_stage (
        .ResultSrcW(ResultSrcW),
        .ReadDataW(ReadDataW),
        .ALUResultW(ALUResultW),
        .PCPlus4W(PCPlus4W),
        .ResultW(ResultW)
    );

endmodule

