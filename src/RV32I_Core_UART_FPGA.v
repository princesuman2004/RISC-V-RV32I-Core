`timescale 1ns /1ps

(* KEEP_HIERARCHY = "yes" *)
(* dont_touch = "true" *)
module Top_Level(
    input wire clk,             
    input wire reset,
    input wire rx,              // UART receive pin
    output wire tx,             // UART transmit pin      
    inout wire [31:0] gpio_pins,// GPIO pins
    output wire [3:0] debug_leds,// (Optional) 4 LEDs for debugging 
    output wire [31:0] imem_monitor // For monitoring IMem[0] if desired
);

    // IF Stage Signals
    wire [31:0] PCF, InstrF, PCPlus4F;
    wire StallF;

    // ID Stage Signals
    wire [31:0] PCD, InstrD, PCPlus4D, ImmExtD, RD1D, RD2D;
    wire StallD, FlushD;
    wire [4:0] Rs1D, Rs2D, RdD;
    wire RegWriteD, MemWriteD, ALUSrcD, BranchD, JumpD;
    wire [2:0] ALUControlD;
    wire [1:0] ResultSrcD, ImmSrcD;

    // EX Stage Signals
    wire [31:0] PCE, PCPlus4E, RD1E, RD2E, ImmExtE, PCTargetE, ALUResultE, WriteDataE;
    wire [4:0] Rs1E, Rs2E, RdE;
    wire RegWriteE, MemWriteE, ALUSrcE, BranchE, JumpE, ZeroE, PCSrcE;
    wire [2:0] ALUControlE;
    wire [1:0] ResultSrcE, ForwardAE, ForwardBE;
    wire FlushE;

    // MEM Stage Signals
    wire [31:0] PCPlus4M, ALUResultM, WriteDataM, ReadDataM;
    wire [4:0] RdM;
    wire RegWriteM, MemWriteM;
    wire [1:0] ResultSrcM;

    // WB Stage Signals
    wire [31:0] PCPlus4W, ALUResultW, ReadDataW, ResultW;
    wire [4:0] RdW;
    wire RegWriteW;
    wire [1:0] ResultSrcW;

    // Wires for instruction memory writes from the single UART
    wire [31:0] instr_mem_addr;
    wire [31:0] instr_mem_data;
    wire        instr_mem_write;

    //--------------------------------------------
    //  IF Stage
    //--------------------------------------------
    IF_Stage if_stage (
        .clk(clk),
        .reset(reset),
        .StallF(StallF),
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),

        // These three signals come from MemoryStage's unified UART loader
        .uart_write_en(instr_mem_write),
        .uart_addr(instr_mem_addr),
        .uart_data(instr_mem_data),

        .PCF(PCF),
        .InstrF(InstrF),
        .PCPlus4F(PCPlus4F),
        .imem_monitor(imem_monitor)
    );

    //--------------------------------------------
    //  IF/ID Pipeline
    //--------------------------------------------
    IF_ID_Pipeline_Reg if_id_reg (
        .clk(clk),
        .reset(reset),
        .FlushD(FlushD),
        .StallD(StallD),
        .PCF(PCF),
        .InstrF(InstrF),
        .PCPlus4F(PCPlus4F),
        .PCD(PCD),
        .InstrD(InstrD),
        .PCPlus4D(PCPlus4D)
    );

    //--------------------------------------------
    //  Control Unit
    //--------------------------------------------
    ControlUnit control_unit (
        .InstrD(InstrD),
        .RegWriteD(RegWriteD),
        .MemWriteD(MemWriteD),
        .ResultSrcD(ResultSrcD),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        .BranchD(BranchD),
        .JumpD(JumpD),
        .ImmSrcD(ImmSrcD)
    );

    // Extract register addresses
    assign Rs1D = InstrD[19:15];
    assign Rs2D = InstrD[24:20];
    assign RdD  = InstrD[11:7];

    //--------------------------------------------
    //  Extend Unit
    //--------------------------------------------
    Extend_Unit extend_unit (
        .InstrD(InstrD),
        .ImmSrc(ImmSrcD),
        .ImmExtD(ImmExtD)
    );

    //--------------------------------------------
    //  Register File
    //--------------------------------------------
    wire [31:0] reg_x1_value; // optional monitor
    Register_File reg_file (
        .clk(clk),
        .reset(reset),
        .RegWriteW(RegWriteW),
        .rs1(Rs1D),
        .rs2(Rs2D),
        .rd(RdW),
        .ResultW(ResultW),
        .RD1(RD1D),
        .RD2(RD2D),
        .reg_x1(reg_x1_value) // to monitor x1 if desired
    );

    //--------------------------------------------
    //  Hazard Unit
    //--------------------------------------------
    Hazard_Unit hazard_unit (
        .Rs1D(Rs1D),
        .Rs2D(Rs2D),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdE(RdE),
        .RdM(RdM),
        .RdW(RdW),
        .RegWriteM(RegWriteM),
        .RegWriteW(RegWriteW),
        .ResultSrcE(ResultSrcE),
        .PCSrcE(PCSrcE),
        .JumpD(JumpD),        // <--- now properly connected
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE)
    );

    //--------------------------------------------
    //  ID/EX Pipeline
    //--------------------------------------------
    ID_EX_Pipeline_Reg id_ex_pipeline (
        .clk(clk),
        .reset(reset),
        .FlushE(FlushE),
        .RD1(RD1D),
        .RD2(RD2D),
        .PCD(PCD),
        .ImmExtD(ImmExtD),
        .PCPlus4D(PCPlus4D),
        .Rs1D(Rs1D),
        .Rs2D(Rs2D),
        .RdD(RdD),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        .MemWriteD(MemWriteD),
        .RegWriteD(RegWriteD),
        .ResultSrcD(ResultSrcD),
        .BranchD(BranchD),
        .JumpD(JumpD),
        .RD1E(RD1E),
        .RD2E(RD2E),
        .PCE(PCE),
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
        .JumpE(JumpE)
    );

    //--------------------------------------------
    //  EX Stage
    //--------------------------------------------
    EX_Stage ex_stage (
        .RD1E(RD1E),
        .RD2E(RD2E),
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
        .WriteDataE(WriteDataE)
    );

    assign PCSrcE = (BranchE & ZeroE) | JumpE; // PCSrcE for branch & jump

    //--------------------------------------------
    //  EX/MEM Pipeline
    //--------------------------------------------
    EX_MEM_Pipeline_Reg ex_mem_pipeline_reg (
        .clk(clk),
        .reset(reset),
        .ALUResultE(ALUResultE),
        .WriteDataE(WriteDataE),
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

    //--------------------------------------------
    //  Memory Stage (single UART + IMem loader)
    //--------------------------------------------
    MemoryStage memory_stage (
        .clk(clk),
        .reset(reset),
        .MemWriteM(MemWriteM),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .rx(rx),
        .tx(tx),
        .gpio_pins(gpio_pins),
        .ReadDataM(ReadDataM),

        // Instruction memory write signals
        .instr_mem_addr(instr_mem_addr),
        .instr_mem_data(instr_mem_data),
        .instr_mem_write(instr_mem_write)
    );

    //--------------------------------------------
    //  MEM/WB Pipeline
    //--------------------------------------------
    MEM_WB_Pipeline memwb_pipeline (
        .clk(clk),
        .reset(reset),
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

    //--------------------------------------------
    //  WB Stage
    //--------------------------------------------
    WBStage wb_stage (
        .ResultSrcW(ResultSrcW),
        .ReadDataW(ReadDataW),
        .ALUResultW(ALUResultW),
        .PCPlus4W(PCPlus4W),
        .ResultW(ResultW)
    );

    // Optional: debug LEDs
    // You can drive them from any signals you want. For example:
    assign debug_leds = 4'b0000; // or tie them to real signals as needed

endmodule


module IF_Stage(
    input wire clk,
    input wire reset,
    input wire StallF,
    input wire PCSrcE,
    input wire [31:0] PCTargetE,

    // Signals for UART-based instruction writes
    input wire uart_write_en,
    input wire [31:0] uart_addr,
    input wire [31:0] uart_data,

    output reg [31:0] PCF,
    output reg [31:0] InstrF,
    output wire [31:0] PCPlus4F,
    output wire [31:0] imem_monitor
);
    parameter Instr_MEM_SIZE = 8192;

    // Force block RAM
    (* ram_style = "block" *)
    reg [31:0] IMem [0:Instr_MEM_SIZE-1];

    // Initialize instruction memory (NOP by default)
    integer i;
    initial begin
        for (i = 0; i < Instr_MEM_SIZE; i = i + 1) begin
            IMem[i] = 32'h00000013; // NOP
        end
    end

    // Synchronous write from the single MemoryStage UART loader
    always @(posedge clk) begin
        if (uart_write_en) begin
            IMem[uart_addr[31:2]] <= uart_data;
        end
    end

    reg [31:0] currentPC;
    wire [31:0] PCPlus4;
    assign PCPlus4 = currentPC + 4;
    assign PCPlus4F = PCPlus4;

    wire [31:0] PCNext = PCSrcE ? PCTargetE : PCPlus4;

    // Synchronous PC update
    always @(posedge clk or posedge reset) begin
        if (reset)
            currentPC <= 32'b0;
        else if (!StallF)
            currentPC <= PCNext;
    end

    // Update PCF
    always @(posedge clk or posedge reset) begin
        if (reset)
            PCF <= 32'b0;
        else if (!StallF)
            PCF <= currentPC;
    end

    // Synchronous instruction fetch
    always @(posedge clk) begin
        InstrF <= IMem[currentPC[31:2]];
    end

    // For debugging: monitor IMem[0] or anything else
    assign imem_monitor = IMem[0];

endmodule




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
        end else if (StallD) begin // Stall when enable is high 
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

(* keep_hierarchy = "yes" *)
(* dont_touch = "true" *)
// Control Unit for RV32I Instruction Set
module ControlUnit(
    input wire [31:0] InstrD,
    output reg RegWriteD,
    output reg MemWriteD,
    output reg [1:0] ResultSrcD,
    output reg [2:0] ALUControlD,
    output reg ALUSrcD,
    output reg BranchD,
    output reg JumpD,
    output reg [2:0] ImmSrcD
);
    wire [6:0] opcode = InstrD[6:0];
    wire [2:0] funct3 = InstrD[14:12];
    wire [6:0] funct7 = InstrD[31:25];
    wire [1:0] ALUOpD;

    wire RegWriteD_internal;
    wire [2:0] ImmSrcD_internal;
    wire ALUSrcD_internal;
    wire MemWriteD_internal;
    wire [1:0] ResultSrcD_internal;
    wire BranchD_internal;
    wire JumpD_internal;
    wire [2:0] ALUControlD_internal;

    Main_Decoder main_decoder (
        .opcode(opcode),
        .RegWriteD(RegWriteD_internal),
        .ImmSrcD(ImmSrcD_internal),
        .ALUSrcD(ALUSrcD_internal),
        .MemWriteD(MemWriteD_internal),
        .ResultSrcD(ResultSrcD_internal),
        .BranchD(BranchD_internal),
        .ALUOpD(ALUOpD),
        .JumpD(JumpD_internal)
    );

    ALU_Decoder alu_decoder (
        .ALUOpD(ALUOpD),
        .funct3(funct3),
        .funct7(funct7),
        .ALUControlD(ALUControlD_internal)
    );

    always @(*) begin
        RegWriteD = RegWriteD_internal;
        ImmSrcD = ImmSrcD_internal;
        ALUSrcD = ALUSrcD_internal;
        MemWriteD = MemWriteD_internal;
        ResultSrcD = ResultSrcD_internal;
        BranchD = BranchD_internal;
        JumpD = JumpD_internal;
        ALUControlD = ALUControlD_internal;
    end
endmodule

(* dont_touch = "true" *)
module Main_Decoder(
    input [6:0] opcode,
    output reg RegWriteD,
    output reg [2:0] ImmSrcD,
    output reg ALUSrcD,
    output reg MemWriteD,
    output reg [1:0] ResultSrcD,
    output reg BranchD,
    output reg [1:0] ALUOpD,
    output reg JumpD
);

always @(*) begin
    case(opcode)
        7'b0000011: begin // lw
            RegWriteD = 1;
            ImmSrcD = 3'b000;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b01;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 0;
        end
        7'b0100011: begin // sw
            RegWriteD = 0;
            ImmSrcD = 3'b001;
            ALUSrcD = 1;
            MemWriteD = 1;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 0;
        end
        7'b0110011: begin // R-type
            RegWriteD = 1;
            ImmSrcD = 3'b000;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b10;
            JumpD = 0;
        end
        7'b0010011: begin // I-type ALU
            RegWriteD = 1;
            ImmSrcD = 3'b000;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b10;
            JumpD = 0;
        end
        7'b1100011: begin // Branch
            RegWriteD = 0;
            ImmSrcD = 3'b010;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 1;
            ALUOpD = 2'b01;
            JumpD = 0;
        end
        7'b1101111: begin // jal
            RegWriteD = 1;
            ImmSrcD = 3'b011;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'b10;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 1;
        end
        7'b1100111: begin // jalr
            RegWriteD = 1;
            ImmSrcD = 3'b000;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b10;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 1;
        end
        7'b0110111: begin // LUI
            RegWriteD = 1;
            ImmSrcD = 3'b100;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b11; // Use special path for LUI
            BranchD = 0;
            ALUOpD = 2'b11;     // Use special ALUOp for LUI
            JumpD = 0;
        end
        7'b0010111: begin // AUIPC
            RegWriteD = 1;
            ImmSrcD = 3'b100;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b11;     // Use special ALUOp for AUIPC
            JumpD = 0;
        end
        default: begin
            RegWriteD = 0;
            ImmSrcD = 3'b000;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 0;
        end
    endcase
end
endmodule

(* dont_touch = "true" *)
module ALU_Decoder(
    input [1:0] ALUOpD,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [2:0] ALUControlD
);

always @(*) begin
    case(ALUOpD)
        2'b00: ALUControlD = 3'b000; // add for lw/sw/jalr
        2'b01: ALUControlD = 3'b001; // subtract for branches
        2'b11: ALUControlD = 3'b000; // pass through for LUI/AUIPC
        2'b10: begin
            case(funct3)
                3'b000: begin
                    if (funct7 == 7'b0000000)
                        ALUControlD = 3'b000; // add
                    else if (funct7 == 7'b0100000)
                        ALUControlD = 3'b001; // sub
                    else
                        ALUControlD = 3'b000; // default to add
                end
                3'b001: ALUControlD = 3'b100; // sll
                3'b010: ALUControlD = 3'b101; // slt
                3'b011: ALUControlD = 3'b110; // sltu
                3'b100: ALUControlD = 3'b010; // xor
                3'b101: begin
                    if (funct7 == 7'b0000000)
                        ALUControlD = 3'b011; // srl
                    else if (funct7 == 7'b0100000)
                        ALUControlD = 3'b111; // sra
                    else
                        ALUControlD = 3'b011; // default to srl
                end
                3'b110: ALUControlD = 3'b111; // or - CORRECTED FROM 3'b100
                3'b111: ALUControlD = 3'b011; // and - CORRECTED FROM 3'b011
                default: ALUControlD = 3'b000;
            endcase
        end
        default: ALUControlD = 3'b000;
    endcase
end
endmodule


(* keep_hierarchy = "yes" *)
(* dont_touch = "true" *)
module Extend_Unit (
    input wire [31:0] InstrD,
    input wire [2:0] ImmSrc,
    output reg [31:0] ImmExtD
);
    wire [11:0] immI = InstrD[31:20];
    wire [11:0] immS = {InstrD[31:25], InstrD[11:7]};
    wire [12:0] immB = {InstrD[31], InstrD[7], InstrD[30:25], InstrD[11:8], 1'b0};
    wire [20:0] immJ = {InstrD[31], InstrD[19:12], InstrD[20], InstrD[30:21], 1'b0};
    wire [31:0] immU = {InstrD[31:12], 12'b0};

    always @(*) begin
        case (ImmSrc)
            3'b000: ImmExtD = {{20{immI[11]}}, immI};
            3'b001: ImmExtD = {{20{immS[11]}}, immS};
            3'b010: ImmExtD = {{19{immB[12]}}, immB};
            3'b011: ImmExtD = {{11{immJ[20]}}, immJ};
            3'b100: ImmExtD = immU;
            default: ImmExtD = 32'b0;
        endcase
    end
endmodule

module Register_File(
    input wire clk,
    input wire reset,
    input wire RegWriteW,          
    input wire [4:0] rs1,          
    input wire [4:0] rs2,          
    input wire [4:0] rd,           
    input wire [31:0] ResultW,     
    output wire [31:0] RD1,        
    output wire [31:0] RD2,
    output wire [31:0] reg_x1      // Output for x1
);

    // Register File: 32 registers, each 32-bit wide
    (* ram_style = "distributed" *) reg [31:0] registers [0:31];
    integer i;

    // Initialize all registers to 0
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'b0;
    end

    // Output x1 for monitoring/debug
    assign reg_x1 = registers[1];

    // Synchronous write with reset
    always @(negedge clk or posedge reset) begin
        if (reset) begin
            for (i = 1; i < 32; i = i + 1) begin  // skip x0
                registers[i] <= 32'b0;
            end
        end else if (RegWriteW && (rd != 5'b00000)) begin
            registers[rd] <= ResultW;  // x0 is never written
        end
    end

    // Read logic (asynchronous)
    assign RD1 = (rs1 == 5'd0) ? 32'd0 : registers[rs1];
    assign RD2 = (rs2 == 5'd0) ? 32'd0 : registers[rs2];

endmodule

(* keep_hierarchy = "yes" *)
(* dont_touch = "true" *)
module Hazard_Unit (
    input wire [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    input wire RegWriteM, RegWriteW,
    input wire [1:0] ResultSrcE,
    input wire PCSrcE,
    input wire JumpD,           // Added for JALR support
    output wire StallF, StallD, FlushD, FlushE,
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
    assign FlushD = PCSrcE || JumpD;  // Flush on jumps too

    // Enhanced forwarding logic
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



(* dont_touch = "true" *)
module ID_EX_Pipeline_Reg (
    input wire clk,
    input wire reset,
    input wire FlushE,

    input wire [31:0] RD1,
    input wire [31:0] RD2,
    input wire [31:0] ImmExtD,
    input wire [31:0] PCPlus4D,
    input wire [4:0] Rs1D,
    input wire [4:0] Rs2D,
    input wire [4:0] RdD,
    input wire [2:0] ALUControlD,
    input wire ALUSrcD,
    input wire MemWriteD,
    input wire RegWriteD,
    input wire [1:0] ResultSrcD,
    input wire BranchD,
    input wire JumpD,
    input wire [31:0] PCD,

    output reg [31:0] RD1E,
    output reg [31:0] RD2E,
    output reg [31:0] ImmExtE,
    output reg [31:0] PCPlus4E,
    output reg [4:0] Rs1E,
    output reg [4:0] Rs2E,
    output reg [4:0] RdE,
    output reg [2:0] ALUControlE,
    output reg ALUSrcE,
    output reg MemWriteE,
    output reg RegWriteE,
    output reg [1:0] ResultSrcE,
    output reg BranchE,
    output reg JumpE,
    output reg [31:0] PCE
);

    always @(posedge clk or posedge reset) begin
        if (reset || FlushE) begin
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
            BranchE <= 1'b0;
            JumpE <= 1'b0;
        end else begin
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
            BranchE <= BranchD;
            JumpE <= JumpD;
            PCE <= PCD;
        end
    end
endmodule


module EX_Stage (
    input wire [31:0] RD1E,        // Read Data 1 from Register File
    input wire [31:0] RD2E,        // Read Data 2 from Register File
    input wire [31:0] PCE,         // Program Counter (PC) at Execute Stage
    input wire [31:0] ImmExtE,     // Immediate value extended
    input wire [2:0] ALUControlE,  // ALU Control Signals
    input wire ALUSrcE,            // ALU Source Select (0: Register, 1: Immediate)
    
    input wire [1:0] ForwardAE,    // Forwarding control signal for ALU operand A
    input wire [1:0] ForwardBE,    // Forwarding control signal for ALU operand B
    input wire [31:0] ResultW,     // Write-back stage result for forwarding
    input wire [31:0] ALUResultM,  // ALU result from Memory stage for forwarding

    output wire [31:0] ALUResultE, // ALU result for Execute Stage
    output wire ZeroE,             // Zero flag output
    output wire [31:0] PCTargetE,  // Computed target PC for branches
    output reg [31:0] WriteDataE   // Forwarded Write Data
);

    // Internal registers
    reg [31:0] SrcAE; // ALU operand A
    reg [31:0] SrcBE; // ALU operand B

    // Forwarding logic for SrcAE
    always @(*) begin
        case (ForwardAE)
            2'b00: SrcAE = RD1E;
            2'b01: SrcAE = ResultW;
            2'b10: SrcAE = ALUResultM;
            default: SrcAE = 32'b0;
        endcase
    end

    // Forwarding logic for WriteDataE
    always @(*) begin
        case (ForwardBE)
            2'b00: WriteDataE = RD2E;
            2'b01: WriteDataE = ResultW;
            2'b10: WriteDataE = ALUResultM;
            default: WriteDataE = 32'b0;
        endcase
    end

    // ALU Source B selection logic
    always @(*) begin
        case (ALUSrcE)
            1'b0: SrcBE = WriteDataE;
            1'b1: SrcBE = ImmExtE;
            default: SrcBE = 32'b0;
        endcase
    end

    // Instantiate ALU
    ALU alu (
        .A(SrcAE),
        .B(SrcBE),
        .ALUControl(ALUControlE),
        .Result(ALUResultE),
        .Zero(ZeroE)
    );

    // Compute branch target address
    assign PCTargetE = PCE + ImmExtE;

endmodule

module ALU (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [2:0] ALUControl,
    output reg [31:0] Result,
    output reg Zero
);
    always @(*) begin
        case (ALUControl)
            3'b000: Result = A + B;
            3'b001: Result = A - B;
            3'b010: Result = A ^ B;
            3'b011: Result = A & B;           // AND - CORRECTED
            3'b100: Result = A << B[4:0];     // Shift left logical
            3'b101: Result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0;
            3'b110: Result = (A < B) ? 32'b1 : 32'b0;
            3'b111: Result = A | B;           // OR - CORRECTED
            default: Result = 32'b0;
        endcase
        Zero = (Result == 32'b0);
    end
endmodule


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



/*0x1xxxxxxx: Data Memory
0x2xxxxxxx: UART Registers
    0x20000000: UART Status
        [1:0] - rx_ready, tx_ready
        [4:2] - uart_state
    0x20000004: UART Data
0x3xxxxxxx: GPIO Registers
    0x30000000: GPIO Direction
    0x30000004: GPIO Output
    0x30000008: GPIO Input
0x4xxxxxxx: Instruction Memory (UART protocol only)*/

module MemoryStage(
    input  wire        clk,
    input  wire        reset,
    input  wire        MemWriteM,
    input  wire [31:0] ALUResultM,
    input  wire [31:0] WriteDataM,
    input  wire        rx,
    output wire        tx,
    inout  wire [31:0] gpio_pins,
    output reg  [31:0] ReadDataM,

    // Instruction memory interface
    output reg  [31:0] instr_mem_addr,
    output reg  [31:0] instr_mem_data,
    output reg         instr_mem_write
);

    // ------------------------
    // 1) Single UART instance
    // ------------------------
    wire [7:0] uart_rx_data;
    wire       uart_rx_ready;
    wire       uart_tx_ready;

    reg  [7:0] uart_tx_data;
    reg        uart_tx_write;

    uart uart_inst (
        .clk       (clk),
        .reset     (reset),
        .rx        (rx),
        .tx        (tx),
        .tx_data   (uart_tx_data),
        .tx_write  (uart_tx_write),
        .rx_data   (uart_rx_data),
        .rx_ready  (uart_rx_ready),
        .tx_ready  (uart_tx_ready)
    );

    // -------------------------------
    // 2) Data memory (example design)
    // -------------------------------
    parameter MEM_BLOCK_SIZE   = 4096;
    parameter NUM_MEMORY_BLOCKS= 4;

    // Array of [NUM_MEMORY_BLOCKS] sub-blocks, each 4096 entries of 32 bits
    (* ram_style = "distributed" *)
    reg [31:0] dataMemory [0:NUM_MEMORY_BLOCKS-1][0:MEM_BLOCK_SIZE-1];

    // For addressing sub-blocks
    wire [1:0]  mem_block_select;
    wire [11:0] mem_block_addr;

    // We must declare these as wires (or regs) at module scope
    assign mem_block_select = ALUResultM[17:16];
    assign mem_block_addr   = ALUResultM[15:4];

    // -------------------------
    // 3) GPIO register section
    // -------------------------
    reg [31:0] gpio_direction;
    reg [31:0] gpio_output;
    wire [31:0] gpio_input;

    // Tristate for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gpio_tristate
            assign gpio_pins[i] = gpio_direction[i] ? gpio_output[i] : 1'bz;
        end
    endgenerate
    assign gpio_input = gpio_pins;

    // ----------------------------
    // 4) UART status (for example)
    // ----------------------------
    // You can store debug/extra bits here if desired.
    reg  [2:0] dummy_uart_state; 
    wire [31:0] uart_status = {24'b0, dummy_uart_state, uart_tx_ready, uart_rx_ready};

    // --------------------------------------------
    // 5) Combinational read logic (memory map)
    // --------------------------------------------
    always @(*) begin
        // Default
        ReadDataM      = 32'b0;
        // We'll set instr_mem_write = 1'b0 in the sync pattern machine, below,
        // so do NOT assign it here in the read logic.

        case (ALUResultM[31:28])
            4'h1: begin
                // Data memory at 0x1xxx_xxxx
                ReadDataM = dataMemory[mem_block_select][mem_block_addr];
            end

            4'h2: begin
                // UART registers at 0x2xxx_xxxx
                case (ALUResultM[3:0])
                    4'h0: ReadDataM = uart_status;           
                    4'h4: ReadDataM = {24'b0, uart_rx_data}; 
                    default: ReadDataM = 32'b0;
                endcase
            end

            4'h3: begin
                // GPIO registers at 0x3xxx_xxxx
                case (ALUResultM[3:0])
                    4'h0: ReadDataM = gpio_direction;
                    4'h4: ReadDataM = gpio_output;
                    4'h8: ReadDataM = gpio_input;
                    default: ReadDataM = 32'b0;
                endcase
            end

            default: ReadDataM = 32'b0;
        endcase
    end

    // ------------------------------------------------
    // 6) Sequential writes from CPU (MemWriteM = 1)
    // ------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            gpio_direction <= 32'b0;
            gpio_output    <= 32'b0;
            uart_tx_write  <= 1'b0;
        end else begin
            // Default: no new UART TX strobe unless we do a store
            uart_tx_write <= 1'b0;

            if (MemWriteM) begin
                case (ALUResultM[31:28])
                    4'h1: begin
                        // Write to data memory
                        dataMemory[mem_block_select][mem_block_addr] <= WriteDataM;
                    end

                    4'h2: begin
                        // Possibly writing to UART TX data register
                        if ((ALUResultM[3:0] == 4'h4) && uart_tx_ready) begin
                            uart_tx_data  <= WriteDataM[7:0];
                            uart_tx_write <= 1'b1;
                        end
                    end

                    4'h3: begin
                        // GPIO writes
                        case (ALUResultM[3:0])
                            4'h0: gpio_direction <= WriteDataM;
                            4'h4: gpio_output    <= WriteDataM;
                            // 4'h8 => input reg is read-only
                        endcase
                    end
                endcase
            end
        end
    end

    // --------------------------------------------------------
    // 7) Sync-pattern protocol => instruction memory loader
    // --------------------------------------------------------
    reg [2:0]  uart_state;
    reg [1:0]  byte_counter;
    reg [31:0] temp_addr;
    reg [31:0] temp_data;

    localparam WAIT_SYNC1  = 3'b001;
    localparam WAIT_SYNC2  = 3'b010;
    localparam READ_ADDR   = 3'b011;
    localparam READ_DATA   = 3'b100;
    localparam WRITE_MEM   = 3'b101;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_state      <= WAIT_SYNC1;
            byte_counter    <= 2'b00;
            temp_addr       <= 32'b0;
            temp_data       <= 32'b0;
            instr_mem_write <= 1'b0;
        end else begin
            // By default, no instruction mem write this cycle
            instr_mem_write <= 1'b0;

            if (uart_rx_ready) begin
                case (uart_state)
                    WAIT_SYNC1: begin
                        if (uart_rx_data == 8'hAA) begin
                            uart_state <= WAIT_SYNC2;
                        end
                    end

                    WAIT_SYNC2: begin
                        if (uart_rx_data == 8'h55) begin
                            uart_state   <= READ_ADDR;
                            byte_counter <= 2'b00;
                            temp_addr    <= 32'b0;
                        end else begin
                            uart_state <= WAIT_SYNC1;
                        end
                    end

                    READ_ADDR: begin
                        temp_addr <= {temp_addr[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state   <= READ_DATA;
                            byte_counter <= 2'b00;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end

                    READ_DATA: begin
                        temp_data <= {temp_data[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state <= WRITE_MEM;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end

                    WRITE_MEM: begin
                        // If address is 0x4xxxxxxx => write to instruction memory
                        if (temp_addr[31:28] == 4'h4) begin
                            instr_mem_addr  <= temp_addr;
                            instr_mem_data  <= temp_data;
                            instr_mem_write <= 1'b1;
                        end
                        // Return to waiting for sync
                        uart_state <= WAIT_SYNC1;
                    end

                    default: uart_state <= WAIT_SYNC1;
                endcase
            end
        end
    end

endmodule



// MEM/WB Pipeline Register (unchanged)
module MEM_WB_Pipeline (
    input wire clk,                  
    input wire reset,                
    input wire RegWriteM,            
    input wire [1:0] ResultSrcM,     
    input wire [31:0] ReadDataM,     
    input wire [31:0] ALUResultM,    
    input wire [31:0] PCPlus4M,      
    input wire [4:0] RdM,            

    output reg RegWriteW,            
    output reg [1:0] ResultSrcW,     
    output reg [31:0] ReadDataW,     
    output reg [31:0] ALUResultW,    
    output reg [31:0] PCPlus4W,      
    output reg [4:0] RdW             
);

    // Sequential logic with synchronous reset
    always @(posedge clk) begin
        if (reset) begin
            // Clear all signals on reset
            RegWriteW   <= 1'b0;
            ResultSrcW  <= 2'b00;
            ReadDataW   <= 32'b0;
            ALUResultW  <= 32'b0;
            PCPlus4W    <= 32'b0;
            RdW         <= 5'b0;
        end else begin
            // Properly latch all signals
            RegWriteW   <= RegWriteM;
            ResultSrcW  <= ResultSrcM;
            ReadDataW   <= ReadDataM;
            ALUResultW  <= ALUResultM;
            PCPlus4W    <= PCPlus4M;
            RdW         <= RdM;
        end
    end
endmodule

module WBStage(
    input wire [1:0] ResultSrcW,
    input wire [31:0] ReadDataW,
    input wire [31:0] ALUResultW,
    input wire [31:0] PCPlus4W,
    output reg [31:0] ResultW
);

    always @(*) begin
        case(ResultSrcW)
            2'b00: ResultW = ALUResultW;      // ALU result
            2'b01: ResultW = ReadDataW;       // Memory read
            2'b10: ResultW = PCPlus4W;        // PC+4 for JAL/JALR
            2'b11: ResultW = ALUResultW;      // For LUI - already contains immediate value
            default: ResultW = 32'b0;
        endcase
    end
endmodule

module uart (
    input wire clk,
    input wire reset,          // Asynchronous reset (as you wanted)
    input wire rx,             // UART RX pin
    output wire tx,            // UART TX pin
    input wire [7:0] tx_data,  // Data to transmit
    input wire tx_write,       // Strobe to initiate TX
    output reg [7:0] rx_data,  // Received byte
    output reg rx_ready,       // Flag: byte received
    output reg tx_ready        // Flag: ready to send
);
    parameter BAUD_RATE = 115200;
    parameter CLK_FREQ = 1000000; // 1 MHz
    parameter BAUD_TICKS = CLK_FREQ / BAUD_RATE;

    // Internal
    reg [15:0] tx_counter;
    reg [15:0] rx_counter;
    reg [3:0] tx_bit_counter;
    reg [3:0] rx_bit_counter;
    reg tx_active, rx_active;
    reg [9:0] tx_shift;
    reg [9:0] rx_shift;
    reg tx_reg;

    assign tx = tx_reg;

    // Transmit logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tx_reg <= 1;
            tx_ready <= 1;
            tx_active <= 0;
            tx_counter <= 0;
            tx_bit_counter <= 0;
            tx_shift <= 10'b1111111111;
        end else begin
            if (tx_write && tx_ready) begin
                tx_shift <= {1'b1, tx_data, 1'b0}; // stop, data, start
                tx_active <= 1;
                tx_ready <= 0;
                tx_bit_counter <= 0;
                tx_counter <= 0;
            end else if (tx_active) begin
                if (tx_counter == BAUD_TICKS - 1) begin
                    tx_reg <= tx_shift[0];
                    tx_shift <= {1'b1, tx_shift[9:1]};
                    tx_bit_counter <= tx_bit_counter + 1;
                    tx_counter <= 0;
                    if (tx_bit_counter == 9) begin
                        tx_active <= 0;
                        tx_ready <= 1;
                    end
                end else begin
                    tx_counter <= tx_counter + 1;
                end
            end
        end
    end

    // Receive logic with mid-bit sampling
    reg [15:0] rx_sample_counter = 0;
    reg rx_start_bit_detected = 0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rx_ready <= 0;
            rx_active <= 0;
            rx_bit_counter <= 0;
            rx_counter <= 0;
            rx_shift <= 0;
            rx_sample_counter <= 0;
            rx_start_bit_detected <= 0;
        end else begin
            if (!rx_active && !rx_start_bit_detected && !rx) begin
                // Start bit detected
                rx_start_bit_detected <= 1;
                rx_sample_counter <= BAUD_TICKS >> 1; // half bit time
            end else if (rx_start_bit_detected) begin
                if (rx_sample_counter == 0) begin
                    rx_start_bit_detected <= 0;
                    rx_active <= 1;
                    rx_bit_counter <= 0;
                    rx_counter <= 0;
                end else begin
                    rx_sample_counter <= rx_sample_counter - 1;
                end
            end else if (rx_active) begin
                if (rx_counter == BAUD_TICKS - 1) begin
                    rx_shift <= {rx, rx_shift[9:1]};
                    rx_bit_counter <= rx_bit_counter + 1;
                    rx_counter <= 0;
                    if (rx_bit_counter == 9) begin
                        rx_active <= 0;
                        rx_data <= rx_shift[8:1]; // data bits
                        rx_ready <= 1;
                    end
                end else begin
                    rx_counter <= rx_counter + 1;
                end
            end else if (rx_ready) begin
                rx_ready <= 0; // clear flag automatically (optional)
            end
        end
    end
endmodule


