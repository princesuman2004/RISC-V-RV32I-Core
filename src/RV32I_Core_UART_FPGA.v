`timescale 1ns /1ps

(* KEEP_HIERARCHY = "yes" *)
(* dont_touch = "true" *)
module Top_Level(
    input wire clk,             
    input wire reset,
     // Add UART pins
    input wire rx,              // UART receive pin
    output wire tx,            // UART transmit pin      
    //inout wire [31:0] gpio_pins, // GPIO pins
    output wire [3:0] debug_leds,    // 4 LEDs for debugging 
    output wire [31:0] imem_monitor  //IMem[]0    
    //output wire [31:0] reg_x1_out 
    
);
     
     //reg value
    (* keep = "true" *)wire [31:0] reg_x1_value;
    // IF Stage Signals
    (* keep = "true" *) wire [31:0] PCF, InstrF, PCPlus4F;
    (* keep = "true" *)wire StallF;
    
    
    
    // ID Stage Signals
    (* keep = "true" *) wire [31:0] PCD, InstrD, PCPlus4D, ImmExtD, RD1D, RD2D;
    (* keep = "true" *)wire StallD, FlushD;
    (* keep = "true" *)wire [4:0] Rs1D, Rs2D, RdD;
    (* keep = "true" *)wire RegWriteD, MemWriteD, ALUSrcD, BranchD, JumpD;
    (* keep = "true" *)wire [2:0] ALUControlD;
    (* keep = "true" *)wire [1:0] ResultSrcD, ImmSrcD;
    
    // EX Stage Signals
    (* keep = "true" *) wire [31:0] PCE, PCPlus4E, RD1E, RD2E, ImmExtE, PCTargetE, ALUResultE, WriteDataE;
    (* keep = "true" *)wire [4:0] Rs1E, Rs2E, RdE;
    (* keep = "true" *)wire RegWriteE, MemWriteE, ALUSrcE, BranchE, JumpE, ZeroE, PCSrcE;
    (* keep = "true" *)wire [2:0] ALUControlE;
    (* keep = "true" *)wire [1:0] ResultSrcE, ForwardAE, ForwardBE;
    (* keep = "true" *)wire FlushE;

    // MEM Stage Signals
    (* keep = "true" *) wire [31:0] PCPlus4M, ALUResultM, WriteDataM, ReadDataM;
    (* keep = "true" *)wire [4:0] RdM;
    (* keep = "true" *)wire RegWriteM, MemWriteM;
    (* keep = "true" *)wire [1:0] ResultSrcM;

    // WB Stage Signals
    (* keep = "true" *) wire [31:0] PCPlus4W, ALUResultW, ReadDataW, ResultW;
    (* keep = "true" *)wire [4:0] RdW;
    (* keep = "true" *)wire RegWriteW;
    (* keep = "true" *)wire [1:0] ResultSrcW;
    
      // New UART related signals
    
    (* keep = "true" *)wire [7:0] uart_rx_data;
    (* keep = "true" *)wire uart_rx_ready;
    (* keep = "true" *)wire uart_tx_ready;
    (* keep = "true" *)wire [7:0] uart_tx_data;
    (* keep = "true" *)wire uart_tx_write;
   
    // UART instruction memory interface signals
    (* keep = "true" *)reg [31:0] uart_imem_addr;
    (* keep = "true" *)reg [31:0] uart_imem_data;
    (* keep = "true" *)reg uart_write_en;
    
   
  
    
    (* dont_touch = "true" *)
    uart uart_inst (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .tx(tx),
        .tx_data(uart_tx_data),
        .tx_write(uart_tx_write),
        .rx_data(uart_rx_data),
        .rx_ready(uart_rx_ready),
        .tx_ready(uart_tx_ready)
    );

// UART State Machine states
    (* keep = "true" *)reg [2:0] uart_state;
    (* keep = "true" *)reg [1:0] byte_counter;
    
    // Remove unused sync_pattern register since we're directly checking values
    
    localparam WAIT_SYNC1 = 3'b001;
    localparam WAIT_SYNC2 = 3'b010;
    localparam READ_ADDR = 3'b011;
    localparam READ_DATA = 3'b100;
    localparam WRITE_MEM = 3'b101;

    // Enhanced UART State Machine with Sync Pattern Detection
       (* keep = "true" *)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_state <= WAIT_SYNC1;
            byte_counter <= 2'b00;
            uart_imem_addr <= 32'b0;
            uart_imem_data <= 32'b0;
            uart_write_en <= 1'b0;
        end else begin
            uart_write_en <= 1'b0;  // Default state
            
            if (uart_rx_ready) begin
                case (uart_state)
                    WAIT_SYNC1: begin
                        if (uart_rx_data == 8'hAA) begin  // First sync byte
                            uart_state <= WAIT_SYNC2;
                        end else begin
                            uart_state <= WAIT_SYNC1;  // Stay in WAIT_SYNC1 if wrong byte
                        end
                    end
                    
                    WAIT_SYNC2: begin
                        if (uart_rx_data == 8'h55) begin  // Second sync byte
                            uart_state <= READ_ADDR;
                            byte_counter <= 2'b00;
                            uart_imem_addr <= 32'b0;
                        end else begin
                            uart_state <= WAIT_SYNC1;  // Go back to WAIT_SYNC1 if sync fails
                        end
                    end
                    
                    READ_ADDR: begin
                        uart_imem_addr <= {uart_imem_addr[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state <= READ_DATA;
                            byte_counter <= 2'b00;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                    
                    READ_DATA: begin
                        uart_imem_data <= {uart_imem_data[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state <= WRITE_MEM;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                    
                    WRITE_MEM: begin
                        uart_write_en <= 1'b1;
                        uart_state <= WAIT_SYNC1;  // Go back to waiting for sync
                    end
                    
                    default: uart_state <= WAIT_SYNC1;
                endcase
            end
        end
    end
    
    //debug signals
    (* keep = "true" *)assign debug_leds[3] = uart_rx_ready;  // Blinks when receiving data
    (* keep = "true" *)assign debug_leds[2:0] = uart_state;   // Shows current state

(* keep = "true" *)
    // IF Stage with UART interface
     IF_Stage if_stage (
        .clk(clk),
        .reset(reset),
        .StallF(StallF),
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),
        .uart_write_en(uart_write_en),
        .uart_addr(uart_imem_addr),
        .uart_data(uart_imem_data),
        .InstrF(InstrF),
        .PCF(PCF),
        .PCPlus4F(PCPlus4F),
        .imem_monitor (imem_monitor)
    );


  (* keep = "true" *)
    // IF/ID Pipeline Register
    IF_ID_Pipeline_Reg if_id_reg (
        .clk(clk),
        .reset(reset),
        .FlushD(FlushD),
        .StallD(StallD),        // Active high stall signal
        .PCF(PCF),
        .InstrF(InstrF),
        .PCPlus4F(PCPlus4F),
        .PCD(PCD),
        .InstrD(InstrD),
        .PCPlus4D(PCPlus4D)
    );

    // Control Unit
    (* keep = "true" *)
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
    assign RdD = InstrD[11:7];
    
    (* keep = "true" *)
    // Extend Unit
    Extend_Unit extend_unit (
        .InstrD(InstrD),
        .ImmSrc(ImmSrcD),
        .ImmExtD(ImmExtD)
    );
    
    (* keep = "true" *)
    // Register File
   Register_File reg_file (
        .clk(clk),
        .reset(reset),
        .RegWriteW(RegWriteW),
        .rs1(Rs1D),
        .rs2(Rs2D),
        .rd(RdW),
        .ResultW(ResultW),
        .RD1(RD1D),
        .RD2(RD2D)
        //.reg_x1(reg_x1_value)    // Connect the new signal
    );
       
    //assign reg_x1_out = reg_x1_value;
   (* keep = "true" *)
    // Hazard Unit
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
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE)
    );
    
          (* keep = "true" *)
    // ID/EX Pipeline Register
    ID_EX_Pipeline_Reg id_ex_pipeline (
        .clk(clk),
        .reset(reset),
        .FlushE(FlushE),          // Changed from reset to FlushE
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
    
         (* keep = "true" *)
    // EX Stage
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

    // Compute PCSrcE
    assign PCSrcE = (BranchE & ZeroE) | JumpE;
     (* keep = "true" *)
    // EX/MEM Pipeline Register
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
(* keep = "true" *)
    // MEM Stage
    MemoryStage memory_stage (
        .clk(clk),
        .reset(reset),
        .MemWriteM(MemWriteM),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .rx(rx),
        .tx(tx),
        .gpio_pins(gpio_pins),
        .ReadDataM(ReadDataM)
    );
(* keep = "true" *)
    // MEM/WB Pipeline Register
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
(* keep = "true" *)
    // WB Stage
    WBStage wb_stage (
        .ResultSrcW(ResultSrcW),
        .ReadDataW(ReadDataW),
        .ALUResultW(ALUResultW),
        .PCPlus4W(PCPlus4W),
        .ResultW(ResultW)
    );

endmodule

module IF_Stage(
    input wire clk,              
    input wire reset,            
    input wire StallF,           
    input wire PCSrcE,           
    input wire [31:0] PCTargetE, 

    // UART write interface
    input wire uart_write_en,    
    input wire [31:0] uart_addr, 
    input wire [31:0] uart_data, 

    output wire [31:0] PCF,      
    output wire [31:0] InstrF,   
    output wire [31:0] PCPlus4F,
    output wire [31:0] imem_monitor 
    
);

    reg [31:0] currentPC;        
    parameter Instr_MEM_SIZE = 8192;
    (* ram_style = "block" *) reg [31:0] IMem [0:Instr_MEM_SIZE-1];

    // Initialize instruction memory
    integer i;
    initial begin
        for (i = 0; i < Instr_MEM_SIZE; i = i + 1) begin
            IMem[i] = 32'h00000013; // NOP instruction (addi x0, x0, 0)
        end
    end

    // Synchronous write to instruction memory
   
always @(posedge clk) begin
    if (uart_write_en) begin
        IMem[uart_addr[31:2]] <= uart_data & 32'hFFFFFFFF;  // Mask all bits
    end
end

    // Fetch instruction based on PC
    assign InstrF = IMem[currentPC[31:2]];
    assign PCPlus4F = currentPC + 4;
    
    // PC update logic
    wire [31:0] PCNext;
    assign PCNext = PCSrcE ? PCTargetE : PCPlus4F;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            currentPC <= 32'b0;
        end else if (StallF) begin
            currentPC <= currentPC;
        end else begin
            currentPC <= PCNext;
        end
    end

    assign PCF = currentPC;
    assign imem_monitor=IMem[0];

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
    input wire [31:0] InstrD,            // Full 32-bit instruction from pipeline register
    output reg RegWriteD,               // Control signal for register write
    output reg MemWriteD,               // Control signal for memory write
    output reg [1:0] ResultSrcD,        // Source of the result (ALU, memory, etc.)
    output reg [2:0] ALUControlD,       // Control signals for ALU
    output reg ALUSrcD,                 // Control signal to select ALU input source
    output reg BranchD,                 // Control signal for branch instructions
    output reg JumpD,                   // Control signal for jump instructions
    output reg [1:0] ImmSrcD            // Control signal for immediate generation
);

// Internal wires
wire [6:0] opcode = InstrD[6:0];
wire [2:0] funct3 = InstrD[14:12];
wire [6:0] funct7 = InstrD[31:25];
wire [1:0] ALUOpD;

wire RegWriteD_internal;
wire [1:0] ImmSrcD_internal;
wire ALUSrcD_internal;
wire MemWriteD_internal;
wire [1:0] ResultSrcD_internal;
wire BranchD_internal;
wire JumpD_internal;
wire [2:0] ALUControlD_internal;

// Instantiate Main_Decoder
(* dont_touch = "true" *)
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

// Instantiate ALU_Decoder
(* dont_touch = "true" *)
ALU_Decoder alu_decoder (
    .ALUOpD(ALUOpD),
    .funct3(funct3),
    .funct7(funct7),
    .ALUControlD(ALUControlD_internal)
);

// Assign internal signals to output ports
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
    output reg [1:0] ImmSrcD,
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
            ImmSrcD = 2'b00;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b01;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 0;
        end
        7'b0100011: begin // sw
            RegWriteD = 0;
            ImmSrcD = 2'b01;
            ALUSrcD = 1;
            MemWriteD = 1;
            ResultSrcD = 2'bxx;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 0;
        end
        7'b0110011: begin // R-type
            RegWriteD = 1;
            ImmSrcD = 2'bxx;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b10;
            JumpD = 0;
        end
        7'b1100011: begin // Branch
            RegWriteD = 0;
            ImmSrcD = 2'b10;
            ALUSrcD = 0;
            MemWriteD = 0;
            ResultSrcD = 2'bxx;
            BranchD = 1;
            ALUOpD = 2'b01;
            JumpD = 0;
        end
        7'b0010011: begin // I-type ALU
            RegWriteD = 1;
            ImmSrcD = 2'b00;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b10;
            JumpD = 0;
        end
        7'b1101111: begin // jal
            RegWriteD = 1;
            ImmSrcD = 2'b11;
            ALUSrcD = 'bx;
            MemWriteD = 0;
            ResultSrcD = 2'b10;
            BranchD = 0;
            ALUOpD = 2'bxx;
            JumpD = 1;
        end
        7'b1100111: begin // jalr
            RegWriteD = 1;
            ImmSrcD = 2'b00;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b10;
            BranchD = 0;
            ALUOpD = 2'b00;
            JumpD = 1;
        end
        7'b0110111: begin // lui
            RegWriteD = 1;
            ImmSrcD = 2'b00;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b11;
            BranchD = 0;
            ALUOpD = 2'bxx;
            JumpD = 0;
        end
        default: begin
            RegWriteD = 0;
            ImmSrcD = 2'b00;
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
                3'b110: ALUControlD = 3'b100; // or
                3'b111: ALUControlD = 3'b011; // and
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

module Register_File(
    input wire clk,
    input wire reset,
    input wire RegWriteW,          
    input wire [4:0] rs1,          
    input wire [4:0] rs2,          
    input wire [4:0] rd,           
    input wire [31:0] ResultW,     
    output wire [31:0] RD1,        
    output wire [31:0] RD2
    //output wire [31:0] reg_x1      // New output to expose x1 
);

    // Register File: 32 registers, each 32-bit wide
    (* ram_style = "distributed" *)reg [31:0] registers [0:31];
    integer i;

    // Initialize all registers to 0
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'b0;
        end
    end

    // Expose x1 directly
    assign reg_x1 = registers[1];  // Register x1 is at index 1

    // Rest of the Register_File code remains the same
    assign RD1 = (rs1 == 5'b0) ? 32'b0 : registers[rs1];
    assign RD2 = (rs2 == 5'b0) ? 32'b0 : registers[rs2];

    always @(negedge clk or posedge reset) begin
        if (reset) begin
            for (i = 1; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (RegWriteW && (rd != 5'b00000)) begin
            registers[rd] <= ResultW;
        end
    end

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
            3'b011: Result = A & B;
            3'b100: Result = A << B[4:0];
            3'b101: Result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0;
            3'b110: Result = (A < B) ? 32'b1 : 32'b0;
            3'b111: Result = $signed(A) >>> B[4:0]; // Arithmetic right shift
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



// Memory Map:
// 0x0000_0000 to 0x0000_3FFF: Instruction Memory (16KB)
// 0x1000_0000 to 0x1000_3FFF: Data Memory (16KB)
// 0x2000_0000: UART Status Register
// 0x2000_0004: UART Data Register
// 0x3000_0000: GPIO Direction Register
// 0x3000_0004: GPIO Output Register
// 0x3000_0008: GPIO Input Register

// Modified UART state machine with sync pattern detection
module MemoryStage(
    input wire clk,
    input wire reset,
    input wire MemWriteM,
    input wire [31:0] ALUResultM,
    input wire [31:0] WriteDataM,
    input wire rx,
    output wire tx,
    inout wire [31:0] gpio_pins,
    output reg [31:0] ReadDataM
);
    parameter MEM_SIZE = 16384;
    (* ram_style = "block" *) reg [31:0] dataMemory [0:MEM_SIZE-1];
    
    // GPIO registers
    reg [31:0] gpio_direction;
    reg [31:0] gpio_output;
    wire [31:0] gpio_input;
    
    // UART signals
    wire [7:0] uart_rx_data;
    wire uart_rx_ready;
    wire uart_tx_ready;
    reg [7:0] uart_tx_data;
    reg uart_tx_write;
    
    // UART state machine states
    reg [2:0] uart_state;
    reg [1:0] byte_counter;
    reg [7:0] sync_pattern;
    
    localparam WAIT_SYNC1 = 3'b001;
    localparam WAIT_SYNC2 = 3'b010;
    localparam READ_ADDR = 3'b011;
    localparam READ_DATA = 3'b100;
    localparam WRITE_MEM = 3'b101;

    
    // UART instruction memory interface signals
    reg [31:0] uart_imem_addr;
    reg [31:0] uart_imem_data;
    reg uart_write_en;

    // UART status register bits
    wire [31:0] uart_status = {30'b0, uart_tx_ready, uart_rx_ready};

    // Instantiate UART
    uart uart_inst (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .tx(tx),
        .tx_data(uart_tx_data),
        .tx_write(uart_tx_write),
        .rx_data(uart_rx_data),
        .rx_ready(uart_rx_ready),
        .tx_ready(uart_tx_ready)
    );

    // GPIO tristate control
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gpio_tristate
            assign gpio_pins[i] = gpio_direction[i] ? gpio_output[i] : 1'bz;
        end
    endgenerate
    
    assign gpio_input = gpio_pins;

    // Enhanced UART State Machine with Sync Pattern Detection
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_state <= WAIT_SYNC1;
            byte_counter <= 2'b00;
            uart_imem_addr <= 32'b0;
            uart_imem_data <= 32'b0;
            uart_write_en <= 1'b0;
            sync_pattern <= 8'b0;
        end else begin
            uart_write_en <= 1'b0;  // Default state
            
            if (uart_rx_ready) begin
                case (uart_state)
                    WAIT_SYNC1: begin
                        if (uart_rx_data == 8'hAA) begin
                            uart_state <= WAIT_SYNC2;
                        end
                    end
                    
                    WAIT_SYNC2: begin
                        if (uart_rx_data == 8'h55) begin
                            uart_state <= READ_ADDR;
                            byte_counter <= 2'b00;
                            uart_imem_addr <= 32'b0;
                        end else begin
                            uart_state <= WAIT_SYNC1;  // Reset if sync pattern fails
                        end
                    end
                    
                    READ_ADDR: begin
                        uart_imem_addr <= {uart_imem_addr[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state <= READ_DATA;
                            byte_counter <= 2'b00;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                    
                    READ_DATA: begin
                        uart_imem_data <= {uart_imem_data[23:0], uart_rx_data};
                        if (byte_counter == 2'b11) begin
                            uart_state <= WRITE_MEM;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                    
                    WRITE_MEM: begin
                        uart_write_en <= 1'b1;
                        uart_state <= WAIT_SYNC1;  // Go back to waiting for sync
                    end
                    
                    default: uart_state <= WAIT_SYNC1;
                endcase
            end
        end
    end

    // Memory read logic
    always @(*) begin
        uart_tx_write = 1'b0;
        uart_tx_data = 8'b0;
        
        case (ALUResultM[31:28])
            4'h1: ReadDataM = dataMemory[ALUResultM[15:2]];
            4'h2: begin
                case (ALUResultM[3:0])
                    4'h0: ReadDataM = uart_status;
                    4'h4: ReadDataM = {24'b0, uart_rx_data};
                    default: ReadDataM = 32'b0;
                endcase
            end
            4'h3: begin
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

    // Memory write logic
    always @(posedge clk) begin
        if (reset) begin
            gpio_direction <= 32'b0;
            gpio_output <= 32'b0;
        end else if (MemWriteM) begin
            case (ALUResultM[31:28])
                4'h1: dataMemory[ALUResultM[15:2]] <= WriteDataM;
                4'h2: begin
                    if (ALUResultM[3:0] == 4'h4 && uart_tx_ready) begin
                        uart_tx_data <= WriteDataM[7:0];
                        uart_tx_write <= 1'b1;
                    end
                end
                4'h3: begin
                    case (ALUResultM[3:0])
                        4'h0: gpio_direction <= WriteDataM;
                        4'h4: gpio_output <= WriteDataM;
                    endcase
                end
            endcase
        end
    end
endmodule

// MEM/WB Pipeline Register
module MEM_WB_Pipeline (
    input wire clk,                  
    input wire reset,                // Added reset signal
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
            2'b11: ResultW = ALUResultW;      // For LUI/AUIPC
            default: ResultW = 32'b0;
        endcase
    end
endmodule

module uart (
    input wire clk,
    input wire reset,
    input wire rx,           // UART receive pin
    output wire tx,          // UART transmit pin
    input wire [7:0] tx_data, // Data to transmit
    input wire tx_write,     // Write strobe for transmission
    output reg [7:0] rx_data, // Received data
    output reg rx_ready,     // Flag indicating data is ready
    output reg tx_ready       // Flag indicating UART is ready to transmit
);
    // UART parameters
    parameter BAUD_RATE = 115200;
    parameter CLK_FREQ = 1000000; // 125 MHz
    parameter BAUD_TICKS = (CLK_FREQ / BAUD_RATE);

    // Internal registers and wires
    reg [15:0] tx_counter;
    reg [15:0] rx_counter;
    reg [3:0] tx_bit_counter;
    reg [3:0] rx_bit_counter;
    reg tx_active, rx_active;
    reg [9:0] tx_shift;      // 1 start, 8 data, 1 stop bits
    reg [9:0] rx_shift;      // Shift register for receiving (1 start, 8 data, 1 stop bits)
    reg tx_reg;              // Register to hold the value for the tx wire

    assign tx = tx_reg;      // Assign the registered value to the tx wire

    // Transmit logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tx_reg <= 1; // Idle state
            tx_ready <= 1;
            tx_active <= 0;
            tx_counter <= 0;
            tx_bit_counter <= 0;
        end else if (tx_write && tx_ready) begin
            tx_ready <= 0;
            tx_active <= 1;
            tx_counter <= 0;
            tx_bit_counter <= 0;
            tx_shift <= {1'b1, tx_data, 1'b0}; // Start, data, stop
        end else if (tx_active) begin
            if (tx_counter == BAUD_TICKS - 1) begin
                tx_counter <= 0;
                tx_reg <= tx_shift[0]; // Update tx wire via tx_reg
                tx_shift <= {1'b1, tx_shift[9:1]};
                tx_bit_counter <= tx_bit_counter + 1;
                if (tx_bit_counter == 9) begin
                    tx_active <= 0;
                    tx_ready <= 1;
                end
            end else begin
                tx_counter <= tx_counter + 1;
            end
        end
    end

    // Receive logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rx_ready <= 0;
            rx_active <= 0;
            rx_counter <= 0;
            rx_bit_counter <= 0;
            rx_shift <= 10'b0; // Clear shift register on reset
        end else if (!rx && !rx_active) begin // Start bit detection
            rx_active <= 1;
            rx_counter <= 0;
            rx_bit_counter <= 0;
        end else if (rx_active) begin
            if (rx_counter == BAUD_TICKS - 1) begin
                rx_counter <= 0;
                rx_shift <= {rx, rx_shift[9:1]}; // Shift in received bits
                rx_bit_counter <= rx_bit_counter + 1;
                if (rx_bit_counter == 9) begin // Stop bit received
                    rx_active <= 0;
                    rx_data <= rx_shift[8:1]; // Extract received byte
                    rx_ready <= 1;
                end
            end else begin
                rx_counter <= rx_counter + 1;
            end
        end else if (rx_ready) begin
            rx_ready <= 0; // Clear ready flag after data is read
        end
    end
endmodule

