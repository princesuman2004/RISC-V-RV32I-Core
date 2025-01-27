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
        7'b1100011: begin // Branch (beq, bne, etc.)
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
        7'b0110111: begin // lui
            RegWriteD = 1;
            ImmSrcD = 2'b00;
            ALUSrcD = 'bx;
            MemWriteD = 0;
            ResultSrcD = 2'b11;
            BranchD = 0;
            ALUOpD = 2'bxx;
            JumpD = 0;
        end
        7'b0010111: begin // auipc
            RegWriteD = 1;
            ImmSrcD = 2'b00;
            ALUSrcD = 1;
            MemWriteD = 0;
            ResultSrcD = 2'b00;
            BranchD = 0;
            ALUOpD = 2'b00;
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
