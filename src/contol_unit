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

    // Extract fields from the instruction
    wire [6:0] opcode = InstrD[6:0];   // Opcode (bits 6-0)
    wire [2:0] funct3 = InstrD[14:12]; // funct3 (bits 14-12)
    wire funct7_5 = InstrD[30];        // Bit 5 of funct7 (bit 30)

    // Define opcode values for different instruction types
    localparam R_TYPE  = 7'b0110011;   // R-type instructions
    localparam I_TYPE  = 7'b0010011;   // I-type instructions
    localparam LOAD    = 7'b0000011;   // Load instructions
    localparam STORE   = 7'b0100011;   // Store instructions
    localparam BRANCH  = 7'b1100011;   // Branch instructions
    localparam JAL     = 7'b1101111;   // JAL (jump and link)
    localparam JALR    = 7'b1100111;   // JALR (jump and link register)

    always @(*) begin
        // Default values for control signals
        RegWriteD   = 0;
        MemWriteD   = 0;
        ResultSrcD  = 2'b00;
        ALUControlD = 3'b000;
        ALUSrcD     = 0;
        BranchD     = 0;
        JumpD       = 0;
        ImmSrcD     = 2'b00;

        case (opcode)
            R_TYPE: begin
                RegWriteD   = 1;               // Enable register write
                ResultSrcD  = 2'b00;           // Result comes from ALU
                ALUSrcD     = 0;               // ALU operand comes from register
                BranchD     = 0;
                JumpD       = 0;
                ImmSrcD     = 2'b00;           // No immediate

                // ALU control based on funct3 and funct7_5
                case (funct3)
                    3'b000: ALUControlD = (funct7_5) ? 3'b001 : 3'b000; // SUB or ADD
                    3'b111: ALUControlD = 3'b100; // AND
                    3'b110: ALUControlD = 3'b101; // OR
                    default: ALUControlD = 3'b000; // Default to ADD
                endcase
            end

            I_TYPE: begin
                RegWriteD   = 1;               // Enable register write
                ResultSrcD  = 2'b00;           // Result comes from ALU
                ALUSrcD     = 1;               // ALU operand comes from immediate
                BranchD     = 0;
                JumpD       = 0;
                ImmSrcD     = 2'b00;           // Immediate for I-type

                case (funct3)
                    3'b000: ALUControlD = 3'b000; // ADDI
                    3'b111: ALUControlD = 3'b100; // ANDI
                    3'b110: ALUControlD = 3'b101; // ORI
                    default: ALUControlD = 3'b000; // Default to ADD
                endcase
            end

            LOAD: begin
                RegWriteD   = 1;               // Enable register write
                MemWriteD   = 0;               // Disable memory write
                ResultSrcD  = 2'b01;           // Result comes from memory
                ALUSrcD     = 1;               // ALU operand comes from immediate
                BranchD     = 0;
                JumpD       = 0;
                ImmSrcD     = 2'b00;           // Immediate for load address
                ALUControlD = 3'b000;          // Default to ADD for address calculation
            end

            STORE: begin
                RegWriteD   = 0;               // Disable register write
                MemWriteD   = 1;               // Enable memory write
                ResultSrcD  = 2'b00;           // No register write-back
                ALUSrcD     = 1;               // ALU operand comes from immediate
                BranchD     = 0;
                JumpD       = 0;
                ImmSrcD     = 2'b01;           // Immediate for store address
                ALUControlD = 3'b000;          // Default to ADD for address calculation
            end

            BRANCH: begin
                RegWriteD   = 0;               // Disable register write
                MemWriteD   = 0;               // Disable memory write
                ResultSrcD  = 2'b00;           // No result
                ALUSrcD     = 0;               // ALU operand comes from register
                BranchD     = 1;               // Enable branching
                JumpD       = 0;
                ImmSrcD     = 2'b10;           // Immediate for branch offset
                ALUControlD = 3'b001;          // Subtract for comparison
            end

            JAL: begin
                RegWriteD   = 1;               // Enable register write
                MemWriteD   = 0;               // Disable memory write
                ResultSrcD  = 2'b10;           // Result comes from PC+4
                ALUSrcD     = 0;               // No ALU operation
                BranchD     = 0;
                JumpD       = 1;               // Enable jump
                ImmSrcD     = 2'b11;           // Immediate for jump offset
            end

            JALR: begin
                RegWriteD   = 1;               // Enable register write
                MemWriteD   = 0;               // Disable memory write
                ResultSrcD  = 2'b10;           // Result comes from PC+4
                ALUSrcD     = 1;               // ALU operand comes from immediate
                BranchD     = 0;
                JumpD       = 1;               // Enable jump
                ImmSrcD     = 2'b00;           // Immediate for JALR
                ALUControlD = 3'b000;          // ADD for target address calculation
            end

            default: begin
                // Default case: Disable all controls
                RegWriteD   = 0;
                MemWriteD   = 0;
                ResultSrcD  = 2'b00;
                ALUControlD = 3'b000;
                ALUSrcD     = 0;
                BranchD     = 0;
                JumpD       = 0;
                ImmSrcD     = 2'b00;
            end
        endcase
    end

endmodule
