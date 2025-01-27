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
    input wire [31:0] A,           // ALU operand A
    input wire [31:0] B,           // ALU operand B
    input wire [2:0] ALUControl,   // ALU control signals

    output reg [31:0] Result,      // ALU result
    output reg Zero                // Zero flag
);

    always @(*) begin
        case (ALUControl)
            3'b000: Result = A + B;                     // ADD (e.g., lw, sw)
            3'b001: Result = A - B;                     // SUB (e.g., branches beq, bne)
            3'b010: Result = A ^ B;                     // XOR
            3'b011: Result = A & B;                     // AND
            3'b100: Result = A << B[4:0];               // SLL (Shift Left Logical)
            3'b101: Result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0;  // SLT (Signed comparison)
            3'b110: Result = (A < B) ? 32'b1 : 32'b0;   // SLTU (Unsigned comparison)
            3'b111: begin
                // Handle SRL (Shift Right Logical) and SRA (Shift Right Arithmetic)
                if (B[10])                               // Assume B[10] differentiates SRL/SRA
                    Result = $signed(A) >>> B[4:0];     // SRA (Arithmetic Right Shift)
                else
                    Result = A >> B[4:0];              // SRL (Logical Right Shift)
            end
            default: Result = 32'b0;                    // Default case for safety
        endcase

        Zero = (Result == 32'b0); // Set Zero flag if result is 0
    end

endmodule


