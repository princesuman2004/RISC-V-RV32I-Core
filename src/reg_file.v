module Register_File(
    input wire clk,
    input wire reset,
    input wire RegWriteW,          // Register write enable signal
    input wire [4:0] rs1,          // Address of source register 1
    input wire [4:0] rs2,          // Address of source register 2
    input wire [4:0] rd,           // Address of destination register
    input wire [31:0] ResultW,     // Data to be written to rd

    // Outputs
    output wire [31:0] RD1,        // Data from source register 1
    output wire [31:0] RD2         // Data from source register 2
);

    // Register File: 32 registers, each 32-bit wide
    reg [31:0] registers [0:31];
    integer i;

    // Initialize all registers to 0 (simulation purposes only)
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'b0;
        end
    end

    // Combinational read
    assign RD1 = (rs1 == 5'b0) ? 32'b0 : registers[rs1];
    assign RD2 = (rs2 == 5'b0) ? 32'b0 : registers[rs2];

    // Write on negative edge of clk, excluding register x0
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

