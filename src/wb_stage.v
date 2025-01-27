module WBStage(
    
    input wire [1:0] ResultSrcW,       // Result selection signal for WB stage
    input wire [31:0] ReadDataW,       // Data read for WB stage
    input wire [31:0] ALUResultW,      // ALU result for WB stage
    input wire [31:0] PCPlus4W,        // PC+4 for WB stage
    

    output reg [31:0] ResultW      // Data to be written to register
     
);

    // Combinational logic for write-back selection
    always @(*) begin
        if (ResultSrcW == 2'b00) begin
            ResultW <= ALUResultW;      // Write ALU result
        end else if (ResultSrcW == 2'b01) begin
            ResultW <= ReadDataW;       // Write data from memory
        end else if (ResultSrcW == 2'b10) begin
            ResultW <= PCPlus4W;        // Write PC+4 value
        end else begin
            ResultW <= 32'b0;           // Default case
        end

     
       
    end

endmodule

