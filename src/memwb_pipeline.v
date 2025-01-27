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
