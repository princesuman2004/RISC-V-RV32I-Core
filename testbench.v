`timescale 1ns / 1ps

module tb;

  reg clk = 0;
  reg [15:0] din = 0;
  wire [15:0] dout;

  top dut(clk, din, dout);

  // Toggle the clock signal in the testbench
  always #5 clk = ~clk;

  initial begin
    // Load instructions into the instruction memory
    dut.inst_mem[0] <= 32'b00001000000000010000000000000101;
    dut.inst_mem[1] <= 32'b00001000010000010000000000000110;
    dut.inst_mem[2] <= 32'b00001000100000010000000000000000;
    dut.inst_mem[3] <= 32'b00001000110000010000000000000110;
    dut.inst_mem[4] <= 32'b00010000100001000000000000000000;
    dut.inst_mem[5] <= 32'b00011000110001110000000000000001;
    dut.inst_mem[6] <= 32'b11000000000000000000000000000100;
    dut.inst_mem[7] <= 32'b00001001000001000000000000000000;

    // Wait for a few clock cycles to let the processor execute the instructions
    #100;

    // Display the values of dout and other relevant signals
    $display("dout = %b", dout);
    // Add more $display statements for other signals as needed

    // Add further testing or verification code if required

    // Stop the simulation
    $finish;
  end

endmodule
