# Implementaion of RISC-V RV32I ISA & Verification.
- Open ubuntu terminal and type the following commands
  ```
  sudo apt-get update
  sudo apt-get install iverilog gtkwave
  ```
- Clone the repository.
  ```
  git clone https://github.com/vinayrayapati/rv32i
  cd rv32i
  ```
- Compile and Open the test file.
  ```
  iverilog -o iitb_rv32i iiitb_rv32i.v iiitb_rv32i_tb.v
  ./iiitb_rv32i
  ```
  ```
  gtkwave iiitb_rv32i.vcd
  ```
![Screenshot from 2024-06-02 22-42-55](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/ce9a482f-0e61-4450-bbec-a0a823200bb0)


## Analysing the test file (iitb_rv32i.v):
### Module Definition
```verilog
module iiitb_rv32i(clk, RN, NPC, WB_OUT);
input clk;
input RN;
output reg [31:0] WB_OUT, NPC;
```
- The module `iiitb_rv32i` is defined with inputs `clk` (clock signal) and `RN` (reset signal).
- The output registers `WB_OUT` and `NPC` are used to hold the result of the write-back stage and the next program counter, respectively.

### Internal Registers and Wires
```verilog
integer k;
wire EX_MEM_COND;
reg BR_EN;
```
- `k` is an integer for loop iteration.
- `EX_MEM_COND` is a wire that indicates if a branch condition is met.
- `BR_EN` is a register that enables branching.

### Stage Registers
These registers are used to store intermediate values between stages.

#### Instruction Fetch (IF) Stage
```verilog
reg[31:0] IF_ID_IR, IF_ID_NPC;
```
- `IF_ID_IR`: Holds the current instruction.
- `IF_ID_NPC`: Holds the address of the next instruction.

#### Instruction Decode (ID) Stage
```verilog
reg[31:0] ID_EX_A, ID_EX_B, ID_EX_RD, ID_EX_IMMEDIATE, ID_EX_IR, ID_EX_NPC;
```
- `ID_EX_A`, `ID_EX_B`: Hold the values of source registers.
- `ID_EX_RD`: Holds the value of the destination register.
- `ID_EX_IMMEDIATE`: Holds the immediate value extracted from the instruction.
- `ID_EX_IR`: Holds the current instruction.
- `ID_EX_NPC`: Holds the address of the next instruction.

#### Execution (EX) Stage
```verilog
reg[31:0] EX_MEM_ALUOUT, EX_MEM_B, EX_MEM_IR;
```
- `EX_MEM_ALUOUT`: Holds the result of the ALU operation.
- `EX_MEM_B`: Holds a value to be stored in memory for store instructions.
- `EX_MEM_IR`: Holds the current instruction.

#### Memory (MEM) Stage
```verilog
reg[31:0] MEM_WB_IR, MEM_WB_ALUOUT, MEM_WB_LDM;
```
- `MEM_WB_IR`: Holds the current instruction.
- `MEM_WB_ALUOUT`: Holds the ALU result.
- `MEM_WB_LDM`: Holds the data loaded from memory.

### Register File and Memory
```verilog
reg [31:0] REG[0:31];  // Register file with 32 registers
reg [31:0] MEM[0:31];  // Instruction memory
reg [31:0] DM[0:31];   // Data memory
```
- `REG`: Register file with 32 registers.
- `MEM`: Instruction memory with 32 instructions.
- `DM`: Data memory with 32 data locations.

### Constants
```verilog
parameter ADD = 3'd0, SUB = 3'd1, AND = 3'd2, OR = 3'd3, XOR = 3'd4, SLT = 3'd5;
parameter ADDI = 3'd0, SUBI = 3'd1, ANDI = 3'd2, ORI = 3'd3, XORI = 3'd4;
parameter LW = 3'd0, SW = 3'd1;
parameter BEQ = 3'd0, BNE = 3'd1;
parameter SLL = 3'd0, SRL = 3'd1;
parameter AR_TYPE = 7'd0, M_TYPE = 7'd1, BR_TYPE = 7'd2, SH_TYPE = 7'd3;
```
- Constants for various instruction types and operations.

### Initial and Instruction Fetch Stage
```verilog
always @(posedge clk or posedge RN) begin
    if (RN) begin
        NPC <= 32'd0;
        BR_EN <= 1'd0;
        REG[0] <= 32'h00000000;
        REG[1] <= 32'd1;
        REG[2] <= 32'd2;
        REG[3] <= 32'd3;
        REG[4] <= 32'd4;
        REG[5] <= 32'd5;
        REG[6] <= 32'd6;
    end else begin
        NPC <= BR_EN ? EX_MEM_ALUOUT : NPC + 32'd1;
        BR_EN <= 1'd0;
        IF_ID_IR <= MEM[NPC];
        IF_ID_NPC <= NPC + 32'd1;
    end
end
```
- On reset (`RN`), initialize `NPC` and some registers.
- Fetch the instruction from `MEM` and update `NPC`.

### Instruction Decode Stage
```verilog
always @(posedge clk) begin
    ID_EX_A <= REG[IF_ID_IR[19:15]];
    ID_EX_B <= REG[IF_ID_IR[24:20]];
    ID_EX_RD <= REG[IF_ID_IR[11:7]];
    ID_EX_IR <= IF_ID_IR;
    ID_EX_IMMEDIATE <= {{20{IF_ID_IR[31]}}, IF_ID_IR[31:20]};
    ID_EX_NPC <= IF_ID_NPC;
end
```
- Decode the instruction and read the source registers and immediate values.

### Execution Stage
```verilog
always @(posedge clk) begin
    EX_MEM_IR <= ID_EX_IR;
    case (ID_EX_IR[6:0])
        AR_TYPE: begin
            if (ID_EX_IR[31:25] == 7'd1) begin
                case (ID_EX_IR[14:12])
                    ADD: EX_MEM_ALUOUT <= ID_EX_A + ID_EX_B;
                    SUB: EX_MEM_ALUOUT <= ID_EX_A - ID_EX_B;
                    AND: EX_MEM_ALUOUT <= ID_EX_A & ID_EX_B;
                    OR:  EX_MEM_ALUOUT <= ID_EX_A | ID_EX_B;
                    XOR: EX_MEM_ALUOUT <= ID_EX_A ^ ID_EX_B;
                    SLT: EX_MEM_ALUOUT <= (ID_EX_A < ID_EX_B) ? 32'd1 : 32'd0;
                endcase
              end
            else begin
                case (ID_EX_IR[14:12])
                    ADDI: EX_MEM_ALUOUT <= ID_EX_A + ID_EX_IMMEDIATE;
                    SUBI: EX_MEM_ALUOUT <= ID_EX_A - ID_EX_IMMEDIATE;
                    ANDI: EX_MEM_ALUOUT <= ID_EX_A & ID_EX_B;
                    ORI: EX_MEM_ALUOUT <= ID_EX_A | ID_EX_B;
                    XORI: EX_MEM_ALUOUT <= ID_EX_A ^ ID_EX_B;
                endcase
            end
        end
        M_TYPE: begin
            case (ID_EX_IR[14:12])
                LW:  EX_MEM_ALUOUT <= ID_EX_A + ID_EX_IMMEDIATE;
                SW:  EX_MEM_ALUOUT <= ID_EX_IR[24:20] + ID_EX_IR[19:15];
            endcase
        end
        BR_TYPE: begin
            case (ID_EX_IR[14:12])
                BEQ: begin
                    EX_MEM_ALUOUT <= ID_EX_NPC + ID_EX_IMMEDIATE;
                    BR_EN <= 1'd1 ? (ID_EX_IR[19:15] == ID_EX_IR[11:7]) : 1'd0;
                end
                BNE: begin
                    EX_MEM_ALUOUT <= ID_EX_NPC + ID_EX_IMMEDIATE;
                    BR_EN <= (ID_EX_IR[19:15] != ID_EX_IR[11:7]) ? 1'd1 : 1'd0;
                end
            endcase
        end
        SH_TYPE: begin
            case (ID_EX_IR[14:12])
                SLL: EX_MEM_ALUOUT <= ID_EX_A << ID_EX_B;
                SRL: EX_MEM_ALUOUT <= ID_EX_A >> ID_EX_B;
            endcase
        end
    endcase
end
```
- Perform the actual ALU operations and calculate the result based on the instruction type.

### Memory Stage
```verilog
always @(posedge clk) begin
    MEM_WB_IR <= EX_MEM_IR;
    case (EX_MEM_IR[6:0])
        AR_TYPE, SH_TYPE: MEM_WB_ALUOUT <= EX_MEM_ALUOUT;
        M_TYPE: begin
            case (EX_MEM_IR[14:12])
                LW:  MEM_WB_LDM <= DM[EX_MEM_ALUOUT];
                SW:  DM[EX_MEM_ALUOUT] <= REG[EX_MEM_IR[11:7]];
            endcase
        end
    endcase
end
```
- Handle memory operations such as load and store.

### Write Back Stage
```verilog
always @(posedge clk) begin
    case (MEM_WB_IR[6:0])
        AR_TYPE, SH_TYPE: begin
            WB_OUT <= MEM_WB_ALUOUT;
            REG[MEM_WB_IR[11:7]] <= MEM_WB_ALUOUT;
        end
        M_TYPE: begin
            case (MEM_WB_IR[14:12])
                LW: begin
                    WB_OUT <= MEM_WB_LDM;
                    REG[MEM_WB_IR[11:7]] <= MEM_WB_LDM;
                end
            endcase
        end
    endcase
```
## Verification using GTKWave output waveforms:

- Final Output
![Screenshot from 2024-06-02 23-22-06](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/d19fb9ec-7437-4bab-ae94-de3efcdbdab9)
- Instruction 1: `add r6,r2,r1`

  ![Screenshot from 2024-06-03 00-20-07](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/d104cb40-f4dd-43e8-b1d3-afed6c676f37)

- Instruction 2:`sub r7,r1,r2`
  *important thing to note result comes at next cycle after getting instructions.*
 ![Screenshot from 2024-06-03 00-37-18](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/98591ab2-f0f0-4959-9f42-0b920874d66e)

- Instruction 3:`and r8,r1,r3`
  ![Screenshot from 2024-06-03 00-54-30](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/589492ac-bb4e-4ac4-8bd5-906acbba8d14)

- Instruction 4:`or r9,r2,r5`
  ![Screenshot from 2024-06-03 00-55-52](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/e1645932-57c2-41fe-baf2-9a08387a411e)
 

- Instruction 5:`xor r10,r1,r4`
  ![Screenshot from 2024-06-03 00-57-40](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/9ad95a90-1dea-4423-a4ed-a39b26be910b)


- Instruction 6:`slt r11,r2,r4`
![Screenshot from 2024-06-03 01-04-50](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/ea9c5509-8ea9-411e-b3b5-a3c544116b4f)


- Instruction 7:`addi r12,r4,5`
![Screenshot from 2024-06-03 01-12-40](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/a6bd0f1d-1211-4eb5-a7ad-206d1ea1e6dc)


- Instruction 8:`sw r3,r1,2`
![Screenshot from 2024-06-03 01-15-24](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/b0407cc0-59ae-43be-af49-f070b189ebcb)


- Instruction 9:`lw r13,r1,2`
  *Value of ALU output doesn't change as it is not an ALU operation.*
![Screenshot from 2024-06-03 01-29-04](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/f41d632c-29ed-47fb-9ebf-96f22c80a906)

- Instruction 10:`beq r0,r0,15`

![Screenshot from 2024-06-03 01-41-28](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/4bfd4c2e-a921-452f-bdbd-7b18c7b69d20)

After branching, performing 
- Instruction 11:`add r14,r2,r2`

![Screenshot from 2024-06-03 01-42-23](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/664bf1ee-af7d-4af8-9690-478e4d16c9d5)

## Conlcusion 
Implementation and verification of a verilog code is done using GTKWave.




