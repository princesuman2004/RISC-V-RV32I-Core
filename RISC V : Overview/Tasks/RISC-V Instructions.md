# RISC-V Instructions

Instructions within a processor direct the processor on what actions to perform. Every program is comprised of a vast number of instructions, which are initially fetched, decoded, and subsequently executed to execute the program.

## Instruction Format

RISC-V instructions follow a fixed-length instruction format, making decoding and execution more efficient. The structure of a RISC-V instruction typically includes:

1. **Opcode**: The opcode field specifies the operation or action to be performed by the processor. It identifies the type of instruction and directs the processor to execute a particular operation, such as arithmetic, logical, memory access, or control flow.

2. **Operand Fields**: These fields provide additional information required by the instruction. Depending on the instruction type, operand fields may specify registers, immediate values, memory addresses, or offsets.

3. **Instruction Encoding**: RISC-V instructions are encoded using a consistent format, allowing for straightforward decoding by the processor. The fixed-length encoding simplifies instruction fetching, decoding, and execution, resulting in improved performance and efficiency.

## Example RISC-V Instructions

Here are examples of common RISC-V instructions:

- **Arithmetic Instructions**: `add`, `sub`, `mul`
- **Logical Instructions**: `and`, `or`, `xor`
- **Memory Access Instructions**: `lw` (load word), `sw` (store word)
- **Control Flow Instructions**: `beq` (branch if equal), `jal` (jump and link)


# RISC-V Instruction Types and Examples

RISC-V instructions are categorized into different types based on their format and the way they encode operands and immediate values. Here are the main instruction types in RISC-V along with examples:

## R-Type (Register Type)

- **Used for:** Arithmetic and logical operations.
- **Operands:** Two source registers and one destination register.
- **Format:**
```
funct7 [31:25] | rs2 [24:20] | rs1 [19:15] | funct3 [14:12] | rd [11:7] | opcode [6:0]
```

- **Example:** `ADD rd, rs1, rs2`

In RISC-V instructions, `funct3` and `funct7` are fields used to specify the exact operation to be performed within certain instruction types.

#### Funct3

- **Purpose:** Distinguishes between different variants of operations within the same instruction type.
- **Size:** 3 bits.
- **Examples:** 
  - In R-Type instructions, `funct3` specifies specific arithmetic or logical operations (e.g., ADD, SUB, AND, OR).
  - In I-Type instructions, `funct3` differentiates between immediate arithmetic/logical operations and load/store operations (e.g., ADDI, LW, SW).
  - In S-Type instructions, `funct3` specifies the type of store operation (e.g., SB, SH, SW).
  - In B-Type instructions, `funct3` determines the branch condition type (e.g., BEQ, BNE).

#### Funct7

- **Purpose:** Works in conjunction with `funct3` to further specify the operation, mainly for R-Type instructions.
- **Size:** 7 bits.
- **Examples:** 
  - In R-Type instructions, `funct7` provides additional information for operations like ADD and SUB, differentiating between addition and subtraction.

These fields are essential for correctly interpreting and executing instructions, enabling a wide range of operations within a concise instruction format.




## I-Type (Immediate Type)

- **Used for:** Operations with immediate values, load instructions.
- **Operands:** One source register, one destination register, and an immediate value.
- **Format:**
 ```
imm[11:0] | rs1 [19:15] | funct3 [14:12] | rd [11:7] | opcode [6:0]
```
- **Example:** `ADDI rd, rs1, imm`
- **Example:** `ADDI r12, r4, 5`
  - **32-bit Code:** `000000000101 00100 000 01100 0010011`

Additional I-Type Instructions:
- **LW r10, 8(r5)**
  - **32-bit Code:** `000000001000 00101 010 01010 0000011`
- **SLTI r11, r3, 15**
  - **32-bit Code:** `000000001111 00011 010 01011 0010011`

## S-Type (Store Type)

- **Used for:** Store instructions.
- **Operands:** Two source registers and an immediate value.
- **Format:**
```
imm[11:5] | rs2 [24:20] | rs1 [19:15] | funct3 [14:12] | imm[4:0] | opcode [6:0]
```

- **Example:** `SW rs2, imm(rs1)`
- **Example:** `SW r3, r1, 2`
  - **32-bit Code:** `0000000 00010 00011 010 00001 0100011`

Additional S-Type Instructions:
- **SB r4, 3(r2)**
  - **32-bit Code:** `0000000 00011 00100 000 00010 0100011`
- **SH r5, 10(r3)**
  - **32-bit Code:** `0000000 01010 00101 001 00011 0100011`

## B-Type (Branch Type)

- **Used for:** Conditional branch instructions.
- **Operands:** Two source registers and an immediate value for the offset.
- **Format:**
```
imm[12] | imm[10:5] | rs2 [24:20] | rs1 [19:15] | funct3 [14:12] | imm[4:1] | imm[11] | opcode [6:0]
```

- **Example:** `BEQ rs1, rs2, imm`
- **Example:** `BNE r0, r1, 20`
  - **32-bit Code:** `000000 000101 00001 001 00000 1100011`

Additional B-Type Instructions:
- **BEQ r2, r3, 12**
  - **32-bit Code:** `000000 001100 00011 000 00010 1100011`
- **BLT r4, r5, -8**
  - **32-bit Code:** `111111 11000 00101 100 00100 1100011`

## U-Type (Upper Immediate Type)

- **Used for:** Instructions that involve large immediate values, such as loading the upper 20 bits.
- **Operands:** One destination register and a 20-bit immediate value.
- **Format:**
```
imm[31:12] | rd [11:7] | opcode [6:0]
```

- **Example:** `LUI rd, imm`
- **Example:** `LUI r15, 100`
  - **32-bit Code:** `000000000100 01111 0110111`

Additional U-Type Instructions:
- **AUIPC r20, 5000**
  - **32-bit Code:** `0000000010011101 10100 0010111`

## J-Type (Jump Type)

- **Used for:** Jump and link instructions.
- **Operands:** One destination register and an immediate value for the offset.
- **Format:**
```
imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd [11:7] | opcode [6:0]
```

- **Example:** `JAL rd, imm`
- **Example:** `JAL r31, 200`
  - **32-bit Code:** `00000000011001000 11111 1101111`

Additional J-Type Instructions:
- **JAL r10, 1024**
  - **32-bit Code:** `0000001000000000 01010 1101111`


### Applying to Given Instructions

Here are detailed examples of various RISC-V instruction types and their corresponding 32-bit codes:

### R-Type Instructions

1. **ADD r6, r2, r1**
 - **Type:** R
 - **32-bit Code:** `0000000 00001 00010 000 00110 0110011`

2. **SUB r7, r1, r2**
 - **Type:** R
 - **32-bit Code:** `0100000 00010 00001 000 00111 0110011`

[remaining R-Type Instructions]

### I-Type Instructions

1. **ADDI r12, r4, 5**
 - **Type:** I
 - **32-bit Code:** `000000000101 00100 000 01100 0010011`

[Add similar details for the remaining I-Type Instructions]

### S-Type Instructions

1. **SW r3, r1, 2**
 - **Type:** S
 - **32-bit Code:** `0000000 00010 00011 010 00001 0100011`

[ remaining S-Type Instructions]

### B-Type Instructions

1. **BNE r0, r1, 20**
 - **Type:** B
 - **32-bit Code:** `000000 000101 00001 001 00000 1100011`

[remaining B-Type Instructions]

### U-Type Instructions

1. **LUI r15, 100**
 - **Type:** U
 - **32-bit Code:** `000000000100 01111 0110111`

[ remaining U-Type Instructions]

### J-Type Instructions

1. **JAL r31, 200**
 - **Type:** J
 - **32-bit Code:** `00000000011001000 11111 1101111`

[ remaining J-Type Instructions]

These examples showcase the diverse range of instruction types and their formats in the RISC-V architecture. Understanding these formats is crucial for programming and optimizing software for RISC-V processors.

## Conclusion

Understanding the structure and format of RISC-V instructions is essential for programming and optimizing software for RISC-V processors. By mastering the instruction set architecture, developers can write efficient and reliable code, harnessing the full potential of RISC-V technology.
