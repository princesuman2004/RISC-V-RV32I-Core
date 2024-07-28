# Microprocessor Design in Verilog

## Project Overview

This project presents a simple CPU design implemented in Verilog. The CPU supports basic arithmetic, logical, load/store, and branch instructions. The design includes components such as a program memory, data memory, general-purpose registers, and a special-purpose register for handling multiplication results. The CPU operates based on a finite state machine (FSM) with different states for instruction fetch, decode, execution, and handling halts.

## Key Components

### Instruction Register (IR)
The instruction register (IR) holds the current instruction to be executed. The instruction format is as follows:
- `IR[31:27]`: Operation Type
- `IR[26:22]`: Destination Register
- `IR[21:17]`: Source Register 1
- `IR[16]`: Immediate Mode Selector
- `IR[15:11]`: Source Register 2
- `IR[15:0]`: Immediate Value

### General Purpose Registers (GPR)
There are 32 general-purpose registers (`GPR[0]` to `GPR[31]`) used for various operations.

### Special Purpose Register (SGPR)
The special-purpose register (`SGPR`) is used to store the most significant bits of the multiplication result.

### Program and Data Memory
- `inst_mem`: Program memory, storing the instructions.
- `data_mem`: Data memory, storing data values.

### Condition Flags
- `sign`: Sign flag, indicates the sign of the result.
- `zero`: Zero flag, indicates if the result is zero.
- `overflow`: Overflow flag, indicates arithmetic overflow.
- `carry`: Carry flag, indicates a carry in arithmetic operations.

### Control Signals
- `jmp_flag`: Jump flag, used to control jump operations.
- `stop`: Stop signal, used to halt the CPU.

## Operation Types
The CPU supports various operations categorized into arithmetic, logical, load/store, and branch instructions.

### Arithmetic Operations
- `movsgpr`: Move from SGPR to GPR
- `mov`: Move value between registers or immediate to register
- `add`: Addition
- `sub`: Subtraction
- `mul`: Multiplication

### Logical Operations
- `ror`: Bitwise OR
- `rand`: Bitwise AND
- `rxor`: Bitwise XOR
- `rxnor`: Bitwise XNOR
- `rnand`: Bitwise NAND
- `rnor`: Bitwise NOR
- `rnot`: Bitwise NOT

### Load and Store Instructions
- `storereg`: Store register content in data memory
- `storedin`: Store `din` bus content in data memory
- `senddout`: Send data from data memory to `dout` bus
- `sendreg`: Send data from data memory to register

### Branch and Jump Instructions
- `jump`: Unconditional jump
- `jcarry`: Jump if carry flag is set
- `jnocarry`: Jump if carry flag is not set
- `jsign`: Jump if sign flag is set
- `jnosign`: Jump if sign flag is not set
- `jzero`: Jump if zero flag is set
- `jnozero`: Jump if zero flag is not set
- `joverflow`: Jump if overflow flag is set
- `jnooverflow`: Jump if overflow flag is not set

### Halt Instruction
- `halt`: Stop the CPU

## Finite State Machine (FSM)
The CPU operates based on an FSM with the following states:
- `idle`: Initial state, checks the reset state.
- `fetch_inst`: Fetches the instruction from program memory.
- `dec_exec_inst`: Decodes and executes the instruction, updates condition flags.
- `delay_next_inst`: Wait state for instruction execution.
- `next_inst`: Prepares for the next instruction.
- `sense_halt`: Checks if the CPU should halt or continue.

## Detailed Workflow
1. **Reset and Initialization**: On reset (`sys_rst`), the CPU initializes the program counter (`PC`) and instruction register (`IR`).
2. **Instruction Fetch**: In the `fetch_inst` state, the CPU fetches the instruction from program memory using the program counter (`PC`).
3. **Instruction Decode and Execute**: In the `dec_exec_inst` state, the CPU decodes the fetched instruction and performs the corresponding operation. Condition flags are updated based on the operation's result.
4. **Delay for Next Instruction**: The `delay_next_inst` state provides a wait period before moving to the next instruction.
5. **Next Instruction**: In the `next_inst` state, the CPU updates the program counter based on jump conditions or simply increments it to fetch the next instruction.
6. **Halt Detection**: In the `sense_halt` state, the CPU checks if it should halt based on the `halt` instruction or continue executing instructions.

## How to Run
1. **Program Memory Initialization**: The program memory is initialized using the `inst_data.mem` file. Ensure this file contains the binary representation of the instructions to be executed.
2. **Simulation**: Use a Verilog simulator to run the `top` module. Apply the clock (`clk`) and reset (`sys_rst`) signals as needed.

## Conclusion
This simple CPU design demonstrates the fundamental concepts of instruction fetch, decode, execute, and condition flag updates. The modular design and FSM-based operation make it easy to understand and extend for more complex CPU functionalities. This project serves as a solid foundation for exploring CPU design and digital logic concepts in Verilog.
