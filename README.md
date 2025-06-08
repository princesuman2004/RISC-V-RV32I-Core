

# RV32I RISC V Pipelined Processor with Full Hazard Handling

This repository contains the design and implementation of a **pipelined processor** in Verilog with **full hazard handling**. The processor design is based on a 5-stage pipeline architecture with mechanisms to handle **data hazards**, **control hazards**, and **structural hazards**. The design includes critical components such as the ALU, control unit, register file, data memory, instruction memory, and hazard unit.

## Features

- **5-Stage Pipeline**: Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), Write-back (WB).
- **Full Hazard Handling**:
  - **Data Hazards**: Managed through forwarding and stalling.
  - **Control Hazards**: Managed through branch prediction and stalling.
  - **Structural Hazards**: Avoided by ensuring no resource conflicts in the pipeline.
- **Forwarding Unit**: Resolves read-after-write (RAW) data hazards by forwarding values from later stages to earlier ones.
- **Stalling Unit**: Inserts no-op instructions in the pipeline to handle data and control hazards.
- **Hazard Unit**: Responsible for detecting and resolving pipeline hazards.

## Folder Structure

The repository follows a modular structure for ease of development and testing:

```
pipelined-processor-with-hazard-handling/
│
├── src/                  # Verilog source files (all the Verilog code for your processor)
│   ├── top_module.v              # The top-level module that connects all components.
│   ├── control_unit.v           # The control unit generates control signals for the processor.
│   ├── ex_mem_pipeline_reg.v    # Pipeline register between EX and MEM stages.
│   ├── ex_stage.v               # The execution stage of the processor (handles ALU operations).
│   ├── extend_unit.v            # Extension unit for handling instruction parsing.
│   ├── hazard_unit.v            # Hazard detection and forwarding unit for handling pipeline hazards.
│   ├── id_ex_pipeline_reg.v     # Pipeline register between ID and EX stages.
│   ├── if_id_reg.v              # Pipeline register between IF and ID stages.
│   ├── if_stage.v               # Instruction Fetch (IF) stage.
│   ├── memory_stage.v           # Memory access stage (handles load/store instructions).
│   ├── memwb_pipeline_reg.v     # Pipeline register between MEM and WB stages.
│   ├── reg_file.v               # Register file to store processor registers.
│   └── wb_stage.v               # Write-back stage of the processor (handles writing results to registers).
│
├── tb/                       # Testbenches (files for testing the functionality of modules)
│   ├── top_module_tb.v         # Testbench for the top module (overall processor simulation).
│   ├── data_mem.mem            # Memory initialization file for data memory.
│   ├── instr_mem.mem           # Memory initialization file for instruction memory.
│   └── regfile.mem             # Memory initialization file for the register file.
│
├── README.md                  # The readme file that explains the project, setup, and usage.
└── LICENSE                    # The license file, such as MIT or GPL, for open-source licensing.

```
## Processor Architecture

Here is the block diagram of the pipelined processor:
![image](https://github.com/user-attachments/assets/46e64f6c-b77d-45e1-b990-af4b6785354a)

## Components

### 1. **Control Unit**
The control unit generates the necessary control signals (`RegWrite`, `MemWrite`, `Branch`, `ALUControl`, etc.) based on the instruction type. It determines whether the operation is arithmetic, logical, or memory-related.

### 2. **ALU (Arithmetic Logic Unit)**
The ALU performs arithmetic and logical operations such as addition, subtraction, AND, OR, etc. It also generates a zero flag based on the result of the operation.

### 3. **Data Memory**
Data memory stores and retrieves data from the processor's memory during execution. It handles read and write operations and is used by load and store instructions.

### 4. **Instruction Memory**
Instruction memory holds the program code and supplies the instructions to be fetched during the Instruction Fetch (IF) stage.

### 5. **Register File**
The register file holds the general-purpose registers of the processor. It handles read and write operations for registers during the Instruction Decode (ID) and Write-back (WB) stages.

### 6. **Hazard Unit**
The hazard unit detects data and control hazards in the pipeline and triggers forwarding or stalling mechanisms to resolve them. It ensures that the pipeline runs smoothly without incorrect data being used.


## Getting Started

### Prerequisites

- **Vivado** (for Verilog design, synthesis, and simulation)
- A text editor or IDE for Verilog code (e.g., VS Code with Verilog extension)

### Setting Up the Project in Vivado

1. **Clone the repository**:
   First, clone the repository to your local machine:
   ```bash
   git clone https://github.com/princesuman2004/RISC-V-RV32I-Core.git
   cd RISC-V-RV32I-Core
   ```

2. **Open Vivado**:
   Open Vivado and create a new project:

   - Launch Vivado.
   - Select **Create New Project**.
   - Name the project and select a location for the project files.
   - Choose the appropriate FPGA family or set it as **None** if you're targeting a simulation only project.
   - Click **Next** until you reach the **Default Part** selection and set it as appropriate for your hardware (or select **None** for simulation).

3. **Add Verilog Files**:
   - In the Vivado project window, select **Add Sources** and add all the Verilog files from the `src/` directory. This will include modules like `TopModule.v`, `control_unit.v`, `ex_stage`, and others.
   - Ensure that all files are included in the **Design Sources**.

4. **Set Up Testbench**:
   - Add the testbenches located in the `tb/` folder as **Simulation Sources** in Vivado.
   - Choose the main testbench file, for example, `top_module_tb.v`, as your top-level simulation module.

5. **Run Synthesis (Optional)**:
   - If you want to perform synthesis, select **Run Synthesis** in Vivado. This step is optional and necessary if you're targeting an FPGA.

6. **Run Simulation**:
   - Select **Run Simulation** from the Vivado toolbar and choose **Simulation Settings**. 
   - Run the **Behavioral Simulation** or **Post-Synthesis Simulation** depending on your project.
   - Vivado will compile your Verilog code and testbench, and launch the **Simulator** to show the waveform and results.

   You can also use the **GUI** to interact with the simulation results or run the simulation from the **Tcl Console** using:
   ```tcl
   launch_simulation
   ```

7. **View Simulation Results**:
   - Once the simulation completes, you can analyze the waveform and debug the design.

### Example of Running a Simulation (Vivado GUI)

1. Open the **Simulation** window in Vivado after creating the testbench.
2. Click on **Run Simulation** > **Run Behavioral Simulation** (or Post-Synthesis, depending on your choice).
3. Observe the **Simulation Results** in the waveform viewer.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- The project is based on standard pipelined processor architecture and can be extended with additional features such as branch prediction, cache memory, and multi-cycle operations.
- Special thanks to the textbook : Sarah Harris, David Harris - Digital Design and Computer Architecture_ RISC-V Edition-Morgan Kaufmann (2021) that provide detailed explanations of pipelined processors and Verilog design.


