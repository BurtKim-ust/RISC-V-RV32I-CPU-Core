# RISC-V Multi-Cycle CPU Core

A simplified 32-bit RISC-V processor implementation designed and simulated using Verilog. This CPU implements a subset of the RV32I base integer instruction set using a multi-cycle architecture.

## Architecture Overview

This is a **multi-cycle CPU** design that executes instructions over multiple clock cycles, utilizing a finite state machine (FSM) controller with 11 distinct states:
- Fetch
- Decode
- Memory Address Calculation
- Memory Read/Write
- Execute (R-type and I-type)
- ALU Write-back
- JAL (Jump and Link)
- BEQ (Branch if Equal)

### Key Components

1. **Datapath (`datapath.v`)**
   - 32-bit ALU with support for arithmetic and logic operations
   - 32-register register file (x0-x31)
   - Program Counter (PC)
   - Instruction register
   - Data memory interface
   - Multi-stage pipeline registers

2. **Controller (`controller.v`)**
   - FSM-based control logic
   - Instruction decoder
   - Control signal generator for datapath

3. **ALU Operations**
   - Addition (ADD)
   - Subtraction (SUB)
   - Bitwise AND
   - Bitwise OR
   - Set Less Than (SLT)

4. **riscvmulti (`riscvmulti.v`)**
   - Simply datapath + controller + top module
   - This can be used for hardware synthesis

## Structures
![alt text](../images/fig1.png)
![alt text](../images/fig3.png)

###  What This CPU CAN Do

#### Memory Operations
- **LW** (Load Word) - Load 32-bit word from memory
- **SW** (Store Word) - Store 32-bit word to memory

#### Arithmetic & Logic (R-Type)
- **ADD** - Addition (rd = rs1 + rs2)
- **SUB** - Subtraction (rd = rs1 - rs2)
- **AND** - Bitwise AND (rd = rs1 & rs2)
- **OR** - Bitwise OR (rd = rs1 | rs2)
- **SLT** - Set Less Than (rd = (rs1 < rs2) ? 1 : 0)

#### Immediate Operations (I-Type)
- **ADDI** - Add immediate (rd = rs1 + imm)
- **SLTI** - Set less than immediate (rd = (rs1 < imm) ? 1 : 0)
- **ANDI** - Bitwise AND with immediate
- **ORI** - Bitwise OR with immediate

#### Control Flow
- **BEQ** - Branch if equal (if rs1 == rs2, PC = PC + offset)
- **JAL** - Jump and link (rd = PC + 4, PC = PC + offset)

## How to test
'''
iverilog -o riscvmulti.vvp -g2012 controller.v datapath.v mem.v testbench.v
vvp riscvmulti.vvp
'''
You should be seeing "Simulation succeeded"