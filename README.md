## RISC-V CPU

This is a single cycle RISC-V CPU built in SystemVerilog. It supports the following RV32I instructions: add (r type arithmetic), addi (i type arithmetic), lw (load word), and sw (store word). The associated waveforms can be seen in the cpu_waveform.png file.

## Architecture

**Fetch Module** - PC logic and instruction memory

**Decode Module** - Instruction parsing and control signals

**Register File** - 32 general purpose registers

**ALU** - Executes arithmetic operations

**Control Units** - Main control and ALU control

**Memory Interface** - Handles memory reads and writes

# Features

1. Implements a RISC-V RV32I subset including arithmetic instructions like ADD, SUB, and ADDI as well as load and store instructions

2. Design with separate stages including: Fetch, Decode, Register File, ALU, Memory Interface, and Control Units

3. One-cycle clocked operation with synchronous reset

# How it Works

FETCH STAGE - The program counter will increment by 4 each clock cycle and fetches the instruction from the instruction memory.

DECODE STAGE - This instruction is then decoded to extract the opcode, registers, funct3, funct7, and immediate values (i-type and s-type).

REGISTER FILE - This will read data from rs1 and rs2. It also supports writing back to rd on the rising edge of the clock (if enabled).

CONTROL UNITS - The main control decodes the opcode into control signals, while the ALU control generates the specific ALU operation from funct3 and funct7.

ALU OPERATION - This will perform the arithmetic or logic operation with either two reigsters or a register and immediate output.

MEMORY INTERFACE - Reading and writing 32-bit data words is supported, and addressing is word aligned using bits [9:2] of the address.

# Testbench

Clock period: 10ns

The reset signal starts high, then goes low after 10 time units, starting the normal operation. It will run for 100 time units.

# License

MIT License - feel free to use (just give credit)
