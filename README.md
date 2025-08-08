## RISC-V CPU

This is a single cycle RISC-V CPU built in SystemVerilog. It supports the following RV32I instructions: add (r type arithmetic), addi (i type arithmetic), lw (load word), and sw (store word).

## Architecture

**Fetch Module** - PC logic and instruction memory
**Decode Module** - Instruction parsin and control signals
**Register File** - 32 general purpose registers
**ALU** - Executes arithmetic operations
**Control Units** - Main control and ALU control
**Memory Interface** - Handles memory reads and writes

The associated waveforms can be seen in the cpu_waveform.png file.

License - MIT (feel free to use this, just give credit)
