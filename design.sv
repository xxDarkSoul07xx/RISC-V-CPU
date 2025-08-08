module fetch (
  input logic clk, // Clock
  input logic reset, // Reset
  output logic [31:0] pc_out, // Outputs the program counter's current value
  output logic [31:0] instruction_out // Outputs the current instruction from the memory
);
  
  logic [31:0] pc; // Program counter's address
  logic [31:0] instruction_memory [0:255]; // 256 entries, 32 bits each (1 word)
  
  // If reset is high, reset everything
  // Else, increment program counter by 4 bytes(1 word = 32 bits)
  always_ff @(posedge clk or posedge reset) begin
    if (reset)
      pc <= 32'h00000000;
    else
      pc <= pc + 4;
  end
  
  // Connect the current pc value to the output port
  assign pc_out = pc;
  
  // Simulate a program that is stored inside of the memory
  initial begin
    instruction_memory[0] = 32'h00500093; // ADDI x1, x0, 5
    instruction_memory[1] = 32'h00a00113; // ADDI x2, x0, 10
    instruction_memory[2] = 32'h002081b3; // ADD x3, x1, x2
    instruction_memory[3] = 32'h00312023; // SW x3, 0(x2)
    instruction_memory[4] = 32'h00012183; // LW x3, 0(x2)
  end
  
  // Bits 0 and 1 of a 4 byte address are always going to be 00, so we can use the upper 8 bits
  assign instruction_out = instruction_memory[pc[9:2]];
endmodule

module decode (
  input logic [31:0] instruction_in, // Instruction input
  output logic [6:0] opcode, // Operation code
  output logic [4:0] rd, // Destination register
  output logic [2:0] funct3, // Operation subtype
  output logic [4:0] rs1, // Source register 1
  output logic [4:0] rs2, // Source reigster 2
  output logic [6:0] funct7, // R type
  output logic [31:0] immediate_out, // For I type
  output logic [31:0] s_immediate_out, // For S type
  output logic is_rtype, // R type?
  output logic is_itype, // I type?
  output logic is_load,  // Load type?
  output logic is_stype  // S type?
);
  
  // Field positions
  assign opcode = instruction_in[6:0];
  assign rd = instruction_in[11:7];
  assign funct3 = instruction_in[14:12];
  assign rs1 = instruction_in[19:15];
  assign rs2 = instruction_in[24:20];
  assign funct7 = instruction_in[31:25];
  
  assign is_rtype = (opcode == 7'b0110011); // Can add, subtract, etc
  assign is_itype = (opcode == 7'b0010011); // Can do addi and related
  assign is_load = (opcode == 7'b0000011); // LW
  assign is_stype = (opcode == 7'b0100011); // SW
  
  // Gets the 12 bit immediate (i type) from the instructions
  // Extend to 32 bits using bit 11 (msb)
  logic [11:0] immediate_i;
  assign immediate_i = instruction_in[31:20];
  assign immediate_out = {{20{immediate_i[11]}}, immediate_i};
  
  // Gets the 12 bit immediate (s type) from the instructions
  // Extend to 32 bits using 11 bits (msb)
  logic [11:0] immediate_s;
  assign immediate_s = {instruction_in[31:25], instruction_in[11:7]};
  assign s_immediate_out = {{20{immediate_s[11]}}, immediate_s};
endmodule

module register_file (
  input logic clk, // Clock
  input logic write_enable, // Allows us to write to the register
  input logic [4:0] rs1_address, // Address of the first source register
  input logic [4:0] rs2_address, // Address of the second source register
  input logic [4:0] rd_address, // Address of the destination register
  input logic [31:0] write_data, // Data that needs to be written to rd
  output logic [31:0] rs1_data, // Data read from rs1
  output logic [31:0] rs2_data // Data read from rs2
);
  
  logic [31:0] registers [31:0]; // 32 registers, each 32 bits
  
  // If either of the addresses are 0, return 0
  // Else return the value that is in the register
  assign rs1_data = (rs1_address == 5'd0) ? 32'd0 : registers[rs1_address];
  assign rs2_data = (rs2_address == 5'd0) ? 32'd0 : registers[rs2_address];
  
  // On rising edge of the clock, if write_enable is high and rd_address is not 0, then write the data to a register
  always_ff @(posedge clk) begin
    if (write_enable && rd_address != 5'd0)
      registers[rd_address] <= write_data;
  end
endmodule

module alu (
  input logic [31:0] operand_a,     // From rs1
  input logic [31:0] operand_b,     // From rs2 or immediate
  input logic [2:0] alu_ctrl,       // ALU control signal
  output logic [31:0] alu_result    // Result
);
  
  always_comb begin
    case (alu_ctrl)
      3'b000: alu_result = operand_a + operand_b; // ADD or ADDI
      3'b001: alu_result = operand_a - operand_b; // SUB
      // By default, the result is 0
      default: alu_result = 32'd0;
    endcase
  end
endmodule


module memory_interface (
  input logic clk, // Clock signal
  input logic reset, // Reset signal
  input logic memory_read, // Enable to read from memory
  input logic memory_write, // Enable to write to memory
  input logic [31:0]  address, // Memory address from the ALU
  input logic [31:0] write_data, // Value from rs2 to store on SW
  output logic [31:0] read_data // Value that needs to be outputted on LW
);
  
  // 256 32 bit words (1KB)
  logic [31:0] memory [0:255];
  
  integer i;
  
  // If reset is high, set everything in the memory to decimal 0
  // If reset is not high and memory_write is enabled, write the rs2_data into memory at the address specified by the ALU
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      for (i = 0; i < 256; i = i + 1)
        memory[i] <= 32'd0;
    end else if (memory_write) begin
      memory[address[9:2]] <= write_data;
    end
  end
  
  // If memory_read is enabled, set read_data to memory at index address[9:2]
  // Else, set read_data to decimal 0
  // read_data is updated if is_load or address changes
  always_comb begin
    if (memory_read)
      read_data = memory[address[9:2]];
    else
      read_data = 32'd0;
  end
endmodule

module main_control (
  input logic [6:0] opcode, // Opcode from the instruction
  output logic register_write, // Enable for writing to register
  output logic alu_source, // Chooses either register or immediate as the ALU input
  output logic memory_read, // Enable for reading from data memory
  output logic memory_write, // Enable for writing to data memory
  output logic memory_to_register, // Chooses data from either memory or ALU to write back
  output logic [1:0] alu_operation // Operation the ALU is performing
);
  
  // By default, all signals are set to 0
  always_comb begin
    register_write = 0;
    alu_source = 0;
    memory_read = 0;
    memory_write = 0;
    memory_to_register = 0;
    alu_operation = 2'b00;
    
    case (opcode)
      7'b0110011: begin // R type - add, sub
        register_write = 1; // Write result back to the register
        alu_source = 0; // ALU input will come from the register
        alu_operation = 2'b10; // Determine the operation with funct3 and funct7
      end
      
      7'b0010011: begin // I type - addi
        register_write = 1; // Write result back to the register
        alu_source = 1; // ALU input will come from the immediate
        alu_operation = 2'b00; // ALU will do basic addition
      end
      
      7'b0000011: begin // Load - lw
        register_write = 1; // Write loaded value back to the register
        alu_source = 1; // ALU input will come from the immediate
        memory_read = 1; // Read from the data memory
        memory_to_register = 1; // Write the memory data to the register
        alu_operation = 2'b00; // ALU adds
      end
      
      7'b0100011: begin // Store - sw
        alu_source = 1; // ALU input will come from the immediate
        memory_write = 1; // Enable memory write
        alu_operation = 2'b00; // ALU will add
      end
      
      
      // By default, keep everything default (0)
      default: begin
      end
    endcase
  end
endmodule

module alu_control (
  input logic [1:0] alu_operation, // Operation the ALU does (from main control)
  input logic [2:0] funct3, // Instruction bits [14:12]
  input logic [6:0] funct7, // INstruction bits [31:25]
  output logic [2:0] alu_ctrl // ALU control signal
);
  
  always_comb begin
    case (alu_operation)
      2'b00: alu_ctrl = 3'b000; // Add (used for load, store, and addi)
      
      2'b10: begin // R type, so decode funct3 and funct7
        if (funct7 == 7'b0000000 && funct3 == 3'b000)
          alu_ctrl = 3'b000; // Add
        else if (funct7 == 7'b0100000 && funct3 == 3'b000)
          alu_ctrl = 3'b001; // Subtract
        else
          alu_ctrl = 3'b111; // Error/undefined
      end
      
      default: alu_ctrl = 3'b000; // By default, add
    endcase
  end
endmodule

module full_cpu (
  input logic clk,
  input logic reset
);
  
  logic [31:0] pc;
  logic [31:0] instruction;
 
  logic [6:0] opcode;
  logic [4:0] rd, rs1, rs2;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic [31:0] imm_i;
  logic [31:0] imm_s;
  logic is_rtype, is_itype, is_load, is_stype;
  logic [31:0] rs1_data, rs2_data;
  logic register_write;
  logic alu_source;
  logic memory_read, memory_write;
  logic memory_to_register;
  logic [1:0] alu_operation;
  logic [2:0] alu_ctrl;
  logic [31:0] operand_b;
  logic [31:0] alu_result;
  logic [31:0] read_data;
  logic [31:0] write_back_data;
  
  // Fetch
  fetch fetch_module (
    .clk(clk),
    .reset(reset),
    .pc_out(pc),
    .instruction_out(instruction)
  );
 
  // Decode
  decode decode_module (
    .instruction_in(instruction),
    .opcode(opcode),
    .rd(rd),
    .funct3(funct3),
    .rs1(rs1),
    .rs2(rs2),
    .funct7(funct7),
    .immediate_out(imm_i),
    .s_immediate_out(imm_s),
    .is_rtype(is_rtype),
    .is_itype(is_itype),
    .is_load(is_load),
    .is_stype(is_stype)
  );

  // Register File
  register_file register_file_module (
    .clk(clk),
    .write_enable(register_write),
    .rs1_address(rs1),
    .rs2_address(rs2),
    .rd_address(rd),
    .write_data(write_back_data),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data)
  );
  
  // Main Control
  main_control main_control_module (
    .opcode(opcode),
    .register_write(register_write),
    .alu_source(alu_source),
    .memory_read(memory_read),
    .memory_write(memory_write),
    .memory_to_register(memory_to_register),
    .alu_operation(alu_operation)
  );

  // ALU
  alu_control alu_control_module (
    .alu_operation(alu_operation),
    .funct3(funct3),
    .funct7(funct7),
    .alu_ctrl(alu_ctrl)
  );

 assign operand_b = alu_source ? (is_stype ? imm_s : imm_i) : rs2_data;


  // ALU
  alu alu_module (
    .operand_a(rs1_data),
    .operand_b(operand_b),
    .alu_ctrl(alu_ctrl),
    .alu_result(alu_result)
  );

  // Memory Interface
  memory_interface memory_interface_module (
    .clk(clk),
    .reset(reset),
    .memory_read(memory_read),
    .memory_write(memory_write),
    .address(alu_result),
    .write_data(rs2_data),
    .read_data(read_data)
  );

  // Write back mux: choose memory output or ALU result
  assign write_back_data = memory_to_register ? read_data : alu_result;
endmodule
