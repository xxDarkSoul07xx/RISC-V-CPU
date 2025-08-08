module cpu_tb;
  logic clk; // Clock signal
  logic reset; // Reset signal
  
  // Instantiation and connecting the clock and reset
  full_cpu cpu (
    .clk(clk),
    .reset(reset)
  );
  
  // Generate a clock that toggles every 5 time units (period = 10)
  always #5 clk = ~clk;
  
  initial begin
    // Save the waveform
    $dumpfile("dump.vcd");
    $dumpvars(0, cpu_tb);
    
    $display("Started");
    clk = 0; // Initialize the clock
    reset = 1; // Start with reset high
    
    #10 reset = 0; // Wait 10 time units and then start the cpu's regular operation
    
    #100; // Wait 100 time units
    
    $display("Done");
    
    $finish;
  end
endmodule
