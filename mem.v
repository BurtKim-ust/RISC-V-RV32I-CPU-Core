// Memory Module
// Combined instruction and data memory
// Loads test program from riscvtest.txt

module mem(
    input        clk,
    input        we,
    input  [31:0] a,
    input  [31:0] wd,
    output [31:0] rd
);

    reg [31:0] RAM [0:127];  // 128 words of memory

    // Load test program from hex file
    initial begin
        $readmemh("riscvtest.txt", RAM);
    end

    // Synchronous write
    always @(posedge clk) begin
        if (we)
            RAM[a[31:2]] <= wd;
    end

    // Asynchronous read
    assign rd = RAM[a[31:2]];

endmodule
