// Single-port Synchronous RAM
// Size: WIDTH x 2^(ADDR_WIDTH) bits
// Do not modify this code.
module SRAM #(
    parameter INITIAL_FILE  = "",   // If this parameter is given, the memory will be 
                                    // initialized with the data stored in the file.
    parameter WIDTH         = 32,   // word width
    parameter ADDR_WIDTH    = 10    // depth of the memory: 2^(ADDR_WIDTH-1)
)(
    input   wire    [WIDTH-1:0]         i_data,
    input   wire    [ADDR_WIDTH-1:0]    i_addr,
    output  reg     [WIDTH-1:0]         o_data,
    input   wire                        i_cs,   //active-high chip select
    input   wire    [WIDTH-1:0]         i_we,   //active-high write-enable. all-zero means read mode.

    input   wire                        i_clk
);

localparam DEPTH = 2**ADDR_WIDTH;
reg [WIDTH-1:0] mem[0:DEPTH-1];

always @(posedge i_clk)
    if(i_cs & (~|i_we)) o_data <= #1 mem[i_addr];
always @(posedge i_clk)
    if(i_cs & (|i_we))
        mem[i_addr] <= #1 (~i_we & mem[i_addr]) | (i_we & i_data);
initial begin
    if(INITIAL_FILE != "") begin
        $readmemh(INITIAL_FILE, mem);
        $display("%s was loaded successfully.", INITIAL_FILE);
    end
end

endmodule


