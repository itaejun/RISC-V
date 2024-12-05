module LUT #(
    parameter INPUT_WIDTH = 3,
    parameter OUTPUT_WIDTH = 3,
    parameter ROM_FILE = ""
)(
    input   wire    [INPUT_WIDTH-1:0]   i,
    output  wire    [OUTPUT_WIDTH-1:0]  o
);

reg [OUTPUT_WIDTH-1:0] rom[0:(2**INPUT_WIDTH)-1];

initial
    $readmemb(ROM_FILE, rom);

assign o = rom[i];

endmodule

