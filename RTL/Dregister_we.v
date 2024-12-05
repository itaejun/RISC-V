//------------------------------------------------------------------------------
// Dflipflop array with write enable 
//------------------------------------------------------------------------------
module Dregister_we #(
        parameter WIDTH = 1,
        parameter [WIDTH-1:0] RESET_VALUE = {WIDTH{1'b0}}
)(
        input   wire    [WIDTH-1:0]     i,
        output  reg     [WIDTH-1:0]     o,
        input   wire                    i_we,
        input   wire                    i_clk
);

always @(posedge i_clk)
        if(i_we) o <= #1 i;

endmodule

