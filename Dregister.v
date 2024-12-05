//------------------------------------------------------------------------------
// Dflipflop array
//------------------------------------------------------------------------------
module Dregister #(
        parameter WIDTH = 1
)(
        input   wire    [WIDTH-1:0]     i,
        output  reg     [WIDTH-1:0]     o,
        input   wire                    i_clk
);

always @(posedge i_clk)
        o <= #1 i;

endmodule

