//------------------------------------------------------------------------------
// Dflipflop array with reset 
//------------------------------------------------------------------------------
module Dregister_rst #(
        parameter WIDTH = 1,
        parameter [WIDTH-1:0] RESET_VALUE = {WIDTH{1'b0}}
)(
        input   wire    [WIDTH-1:0]     i,
        output  reg     [WIDTH-1:0]     o,
        input   wire                    i_clk,
        input   wire                    i_rst
);

always @(posedge i_clk or posedge i_rst)
        if(i_rst) o <= #1 RESET_VALUE;
        else o <= #1 i;

endmodule

