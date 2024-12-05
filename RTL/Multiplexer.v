//------------------------------------------------------------------------------
// Multiplexer
//------------------------------------------------------------------------------
module Multiplexer #(
        parameter SEL_WIDTH = 2,
        parameter WIDTH = 1
)(
        input   wire    [(2**SEL_WIDTH)*WIDTH-1:0]      i,
        input   wire    [SEL_WIDTH-1:0]                 i_sel,
        output  wire    [WIDTH-1:0]                     o
);

genvar idx1;
genvar idx2;
generate
        for(idx1=0; idx1<WIDTH; idx1=idx1+1) begin: genblk1
                wire    [2**SEL_WIDTH-1:0]      anded;
                for(idx2=0; idx2<2**SEL_WIDTH; idx2=idx2+1) begin: genblk2
                        assign  anded[idx2] 
                                = i[idx2*WIDTH+idx1] & (i_sel == idx2);
                end
                assign  o[idx1] = |anded;
        end
endgenerate

endmodule

