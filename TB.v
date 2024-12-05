module TB ();
    localparam INITIAL_FILE = "test3.hex";
    localparam VCD_FILE = "TB.vcd";
    localparam CLK_PREIOD = 30;
    localparam RST_DELAY = CLK_PREIOD * 5.2;
    localparam SIMULATION_TIME = CLK_PREIOD * 1000;

    localparam IM_ADDR_WIDTH = 10;
    localparam DM_ADDR_WIDTH = 10;

    wire [31:0] im_addr;
    wire [31:0] inst;
    wire im_cs;

    wire [31:0] dm_addr;
    wire [31:0] dm_rdata;
    wire [31:0] dm_wdata;
    wire [31:0] dm_we;
    wire        dm_cs;

    reg rst;
    reg clk;

    CORE core(
        .im_addr    (im_addr),
        .im_cs      (im_cs),
        .inst       (inst),

        .dm_addr (dm_addr),
        .dm_cs   (dm_cs),
        .dm_we   (dm_we),
        .dm_wdata(dm_wdata),
        .dm_rdata(dm_rdata),

        .clk(clk),
        .rst(rst)
    );

SRAM #(
    .WIDTH(32),
    .ADDR_WIDTH(DM_ADDR_WIDTH)
)dm(
    .i_data(dm_wdata),
    .i_addr(dm_addr[DM_ADDR_WIDTH+1:2]),
    .o_data(dm_rdata),
    .i_cs(dm_cs),
    .i_we(dm_we),
    .i_clk(clk)
);

SRAM #(
    .INITIAL_FILE(INITIAL_FILE),
    .WIDTH(32),
    .ADDR_WIDTH(IM_ADDR_WIDTH)
)im(
    .i_data(32'bx),
    .i_addr(im_addr[IM_ADDR_WIDTH+1:2]),
    .o_data(inst),
    .i_cs(im_cs),
    .i_we(32'b0),
    .i_clk(clk)
);

initial begin
    rst = 1'b0;
#(RST_DELAY)
    rst = 1'b1;
#(CLK_PREIOD)
    rst = 1'b0;
end

initial begin
    clk = 0;
    forever #(CLK_PREIOD/2) clk =~clk;
end

initial begin
    $dumpfile(VCD_FILE);
    $dumpvars(0,TB);
#(SIMULATION_TIME)
    $dumpflush;
    $finish;
end

endmodule