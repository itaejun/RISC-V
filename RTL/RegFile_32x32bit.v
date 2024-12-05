// 2-read 1-write register file with 32 32-bit registers
// Do not modify this code.
module RegFile_32x32bit(
    input   wire    [4:0]   i_read_addr0,   //5'd0 ~ 5'd31
    output  wire    [31:0]  o_read_data0,

    input   wire    [4:0]   i_read_addr1,   //5'd0 ~ 5'd31
    output  wire    [31:0]  o_read_data1,

    input   wire            i_write,        //active-high write request of write-port0
    input   wire    [4:0]   i_write_addr,   //5'd0 ~ 5'd31
    input   wire    [31:0]  i_write_data,

    input   wire            i_clk,          //clock
    input   wire            i_rst           //active-high async. reset
);
reg [31:0]  regs[0:31];

assign o_read_data0 = regs[i_read_addr0];
assign o_read_data1 = regs[i_read_addr1];

always @(posedge i_clk or posedge i_rst)
    if(i_rst) begin
        regs[0] <= #1 32'd0;  regs[1] <= #1 32'd0;  regs[2] <= #1 32'd0;  regs[3] <= #1 32'd0;
        regs[4] <= #1 32'd0;  regs[5] <= #1 32'd0;  regs[6] <= #1 32'd0;  regs[7] <= #1 32'd0;
        regs[8] <= #1 32'd0;  regs[9] <= #1 32'd0;  regs[10] <= #1 32'd0; regs[11] <= #1 32'd0;
        regs[12] <= #1 32'd0; regs[13] <= #1 32'd0; regs[14] <= #1 32'd0; regs[15] <= #1 32'd0;
        regs[16] <= #1 32'd0; regs[17] <= #1 32'd0; regs[18] <= #1 32'd0; regs[19] <= #1 32'd0;
        regs[20] <= #1 32'd0; regs[21] <= #1 32'd0; regs[22] <= #1 32'd0; regs[23] <= #1 32'd0;
        regs[24] <= #1 32'd0; regs[25] <= #1 32'd0; regs[26] <= #1 32'd0; regs[27] <= #1 32'd0;
        regs[28] <= #1 32'd0; regs[29] <= #1 32'd0; regs[30] <= #1 32'd0; regs[31] <= #1 32'd0;	
    end
    else if(i_write & (i_write_addr != 5'd0)) regs[i_write_addr] <= #1 i_write_data;

endmodule
