module CORE(
    // Insterface with program (Instruction) memory
    output wire [31:0] im_addr,
    output wire        im_cs, 
    input wire  [31:0] inst,

    // Interface with data memory
    output wire [31:0] dm_addr,
    output wire        dm_cs,
    output wire [31:0] dm_we,
    output wire [31:0] dm_wdata,
    input  wire [31:0] dm_rdata,

    input wire clk,
    input wire rst
);
    // IF
    wire [31:0] pc_next, pc, target_addr;
    wire [31:0] PCSrc_mux_o;
    assign im_cs = 1'b1; // Inst Mem 활성화
    assign IF_flush = (target_addr != 32'b0) ? 1'b1 : 1'b0;
    wire [0:0] PCSrc_sel_i;

    Multiplexer #( // 2 to 1 Mux
    .SEL_WIDTH(1),
    .WIDTH(32)
    ) PCSrc_mux (
        .i({target_addr, pc_next}),
        .i_sel(PCSrc_sel_i),
        .o(PCSrc_mux_o)
    );
    // assign im_addr_IF11 = (IF_flush_IF_o == 1'h1) ? branch addr + 4 : im_addr_IF1;
    Dregister_we_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) pc_register (
        .i(PCSrc_mux_o),
        .o(pc), 
        .i_clk(clk),
        .i_rst(rst),
        .i_we(1'b1) // IF_flush_IF_o
    );
    assign pc_next = (jump_control) ? target_addr : pc + 4;

    assign im_addr = pc;

    // IF_ID
    wire [0:0] IF_flush, IF_flush_IF;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_IF_flush_IF (
        .i(IF_flush),
        .o(IF_flush_IF),
        .i_clk(clk),
        .i_rst(rst) // **********
    );
    wire [31:0] im_addr_ID, im_addr_ID1;
    wire [31:0] inst_ID, inst_ID1;
    // 명령어 레지스터
    Dregister_we_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0) // NOP
    ) Dreg_inst (
        .i(inst), // IF_Flush 시 NOP 입력
        .o(inst_ID1),
        .i_clk(clk),
        .i_rst(rst),
        .i_we(IF_IDWrite)
    );
    // 주소 레지스터
    Dregister_we_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0) // 초기화
    ) Dreg_im_addr_ID (
        .i(im_addr),
        .o(im_addr_ID1),
        .i_clk(clk),
        .i_rst(rst),
        .i_we(IF_IDWrite)
    );
    assign im_addr_ID = (IF_flush) ? 32'h0 : (im_addr_ID1);
    assign inst_ID = (IF_flush_IF) ? 32'h0 : (inst_ID1);
    // ID
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [4:0] rs1_addr, rs2_addr, rd_addr_ID;
    wire [31:0] imm;
    Hazard_Detection_Unit hazard_detection(
        .rd_addr_EX(rd_addr_EX),
        .opcode(opcode),
        .rd_addr_MEM(rd_addr_MEM),
        .rd_addr_WB(rd_addr_WB),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rs1_addr_EX(rs1_addr_EX),
        .rs2_addr_EX(rs2_addr_EX),
        .rst(rst),
        .MemRead(MemRead),
        .PCWrite(PCWrite),
        .IF_IDWrite(IF_IDWrite),
        .control_sel(control_sel)
    );
    
    Decode decode (
        .inst(inst_ID),
        .IF_flush(IF_flush),
        .IF_flush_IF(IF_flush_IF),
        .opcode(opcode),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(rd_addr_ID),
        .imm(imm),
        .funct3(funct3),
        .funct7(funct7)
    );

    wire [31:0] imm_gen_o;
    Imm_Gen imm_gen (
        .imm(imm),
        .opcode(opcode),
        .imm_gen_o(imm_gen_o)
    );

    wire [0:0] jump_control, jump_control_o;
    assign jump_control = (opcode == 7'b1101111 || opcode == 7'b1100111) ? 1'b1 : 1'b0;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_jump_control_o (
        .i(jump_control),
        .o(jump_control_o),
        .i_clk(clk),
        .i_rst(rst)
    );


    
    wire [0:0] RegWrite, PCSrc, MemWrite, MemRead, MemtoReg;
    wire [1:0] ALUOp;
    wire [31:0] return_addr_rd;
    Control control (
        .opcode(opcode),
        .ALUOp(ALUOp), // ex
        .PCSrc(PCSrc), // m
        .MemWrite(MemWrite), // m
        .MemRead(MemRead), // m
        .RegWrite(RegWrite), // wb
        .MemtoReg(MemtoReg), // wb

        .rs1_data(rs1_data),
        .rs2_data(rs2_data),
        .base_addr(pc),
        .offset(imm_gen_o),
        .target_addr(target_addr),
        .PCSrc_sel(PCSrc_sel),
        .IF_flush_IF(IF_flush_IF),

        .return_addr_rd(return_addr_rd)
    );
    assign PCSrc_sel_i = PCSrc_sel;
    
    wire [6:0] control_signal, control_mux_o;
    assign control_signal = {ALUOp,PCSrc,MemWrite,MemRead,RegWrite,MemtoReg};

   Multiplexer #( // 2 to 1 Mux
    .SEL_WIDTH(1),
    .WIDTH(7)
   ) Control_mux (
        .i({7'b0,control_signal}),
        .i_sel(control_sel),
        .o(control_mux_o)
    );

    wire [31:0] rs1_data, rs2_data, rd_data;
    RegFile_32x32bit reg_file(
        .i_read_addr0(rs1_addr),
        .o_read_data0(rs1_data),
        .i_read_addr1(rs2_addr),
        .o_read_data1(rs2_data),
        .i_write(RegWrite),                
        .i_write_addr(rd_addr_ID),   
        .i_write_data(rd_data),     
        .i_clk(clk),             
        .i_rst(rst)       
    );
    // ID_EX
    wire [0:0] RegWrite_EX;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_RegWrite_EX (
        .i(RegWrite),
        .o(RegWrite_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemtoReg_EX;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemtoReg_EX (
        .i(MemtoReg),
        .o(MemtoReg_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] PCSrc_EX;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_PCSrc_EX (
        .i(PCSrc),
        .o(PCSrc_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemWrite_EX;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemWrite_EX (
        .i(MemWrite),
        .o(MemWrite_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemRead_EX;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemRead_EX (
        .i(MemRead), //control_mux_o[4:4]
        .o(MemRead_EX),
        .i_clk(clk),
        .i_rst(rst)
    );

    wire [1:0] ALUOp_EX;
    Dregister_rst #(
        .WIDTH(2),
        .RESET_VALUE(2'h0)
    ) Dreg_ALUOp_EX (
        .i(ALUOp),
        .o(ALUOp_EX),
        .i_clk(clk),
        .i_rst(rst)
    );

    wire [31:0] im_addr_EX;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_im_addr_EX (
        .i(im_addr_ID),
        .o(im_addr_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] rs1_data_EX;
    wire [4:0] rs1_addr_EX;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_rs1_data (
        .i(rs1_data),
        .o(rs1_data_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    Dregister_rst #(
        .WIDTH(5),
        .RESET_VALUE(32'h0)
    ) Dreg_rs1_addr (
        .i(rs1_addr),
        .o(rs1_addr_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] rs2_data_EX;
    wire [4:0] rs2_addr_EX;
    Dregister_rst #(
        .WIDTH(5),
        .RESET_VALUE(32'h0)
    ) Dreg_rs2_addr (
        .i(rs2_addr),
        .o(rs2_addr_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_rs2_data (
        .i(rs2_data),
        .o(rs2_data_EX),
        .i_clk(clk),
        .i_rst(rst)
    );

    wire [31:0] imm_gen_o_EX;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_imm (
        .i(imm_gen_o),
        .o(imm_gen_o_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [2:0] funct3_EX;
    Dregister_rst #(
        .WIDTH(3),
        .RESET_VALUE(3'h0)
    ) Dreg_funct3_EX (
        .i(funct3),
        .o(funct3_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [6:0] funct7_EX;
    Dregister_rst #(
        .WIDTH(7),
        .RESET_VALUE(7'h0)
    ) Dreg_funct7_EX (
        .i(funct7),
        .o(funct7_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [4:0] rd_addr_EX;
    Dregister_rst #(
        .WIDTH(5),
        .RESET_VALUE(5'h0)
    ) Dreg_rd_addr_EX (
        .i(rd_addr_ID),
        .o(rd_addr_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [6:0] opcode_EX;
    Dregister_rst #(
        .WIDTH(7),
        .RESET_VALUE(7'h0)
    ) Dreg_opcode_EX (
        .i(opcode),
        .o(opcode_EX),
        .i_clk(clk),
        .i_rst(rst)
    );
    
    // EX
    wire [1:0] ForwardA_mux, ForwardB_mux;
    Forwarding_Unit forwarding_Unit(
        .rd_addr_MEM(rd_addr_MEM),
        .rd_addr_WB(rd_addr_WB),
        .RegWrite_MEM(RegWrite_MEM),
        .RegWrite_WB(RegWrite_WB),
        .rs1_addr_EX(rs1_addr_EX),
        .rs2_addr_EX(rs2_addr_EX),
        .opcode_EX(opcode_EX),
        .ForwardA_mux(ForwardA_mux),
        .ForwardB_mux(ForwardB_mux)
    );

     wire [31:0] ALU_mux_output1, ALU_mux_output2;
    Multiplexer #( // 4 to 1 Mux
    .SEL_WIDTH(2),
    .WIDTH(32)
    ) Forward_mux1 (
    .i({32'b0,ALU_result_MEM,rd_data,rs1_data}),
    .i_sel(ForwardA_mux),
    .o(ALU_mux_output1)
    );
    Multiplexer #( // 4 to 1 Mux
    .SEL_WIDTH(2),
    .WIDTH(32)
    ) Forward_mux2 (
    .i({imm_gen_o,ALU_result_MEM,rd_data,rs2_data}), 
    .i_sel(ForwardB_mux),
    .o(ALU_mux_output2)
    );

    wire [3:0] ALUcontrol_input;
    ALUcontrol alucontrol (
        .ALUOp(ALUOp),
        .funct3(funct3),
        .funct7(funct7),
        .ALUcontrol_input(ALUcontrol_input)
    );
    wire [31:0] ALU_result;
    wire [0:0] Zero;
    wire [31:0] mux_output;
    ALU alu (
        .ALU_mux_output1(ALU_mux_output1),
        .ALU_mux_output2(ALU_mux_output2),
        .ALUcontrol_input(ALUcontrol_input),
        .ALU_result(ALU_result)
    );

    /*wire [31:0] target_addr_j, offset2;
    assign offset2 = (opcode == 7'b1100111 || opcode == 7'b1101111 || opcode == 7'b110001) ?
                        (imm_gen_o - 32'h4) : 32'b0;
                        
    assign jump_control = (opcode == 7'b1101111) ? 1'b1 : 1'b0;
    ADDER_Jump Adder_jump (
        .offset(offset2),
        .opcode(opcode),
        .im_addr(im_addr),
        .rd_addr(rd_addr_ID),
        .target_addr_j(target_addr_j)
    );
     //assign target_addr = target_addr_j; */

    // EX_MEM
    wire [0:0] RegWrite_MEM;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_RegWrite_MEM (
        .i(RegWrite_EX),
        .o(RegWrite_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemtoReg_MEM;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemtoReg_MEM (
        .i(MemtoReg_EX),
        .o(MemtoReg_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );

    wire [0:0] PCSrc_MEM;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_PCSrc_MEM (
        .i(PCSrc_EX),
        .o(PCSrc_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemWrite_MEM;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemWrite_MEM (
        .i(MemWrite_EX),
        .o(MemWrite_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemRead_MEM;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemRead_MEM (
        .i(MemRead_EX),
        .o(MemRead_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] ALU_result_MEM;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_ALU_result_MEM (
        .i(ALU_result),
        .o(ALU_result_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] rs2_data_MEM;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_rs2_data_MEM (
        .i(rs2_data_EX),
        .o(rs2_data_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [4:0] rd_addr_MEM;
    Dregister_rst #(
        .WIDTH(5),
        .RESET_VALUE(5'h0)
    ) Dreg_rd_addr_MEM (
        .i(rd_addr_EX),
        .o(rd_addr_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [6:0] opcode_MEM;
    Dregister_rst #(
        .WIDTH(7),
        .RESET_VALUE(7'h0)
    ) Dreg_opcode_MEM (
        .i(opcode_EX),
        .o(opcode_MEM),
        .i_clk(clk),
        .i_rst(rst)
    );

    // MEM
    assign dm_addr = (MemWrite_MEM || MemRead_MEM) ? (ALU_result_WB) : 32'b0;
    assign dm_wdata = (dm_we) ? rs2_data_MEM : 32'b0;
    assign dm_cs = (MemRead_MEM == 1'b1 || MemWrite_MEM) ? 1'b1 : 1'b0;
    assign dm_we = (dm_cs == 1'b1 && MemRead_MEM == 1'b0 && MemWrite_MEM == 1'b1) ?
                    32'hFFFF_FFFF : 32'b0;
    
    // MEM_WB
    wire [0:0] RegWrite_WB;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_RegWrite_WB (
        .i(RegWrite_MEM),
        .o(RegWrite_WB),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [0:0] MemtoReg_WB;
    Dregister_rst #(
        .WIDTH(1),
        .RESET_VALUE(1'h0)
    ) Dreg_MemtoReg_WB (
        .i(MemtoReg_MEM),
        .o(MemtoReg_WB),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] dm_rdata_WB;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_dm_rdata_WB (
        .i(dm_rdata),
        .o(dm_rdata_WB),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [31:0] ALU_result_WB;
    Dregister_rst #(
        .WIDTH(32),
        .RESET_VALUE(32'h0)
    ) Dreg_ALU_result_WB (
        .i((ALU_result_MEM)),
        .o(ALU_result_WB),
        .i_clk(clk),
        .i_rst(rst)
    );
    wire [4:0] rd_addr_WB;
    Dregister_rst #(
        .WIDTH(5),
        .RESET_VALUE(5'h0)
    ) Dreg_rd_addr_WB (
        .i(rd_addr_MEM),
        .o(rd_addr_WB),
        .i_clk(clk),
        .i_rst(rst)
    );

    // WB
    wire [31:0] MemtoReg_mux_output;
    assign rd_addr = rd_addr_WB;
    assign rd_data = (rd_addr_ID == 5'h0) ? 32'h0 :
                    (RegWrite) ? ALU_result :
                    (jump_control) ? return_addr_rd : MemtoReg_mux_output;

    Multiplexer #( // 2 to 1 Mux
    .SEL_WIDTH(1),
    .WIDTH(32)
    ) MemtoReg_mux (
    .i({ALU_result_WB, dm_rdata_WB}),
    .i_sel(MemtoReg_WB),
    .o(MemtoReg_mux_output)
    );


endmodule

module Control (
    input wire [6:0] opcode,
    output wire [0:0] RegWrite,
    output wire [1:0] ALUOp,
    output wire [0:0] PCSrc,
    output wire [0:0] MemWrite,
    output wire [0:0] MemRead,
    output wire [0:0] MemtoReg,

    input wire [31:0] rs1_data,
    input wire [31:0] rs2_data,
    input wire [31:0] base_addr,
    input wire [31:0] offset,
    output wire [31:0] target_addr,
    output wire [0:0] PCSrc_sel,
    output wire [0:0] IF_flush_IF,

    output wire [31:0] return_addr_rd
);
    wire [0:0] zero;
    if(IF_flush_IF != 0) begin
        assign ALUOp = (opcode == 7'b0110011 || opcode == 7'b1101111|| opcode == 7'b1100111) ? 2'b10 :
                    (opcode == 7'b0000011 || opcode == 7'b0100011) ? 2'b00 :
                    (opcode == 7'b1100011) ? 2'b01 :
                    (opcode == 7'b0010011) ? 2'b11 :
                    2'b00; 

        assign PCSrc = (opcode == 7'b1100111) || (opcode == 7'b1101111 || opcode == 7'b1100011) ? 1'b1 :
                1'b0;

        assign MemRead = (opcode == 7'b0000011) ? 1'b1 : 1'b0; 

        assign MemWrite = (opcode == 7'b0100011) ? 1'b1 : 1'b0; 

        assign MemtoReg = (opcode == 7'b0000011) ? 1'b1 : 1'b0;

        assign RegWrite = (opcode == 7'b0110011 || opcode == 7'b0000011 || opcode == 7'b0010011 || 
                                opcode == 7'b1101111 || opcode == 7'b1100111) ? 1'b1 :1'b0; 

        assign target_addr = (opcode == 7'b1101111) ? (base_addr + offset - 32'h4) : // jalr
                        (opcode == 7'b1100111) ? (rs1_data + offset) : // jal
                        (opcode == 7'b1100011) ? (offset) : 32'b0;  // beq
                    
        assign zero = (rs1_data == rs2_data) ? 1'h1 : 1'h0;
        assign PCSrc_sel = (zero & PCSrc) ? 1'h1 : 1'h0;
        assign return_addr_rd = (opcode == 7'b1101111 || opcode == 7'b1100111) ? (base_addr + 4) : 32'h0;
    end
    else begin
        
    end
endmodule

module ALUcontrol (
    input wire [1:0] ALUOp,
    input wire [2:0] funct3,
    input wire [6:0] funct7,
    output wire [3:0] ALUcontrol_input
);
    assign ALUcontrol_input =
            (ALUOp == 2'b00 && funct3 == 3'b010) ? 4'b0010 : // lw
            (ALUOp == 2'b00 && funct3 == 3'b010) ? 4'b0010 : // sw
            (ALUOp == 2'b01 && funct3 == 3'b000) ? 4'b0110 : // beq
            (ALUOp == 2'b10 && funct3 == 3'b111 && funct7 == 7'b0) ? 4'b0000 : // and
            (ALUOp == 2'b10 && funct3 == 3'b110 && funct7 == 7'b0) ? 4'b0001 : // or
            (ALUOp == 2'b10 && funct3 == 3'b000 && funct7 == 7'b0) ? 4'b0010 : // add
            (ALUOp == 2'b10 && funct3 == 3'b000 && funct7 == 7'b0100000) ? 4'b0110 : // sub
            (ALUOp == 2'b11 && funct3 == 3'b000) ? 4'b0010 : // addi
            (ALUOp == 2'b11 && funct3 == 3'b111) ? 4'b0000 : // andi
            4'b0000; //default 
endmodule
module Hazard_Detection_Unit (
    input wire [4:0] rd_addr_EX,
    input wire [4:0] rd_addr_MEM,
    input wire [4:0] rd_addr_WB,
    input wire [4:0] rs1_addr,
    input wire [4:0] rs2_addr,
    input wire [4:0] rs1_addr_EX,
    input wire [4:0] rs2_addr_EX,
    input wire [0:0] MemRead,
    input wire [6:0] opcode,
    input wire [0:0] rst,
    output wire [0:0] PCWrite,
    output wire [0:0] IF_IDWrite,
    output wire [0:0] control_sel
);
    assign PCWrite = (~(rst) && ((rd_addr_MEM == rs1_addr_EX) || (rd_addr_MEM == rs2_addr_EX) 
                            || (rd_addr_WB == rs1_addr_EX) || (rd_addr_WB == rs2_addr_EX))) ? 1'b1 : 1'b0; // Active high forwarding

    assign IF_IDWrite = ((MemRead) && (rd_addr_EX != 0) && ((rd_addr_EX == rs1_addr) || (rd_addr_EX == rs2_addr))) ? 1'b0 : 1'b1; // Active high stall and forwarding

    assign control_sel = ((~IF_IDWrite) || PCWrite);
endmodule

module Forwarding_Unit (
    input wire [4:0] rd_addr_MEM,
    input wire [4:0] rd_addr_WB,
    input wire [0:0] RegWrite_MEM,
    input wire [0:0] RegWrite_WB,
    input wire [4:0] rs1_addr_EX,
    input wire [4:0] rs2_addr_EX,
    input wire [6:0] opcode_EX,
    output wire [1:0] ForwardA_mux,
    output wire [1:0] ForwardB_mux
);

    assign ForwardA_mux = (RegWrite_MEM && (rd_addr_MEM != 0) && (rd_addr_MEM == rs1_addr_EX)) ? 2'b10 :  // ALU_result sel
                            (RegWrite_WB && (rd_addr_WB != 0) && (rd_addr_WB == rs1_addr_EX)) ? 2'b01 : // Write_back_data sel
                            (opcode_EX == 7'b1101111 || opcode_EX == 7'b1100111) ? 2'b11 : 
                            2'b00; // Not hazard : rs1_data sel
    assign ForwardB_mux = (RegWrite_MEM && (rd_addr_MEM != 0) && (rd_addr_MEM == rs2_addr_EX)) ? 2'b10 :
                            (RegWrite_WB && (rd_addr_WB != 0) && (rd_addr_WB == rs2_addr_EX)) ? 2'b01 :
                            (opcode_EX == 7'b0010011 || opcode_EX == 7'b0100011 || opcode_EX == 7'b0000011) ? 2'b11 :
                            2'b00;

endmodule

module ALU(
    input wire [31:0] ALU_mux_output1,
    input wire [31:0] ALU_mux_output2,
    input wire [3:0] ALUcontrol_input,
    output wire [31:0] ALU_result
);
    assign ALU_result = 
            (ALUcontrol_input == 4'b0010) ? (ALU_mux_output1 + ALU_mux_output2) : // add
            (ALUcontrol_input == 4'b0110) ? (ALU_mux_output1 - ALU_mux_output2) : // sub
            (ALUcontrol_input == 4'b0000) ? (ALU_mux_output1 & ALU_mux_output2) : // and
            (ALUcontrol_input == 4'b0001) ? (ALU_mux_output1 | ALU_mux_output2) : // or
            32'b0; // Default value

endmodule

module Decode (
    input wire [31:0] inst,
    input wire [0:0] IF_flush, IF_flush_IF,
    output wire [6:0] opcode,
    output wire [4:0] rs1_addr,
    output wire [4:0] rs2_addr,
    output wire [4:0] rd_addr,
    output wire [31:0] imm,
    output wire [2:0] funct3,
    output wire [6:0] funct7
);
    assign opcode = inst[30:24];
    assign funct3 = inst[22:20];
    assign funct7 = (opcode == 7'b0110011) ? inst[7:1] : 7'b0;

    assign rs1_addr = {inst[11:8],inst[23:23]};
    assign rs2_addr = (opcode == 7'b0110011 || opcode == 7'b0100011 || opcode == 7'b1100011) ? {inst[0:0],inst[15:12]} : 
                     5'b0;
    assign rd_addr = ((opcode == 7'b1101111) || (opcode == 7'b1100111) || (opcode == 7'b0000011) || (opcode == 7'b0010011) || (opcode == 7'b0110011)) ? {inst[19:16],inst[31:31]} :
                    (IF_flush == 1'b0 || IF_flush_IF == 1'b0) ? 5'b0 :
                    5'b11110;
    assign imm = inst[31:0];

endmodule

module Imm_Gen (    
    input wire [31:0] imm,
    input wire [6:0] opcode,
    output wire [31:0] imm_gen_o
);
    assign imm_gen_o = (opcode == 7'b0010011) ? {imm[7:0],imm[15:12]} : // ADDI, ANDI
                (opcode == 7'b0100011) ? {imm[7:1],imm[19:16],imm[31]} : // SW
                (opcode == 7'b0000011) ? {imm[7:0],imm[15:12]} : // LW
                (opcode == 7'b1101111) ? ({imm[7:0],imm[15:8],imm[23:20]} >> 8 ) : // JAL
                (opcode == 7'b1100111) ? {imm[7:0],imm[15:12]} : // JALR
                (opcode == 7'b1100011) ? ({imm[7:1],imm[19:16],imm[31]} << 1) : // BEQ
                32'b0;
endmodule
