//-----------------------------------------------------------------------------
// riscv_decode.v  (UNCHANGED from the multicycle version)
//
// ID stage wrapper. Combines the combinational decode table (riscv_decoder.v),
// the register file read ports (riscv_regfile.v), and the immediate
// generator into the single module that represents the Instruction Decode
// pipeline stage, per the TRM Section 5.
//
// The register file's write port is driven straight through from the WB
// stage, so the same-cycle write-to-read bypass built into riscv_regfile.v
// resolves the WB -> ID hazard for free, on top of the EX-stage
// forwarding riscv_hazard.v now provides for the closer hazards.
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_decode
(
    input  wire                          clk_i,
    input  wire                          rst_i,

    // From IF/ID pipeline latch
    input  wire [31:0]                   instr_i,
    input  wire [`RISCV_XLEN-1:0]        pc_i,

    // Register file write port, driven by the WB stage
    input  wire [`RISCV_REG_ADDR_W-1:0]  wb_rd_addr_i,
    input  wire [`RISCV_XLEN-1:0]        wb_rd_data_i,
    input  wire                          wb_rd_wren_i,

    // ---------------- To EX stage / ID-EX pipeline register ----------------
    output wire [`RISCV_XLEN-1:0]        pc_o,
    output wire [`RISCV_REG_ADDR_W-1:0]  rd_addr_o,

    output wire [`RISCV_XLEN-1:0]        rs1_data_o,
    output wire [`RISCV_XLEN-1:0]        rs2_data_o,
    output wire [`RISCV_XLEN-1:0]        imm_o,

    output wire [`ALU_OP_W-1:0]          alu_op_o,
    output wire [`ALU_SRC_A_W-1:0]       alu_src_a_sel_o,
    output wire                          alu_src_b_imm_o,

    output wire                          mem_read_o,
    output wire                          mem_write_o,
    output wire [2:0]                    funct3_o,   // to LSU (width/sign) and branch unit

    output wire                          branch_o,
    output wire                          jump_o,
    output wire                          jalr_o,

    output wire                          reg_write_o,
    output wire [`WB_SEL_W-1:0]          wb_sel_o,

    output wire                          muldiv_en_o,
    output wire [`MULDIV_OP_W-1:0]       muldiv_op_o,

    output wire                          csr_en_o,
    output wire                          system_o,
    output wire                          illegal_o
);

    wire [`RISCV_REG_ADDR_W-1:0] rs1_addr;
    wire [`RISCV_REG_ADDR_W-1:0] rs2_addr;
    wire [`IMM_TYPE_W-1:0]       imm_type;

    //-------------------------------------------------------------------
    // Combinational decode table
    //-------------------------------------------------------------------
    riscv_decoder u_decoder
    (
        .instr_i         (instr_i),

        .rd_addr_o       (rd_addr_o),
        .rs1_addr_o      (rs1_addr),
        .rs2_addr_o      (rs2_addr),

        .imm_type_o      (imm_type),

        .alu_op_o        (alu_op_o),
        .alu_src_a_sel_o (alu_src_a_sel_o),
        .alu_src_b_imm_o (alu_src_b_imm_o),

        .mem_read_o      (mem_read_o),
        .mem_write_o     (mem_write_o),

        .branch_o        (branch_o),
        .jump_o          (jump_o),
        .jalr_o          (jalr_o),

        .reg_write_o     (reg_write_o),
        .wb_sel_o        (wb_sel_o),

        .muldiv_en_o     (muldiv_en_o),
        .muldiv_op_o     (muldiv_op_o),

        .csr_en_o        (csr_en_o),
        .system_o        (system_o),
        .illegal_o       (illegal_o)
    );

    //-------------------------------------------------------------------
    // Register file (2 read ports here; write port driven by WB stage)
    //-------------------------------------------------------------------
    riscv_regfile u_regfile
    (
        .clk_i      (clk_i),
        .rst_i      (rst_i),

        .rs1_addr_i (rs1_addr),
        .rs1_data_o (rs1_data_o),

        .rs2_addr_i (rs2_addr),
        .rs2_data_o (rs2_data_o),

        .rd_addr_i  (wb_rd_addr_i),
        .rd_data_i  (wb_rd_data_i),
        .rd_wren_i  (wb_rd_wren_i)
    );

    //-------------------------------------------------------------------
    // Immediate generator - sign/zero-extends per RV32 instruction format
    //-------------------------------------------------------------------
    reg [`RISCV_XLEN-1:0] imm_r;

    always @(*)
    begin
        case (imm_type)
            `IMM_I  : imm_r = {{20{instr_i[31]}}, instr_i[31:20]};
            `IMM_S  : imm_r = {{20{instr_i[31]}}, instr_i[31:25], instr_i[11:7]};
            `IMM_B  : imm_r = {{19{instr_i[31]}}, instr_i[31], instr_i[7],
                               instr_i[30:25], instr_i[11:8], 1'b0};
            `IMM_U  : imm_r = {instr_i[31:12], 12'b0};
            `IMM_J  : imm_r = {{11{instr_i[31]}}, instr_i[31], instr_i[19:12],
                               instr_i[20], instr_i[30:21], 1'b0};
            default : imm_r = {`RISCV_XLEN{1'b0}};  // IMM_NONE
        endcase
    end

    assign imm_o    = imm_r;
    assign pc_o     = pc_i;
    assign funct3_o = instr_i[`INSTR_FUNCT3_R];

endmodule