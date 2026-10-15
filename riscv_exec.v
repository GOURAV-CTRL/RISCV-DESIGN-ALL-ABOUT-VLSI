//-----------------------------------------------------------------------------
// riscv_exec.v
//
// EX stage wrapper. Instantiates riscv_alu.v behind an operand-source mux
// driven by the decoder's control signals, and adds the logic that is
// specific to the EX stage but doesn't belong in the ALU itself:
//   - PC+4 (link value for JAL/JALR, written back via WB_SEL_PC4)
//   - PC+imm (branch/JAL target, via its own dedicated adder so the ALU is
//     free to compute the branch condition at the same time)
//   - Branch condition resolution (funct3 + ALU compare result -> taken/not)
//   - JALR target LSB clear, per the RISC-V spec (target &= ~1)
//
// Per the TRM Section 5, "Branch/Jump Resolution": evaluates branch
// conditions and computes branch/jump target addresses in EX; issues
// pipeline redirect and flush signals.
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_exec
(
    input  wire [`RISCV_XLEN-1:0]   pc_i,
    input  wire [`RISCV_XLEN-1:0]   rs1_data_i,
    input  wire [`RISCV_XLEN-1:0]   rs2_data_i,
    input  wire [`RISCV_XLEN-1:0]   imm_i,

    // Control, from riscv_decode.v
    input  wire [`ALU_OP_W-1:0]     alu_op_i,
    input  wire [`ALU_SRC_A_W-1:0]  alu_src_a_sel_i,
    input  wire                     alu_src_b_imm_i,
    input  wire [2:0]               funct3_i,
    input  wire                     branch_i,
    input  wire                     jump_i,
    input  wire                     jalr_i,

    // ---------------- Results, to MEM/WB ----------------
    output wire [`RISCV_XLEN-1:0]   alu_result_o,   // ALU/address result
    output wire [`RISCV_XLEN-1:0]   pc_plus4_o,     // link value (WB_SEL_PC4)

    // ---------------- Pipeline redirect, to fetch/hazard unit ----------------
    output wire                     redirect_o,     // 1 = flush IF/ID, take redirect_pc_o
    output wire [`RISCV_XLEN-1:0]   redirect_pc_o
);

    //-------------------------------------------------------------------
    // ALU operand-A mux: rs1 (default) / PC (AUIPC, JAL) / zero (LUI)
    //-------------------------------------------------------------------
    reg [`RISCV_XLEN-1:0] operand_a;
    always @(*)
    begin
        case (alu_src_a_sel_i)
            `ALU_SRC_A_PC   : operand_a = pc_i;
            `ALU_SRC_A_ZERO : operand_a = {`RISCV_XLEN{1'b0}};
            default         : operand_a = rs1_data_i; // ALU_SRC_A_RS1
        endcase
    end

    //-------------------------------------------------------------------
    // ALU operand-B mux: rs2 or immediate
    //-------------------------------------------------------------------
    wire [`RISCV_XLEN-1:0] operand_b = alu_src_b_imm_i ? imm_i : rs2_data_i;

    //-------------------------------------------------------------------
    // ALU instance - shared by integer ops, address generation, and the
    // branch condition compare (SUB for eq/ne, SLT/SLTU for ordered)
    //-------------------------------------------------------------------
    riscv_alu u_alu
    (
        .alu_op_i    (alu_op_i),
        .operand_a_i (operand_a),
        .operand_b_i (operand_b),
        .result_o    (alu_result_o)
    );

    //-------------------------------------------------------------------
    // Dedicated adders: link value (PC+4) and branch/JAL target (PC+imm).
    // Kept separate from the ALU so a branch's condition-compare and its
    // target address can be computed in the same cycle.
    //-------------------------------------------------------------------
    assign pc_plus4_o = pc_i + 32'd4;

    wire [`RISCV_XLEN-1:0] pc_plus_imm = pc_i + imm_i;

    //-------------------------------------------------------------------
    // Branch condition resolution. The decoder has already selected the
    // ALU op (SUB for BEQ/BNE, SLT for BLT/BGE, SLTU for BLTU/BGEU); here
    // we just interpret that result against funct3 for taken/not-taken.
    //-------------------------------------------------------------------
    reg branch_taken;
    always @(*)
    begin
        case (funct3_i)
            `F3_BEQ  : branch_taken =  (alu_result_o == {`RISCV_XLEN{1'b0}});
            `F3_BNE  : branch_taken =  (alu_result_o != {`RISCV_XLEN{1'b0}});
            `F3_BLT  : branch_taken =  alu_result_o[0];   // SLT result is 0/1
            `F3_BGE  : branch_taken = ~alu_result_o[0];
            `F3_BLTU : branch_taken =  alu_result_o[0];   // SLTU result is 0/1
            `F3_BGEU : branch_taken = ~alu_result_o[0];
            default  : branch_taken = 1'b0;
        endcase
    end

    wire branch_taken_qual = branch_i & branch_taken;

    //-------------------------------------------------------------------
    // Redirect target: JALR uses rs1+imm (the ALU result) with the LSB
    // cleared per spec; branches and JAL use the dedicated PC+imm adder.
    //-------------------------------------------------------------------
    wire [`RISCV_XLEN-1:0] jalr_target = {alu_result_o[`RISCV_XLEN-1:1], 1'b0};

    assign redirect_o    = branch_taken_qual | jump_i;
    assign redirect_pc_o = jalr_i ? jalr_target : pc_plus_imm;

endmodule