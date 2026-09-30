//-----------------------------------------------------------------------------
// riscv_decoder.v
//
// Combinational opcode/funct3/funct7 decode table. Takes a raw 32-bit
// instruction and produces the control signals needed by the rest of the
// pipeline (ALU op + operand-source selects, memory/branch/jump control,
// writeback source select, RV32M and CSR enables). Purely combinational,
// no state, no register file access - see riscv_decode.v for the ID-stage
// wrapper that uses this table.
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable, purely combinational
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_decoder
(
    input  wire [31:0]                   instr_i,
    // Raw register address fields (meaningless for U/J-type, harmless to output)
    output wire [`RISCV_REG_ADDR_W-1:0]  rd_addr_o,
    output wire [`RISCV_REG_ADDR_W-1:0]  rs1_addr_o,
    output wire [`RISCV_REG_ADDR_W-1:0]  rs2_addr_o,
    // Immediate format select, for the ID-stage immediate generator
    output reg  [`IMM_TYPE_W-1:0]        imm_type_o,
    // ALU control
    output reg  [`ALU_OP_W-1:0]          alu_op_o,
    output reg  [`ALU_SRC_A_W-1:0]       alu_src_a_sel_o,  // rs1 / PC / zero
    output reg                           alu_src_b_imm_o,  // 0=rs2, 1=immediate
    // Memory control (LSU)
    output reg                           mem_read_o,
    output reg                           mem_write_o,
    // Control flow
    output reg                           branch_o,
    output reg                           jump_o,   // JAL or JALR
    output reg                           jalr_o,   // specifically JALR
    // Writeback control
    output reg                           reg_write_o,
    output reg  [`WB_SEL_W-1:0]          wb_sel_o, // this will select where is the data coming that should be written back to the register
    // RV32M
    output reg                           muldiv_en_o,
    output reg  [`MULDIV_OP_W-1:0]       muldiv_op_o,
    // SYSTEM / CSR
    //these signals are used to indicated what kind of instructions are these
    output reg                           csr_en_o,
    output reg                           system_o,   // ECALL/EBREAK/MRET (non-CSR SYSTEM ops)
    output reg                           illegal_o
);

    wire [6:0] opcode = instr_i[`INSTR_OPCODE_R];
    wire [2:0] funct3 = instr_i[`INSTR_FUNCT3_R];
    wire [6:0] funct7 = instr_i[`INSTR_FUNCT7_R];

    assign rd_addr_o  = instr_i[`INSTR_RD_R];
    assign rs1_addr_o = instr_i[`INSTR_RS1_R];
    assign rs2_addr_o = instr_i[`INSTR_RS2_R];

    always @(*)
    begin
        
        imm_type_o      = `IMM_NONE;
        alu_op_o        = `ALU_ADD;
        alu_src_a_sel_o = `ALU_SRC_A_RS1;
        alu_src_b_imm_o = 1'b0;
        mem_read_o      = 1'b0;
        mem_write_o     = 1'b0;
        branch_o        = 1'b0;
        jump_o          = 1'b0;
        jalr_o          = 1'b0;
        reg_write_o     = 1'b0;
        wb_sel_o        = `WB_SEL_ALU;
        muldiv_en_o     = 1'b0;
        muldiv_op_o     = {`MULDIV_OP_W{1'b0}};
        csr_en_o        = 1'b0;
        system_o        = 1'b0;
        illegal_o       = 1'b0;

        case (opcode)

            //-----------------------------------------------------------
            // OP-IMM: ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI
            //-----------------------------------------------------------
            `OPCODE_OP_IMM:
            begin
                imm_type_o      = `IMM_I;
                alu_src_b_imm_o = 1'b1;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_ALU;
                case (funct3)
                    `F3_ADD_SUB: alu_op_o = `ALU_ADD;  // ADDI (no SUBI in RV32I)
                    `F3_SLT    : alu_op_o = `ALU_SLT;
                    `F3_SLTU   : alu_op_o = `ALU_SLTU;
                    `F3_XOR    : alu_op_o = `ALU_XOR;
                    `F3_OR     : alu_op_o = `ALU_OR;
                    `F3_AND    : alu_op_o = `ALU_AND;
                    `F3_SLL    : alu_op_o = `ALU_SLL;
                    `F3_SRL_SRA: alu_op_o = (funct7 == `F7_SUB_SRA) ? `ALU_SRA : `ALU_SRL;
                    default    : illegal_o = 1'b1;
                endcase
            end

            //-----------------------------------------------------------
            // OP: ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND + RV32M
            //-----------------------------------------------------------
            `OPCODE_OP:
            begin
                reg_write_o = 1'b1;
                if (funct7 == `F7_MULDIV)
                begin
                    muldiv_en_o = 1'b1;
                    wb_sel_o    = `WB_SEL_MULDIV;
                    case (funct3)
                        `F3_MUL   : muldiv_op_o = `MULDIV_MUL; // signed * signed lower 32 bits
                        `F3_MULH  : muldiv_op_o = `MULDIV_MULH; //signed * signed upper 32 bits
                        `F3_MULHSU: muldiv_op_o = `MULDIV_MULHSU; // signed * unsigned upper 32 bits
                        `F3_MULHU : muldiv_op_o = `MULDIV_MULHU; //unsigned * unsigned upper 32 bits
                        `F3_DIV   : muldiv_op_o = `MULDIV_DIV; //division operation
                        `F3_DIVU  : muldiv_op_o = `MULDIV_DIVU; //unsigned division
                        `F3_REM   : muldiv_op_o = `MULDIV_REM; //signed remainder
                        `F3_REMU  : muldiv_op_o = `MULDIV_REMU; //unsigned remainder
                        default   : illegal_o   = 1'b1;
                    endcase
                end
                else
                begin
                    wb_sel_o = `WB_SEL_ALU;
                    case (funct3)
                        `F3_ADD_SUB: alu_op_o = (funct7 == `F7_SUB_SRA) ? `ALU_SUB : `ALU_ADD;
                        `F3_SLT    : alu_op_o = `ALU_SLT;
                        `F3_SLTU   : alu_op_o = `ALU_SLTU;
                        `F3_XOR    : alu_op_o = `ALU_XOR;
                        `F3_OR     : alu_op_o = `ALU_OR;
                        `F3_AND    : alu_op_o = `ALU_AND;
                        `F3_SLL    : alu_op_o = `ALU_SLL;
                        `F3_SRL_SRA: alu_op_o = (funct7 == `F7_SUB_SRA) ? `ALU_SRA : `ALU_SRL;
                        default    : illegal_o = 1'b1;
                    endcase
                end
            end

            //-----------------------------------------------------------
            // LOAD: LB/LH/LW/LBU/LHU  -  address = rs1 + imm
            //-----------------------------------------------------------
            `OPCODE_LOAD:
            begin
                imm_type_o      = `IMM_I;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                mem_read_o      = 1'b1;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_MEM;
                case (funct3)
                    `F3_LB, `F3_LH, `F3_LW, `F3_LBU, `F3_LHU: ; // valid widths
                    default: illegal_o = 1'b1;
                endcase
            end

            //-----------------------------------------------------------
            // STORE: SB/SH/SW  -  address = rs1 + imm
            //-----------------------------------------------------------
            `OPCODE_STORE:
            begin
                imm_type_o      = `IMM_S;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                mem_write_o     = 1'b1;
                case (funct3)
                    `F3_SB, `F3_SH, `F3_SW: ; // valid widths
                    default: illegal_o = 1'b1;
                endcase
            end

            //-----------------------------------------------------------
            // BRANCH: BEQ/BNE/BLT/BGE/BLTU/BGEU
            // ALU performs the compare; exec stage interprets the result
            // together with funct3 to decide taken/not-taken. The branch
            // target itself (PC + imm) is computed by a separate adder in
            // the EX stage, not through this ALU op.
            //-----------------------------------------------------------
            `OPCODE_BRANCH:
            begin
                imm_type_o = `IMM_B;
                branch_o   = 1'b1;
                case (funct3)
                    `F3_BEQ , `F3_BNE : alu_op_o = `ALU_SUB;
                    `F3_BLT , `F3_BGE : alu_op_o = `ALU_SLT;
                    `F3_BLTU, `F3_BGEU: alu_op_o = `ALU_SLTU;
                    default           : illegal_o = 1'b1;
                endcase
            end

            //-----------------------------------------------------------
            // JAL: rd = PC+4 (via WB_SEL_PC4); ALU computes jump target
            // (PC + imm) so the fetch redirect can reuse the same adder.
            //-----------------------------------------------------------
            `OPCODE_JAL:
            begin
                imm_type_o      = `IMM_J;
                jump_o          = 1'b1;
                alu_src_a_sel_o = `ALU_SRC_A_PC;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_PC4;
            end

            //-----------------------------------------------------------
            // JALR: rd = PC+4; ALU computes jump target (rs1 + imm).
            // Target LSB clear (per spec) is handled in the EX stage.
            //-----------------------------------------------------------
            `OPCODE_JALR:
            begin
                imm_type_o      = `IMM_I;
                jump_o          = 1'b1;
                jalr_o          = 1'b1;
                alu_src_a_sel_o = `ALU_SRC_A_RS1;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_PC4;
                if (funct3 != 3'b000)
                    illegal_o = 1'b1;
            end

            //-----------------------------------------------------------
            // LUI: rd = imm (0 + imm)
            //-----------------------------------------------------------
            `OPCODE_LUI:
            begin
                imm_type_o      = `IMM_U;
                alu_src_a_sel_o = `ALU_SRC_A_ZERO;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_ALU;
            end

            //-----------------------------------------------------------
            // AUIPC: rd = PC + imm
            //-----------------------------------------------------------
            `OPCODE_AUIPC:
            begin
                imm_type_o      = `IMM_U;
                alu_src_a_sel_o = `ALU_SRC_A_PC;
                alu_src_b_imm_o = 1'b1;
                alu_op_o        = `ALU_ADD;
                reg_write_o     = 1'b1;
                wb_sel_o        = `WB_SEL_ALU;
            end

            //-----------------------------------------------------------
            // SYSTEM: CSRRW/S/C[I] and ECALL/EBREAK/MRET
            // No standard sign-extended immediate; CSR address and the
            // optional 5-bit uimm (for *I forms) are read directly from
            // the instruction by riscv_csr.v.
            //-----------------------------------------------------------
            `OPCODE_SYSTEM:
            begin
                system_o   = 1'b1;
                imm_type_o = `IMM_NONE;
                if (funct3 == `F3_PRIV)
                begin
                    // ECALL / EBREAK / MRET distinguished by instr[31:20]
                    // in riscv_csr.v; no regfile/ALU activity here.
                end
                else
                begin
                    csr_en_o    = 1'b1;
                    reg_write_o = 1'b1;   // regfile silently drops x0 writes
                    wb_sel_o    = `WB_SEL_CSR;
                    case (funct3)
                        `F3_CSRRW, `F3_CSRRS, `F3_CSRRC,
                        `F3_CSRRWI, `F3_CSRRSI, `F3_CSRRCI: ; // valid CSR ops
                        default: illegal_o = 1'b1;
                    endcase
                end
            end

            //-----------------------------------------------------------
            // MISC-MEM: FENCE / FENCE.I - no-op at this pipeline depth
            //-----------------------------------------------------------
            `OPCODE_MISC_MEM: ;

            default: illegal_o = 1'b1;

        endcase
    end

endmodule