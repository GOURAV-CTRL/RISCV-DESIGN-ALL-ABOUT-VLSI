//-----------------------------------------------------------------------------
// riscv_alu.v
//
// Combinational arithmetic/logic unit. Implements the base RV32I ALU
// operations (add/sub, shifts, set-less-than, bitwise). Used for both
// EX-stage results and effective-address generation (loads/stores,
// branch targets) as described in the TRM, Section 5.
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable, purely combinational
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_alu
(
    input  wire [`ALU_OP_W-1:0]     alu_op_i,      // Operation select, see riscv_defs.v
    input  wire [`RISCV_XLEN-1:0]   operand_a_i,   // rs1 (or PC, for AUIPC/branch targets)
    input  wire [`RISCV_XLEN-1:0]   operand_b_i,   // rs2 or sign-extended immediate

    output reg  [`RISCV_XLEN-1:0]   result_o       // ALU result
);

//-----------------------------------------------------------------------------
// Combinational operation select
//-----------------------------------------------------------------------------
always @(*)
begin
    case (alu_op_i)
        `ALU_ADD : result_o = operand_a_i + operand_b_i;
        `ALU_SUB : result_o = operand_a_i - operand_b_i;

        // Shift amount is the low 5 bits of operand_b (RV32: shamt[4:0])
        `ALU_SLL : result_o = operand_a_i << operand_b_i[4:0];
        `ALU_SRL : result_o = operand_a_i >> operand_b_i[4:0];
        `ALU_SRA : result_o = $signed(operand_a_i) >>> operand_b_i[4:0];

        // Set-less-than: signed and unsigned compare, result is 0 or 1
        `ALU_SLT  : result_o = ($signed(operand_a_i) < $signed(operand_b_i)) ?
                                 {{(`RISCV_XLEN-1){1'b0}}, 1'b1} :
                                 {`RISCV_XLEN{1'b0}};
        `ALU_SLTU : result_o = (operand_a_i < operand_b_i) ?
                                 {{(`RISCV_XLEN-1){1'b0}}, 1'b1} :
                                 {`RISCV_XLEN{1'b0}};

        `ALU_XOR : result_o = operand_a_i ^ operand_b_i;
        `ALU_OR  : result_o = operand_a_i | operand_b_i;
        `ALU_AND : result_o = operand_a_i & operand_b_i;

        default  : result_o = {`RISCV_XLEN{1'b0}};
    endcase
end

endmodule