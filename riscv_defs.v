//-----------------------------------------------------------------------------
// riscv_defs.v
//
// Global parameter, opcode, funct-code, and CSR-address definitions shared
// across all core modules (fetch, decode, execute, LSU, CSR, hazard unit).
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable
//-----------------------------------------------------------------------------
`ifndef RISCV_DEFS_V
`define RISCV_DEFS_V

//-----------------------------------------------------------------------------
// Core-wide parameters
//-----------------------------------------------------------------------------
`define RISCV_XLEN              32          // Register / data width
`define RISCV_REG_ADDR_W        5           // 32 architectural registers
`define RISCV_NUM_REGS          32

// Default configuration values (overridable via top-level parameters)
`define RISCV_BOOT_ADDRESS_DEFAULT  32'h0000_2000
`define RISCV_TCM_SIZE_DEFAULT      (64*1024)   // 64KB default TCM

//-----------------------------------------------------------------------------
// Instruction field bit positions (common to all RV32 instruction formats)
//-----------------------------------------------------------------------------
`define INSTR_OPCODE_R    6:0
`define INSTR_RD_R        11:7
`define INSTR_FUNCT3_R    14:12
`define INSTR_RS1_R       19:15
`define INSTR_RS2_R       24:20
`define INSTR_FUNCT7_R    31:25

//-----------------------------------------------------------------------------
// RV32I base opcode map (bits [6:0], with mandatory [1:0] = 2'b11)
//-----------------------------------------------------------------------------
`define OPCODE_LOAD       7'b0000011   // I-type : LB/LH/LW/LBU/LHU
`define OPCODE_MISC_MEM   7'b0001111   // FENCE/FENCE.I
`define OPCODE_OP_IMM     7'b0010011   // I-type : ADDI/SLTI/.../SRAI
`define OPCODE_AUIPC      7'b0010111   // U-type : AUIPC
`define OPCODE_STORE      7'b0100011   // S-type : SB/SH/SW
`define OPCODE_OP         7'b0110011   // R-type : ADD/SUB/.../AND (+ RV32M)
`define OPCODE_LUI        7'b0110111   // U-type : LUI
`define OPCODE_BRANCH     7'b1100011   // B-type : BEQ/BNE/.../BGEU
`define OPCODE_JALR       7'b1100111   // I-type : JALR
`define OPCODE_JAL        7'b1101111   // J-type : JAL
`define OPCODE_SYSTEM     7'b1110011   // I-type : ECALL/EBREAK/CSRxx

//-----------------------------------------------------------------------------
// funct3 encodings - OPCODE_OP_IMM / OPCODE_OP (integer ALU ops)
//-----------------------------------------------------------------------------
`define F3_ADD_SUB   3'b000    // ADD/ADDI, SUB (funct7 distinguishes)
`define F3_SLL       3'b001    // SLL/SLLI
`define F3_SLT       3'b010    // SLT/SLTI
`define F3_SLTU      3'b011    // SLTU/SLTIU
`define F3_XOR       3'b100    // XOR/XORI
`define F3_SRL_SRA   3'b101    // SRL/SRLI, SRA/SRAI (funct7 distinguishes)
`define F3_OR        3'b110    // OR/ORI
`define F3_AND       3'b111    // AND/ANDI

//-----------------------------------------------------------------------------
// funct7 encodings - OPCODE_OP (distinguishes ADD/SUB and SRL/SRA)
//-----------------------------------------------------------------------------
`define F7_ADD_SRL   7'b0000000    // ADD, SRL
`define F7_SUB_SRA   7'b0100000    // SUB, SRA
`define F7_MULDIV    7'b0000001    // RV32M extension (MUL/DIV/REM family)

//-----------------------------------------------------------------------------
// funct3 encodings - OPCODE_OP with funct7 = F7_MULDIV (RV32M)
//-----------------------------------------------------------------------------
`define F3_MUL       3'b000
`define F3_MULH      3'b001
`define F3_MULHSU    3'b010
`define F3_MULHU     3'b011
`define F3_DIV       3'b100
`define F3_DIVU      3'b101
`define F3_REM       3'b110
`define F3_REMU      3'b111

//-----------------------------------------------------------------------------
// funct3 encodings - OPCODE_LOAD / OPCODE_STORE
//-----------------------------------------------------------------------------
`define F3_LB        3'b000
`define F3_LH        3'b001
`define F3_LW        3'b010
`define F3_LBU       3'b100
`define F3_LHU       3'b101

`define F3_SB        3'b000
`define F3_SH        3'b001
`define F3_SW        3'b010

//-----------------------------------------------------------------------------
// funct3 encodings - OPCODE_BRANCH
//-----------------------------------------------------------------------------
`define F3_BEQ        3'b000
`define F3_BNE        3'b001
`define F3_BLT        3'b100
`define F3_BGE        3'b101
`define F3_BLTU       3'b110
`define F3_BGEU       3'b111

//-----------------------------------------------------------------------------
// funct3 encodings - OPCODE_SYSTEM (CSR + ECALL/EBREAK)
//-----------------------------------------------------------------------------
`define F3_PRIV       3'b000   // ECALL/EBREAK/MRET (funct12/imm distinguishes)
`define F3_CSRRW      3'b001
`define F3_CSRRS      3'b010
`define F3_CSRRC      3'b011
`define F3_CSRRWI     3'b101
`define F3_CSRRSI     3'b110
`define F3_CSRRCI     3'b111

// imm[11:0] values for OPCODE_SYSTEM / F3_PRIV
`define IMM12_ECALL   12'h000
`define IMM12_EBREAK  12'h001
`define IMM12_MRET    12'h302

//-----------------------------------------------------------------------------
// Internal ALU operation encoding (decoder -> EX stage control)
//-----------------------------------------------------------------------------
`define ALU_OP_W      4
`define ALU_ADD       4'd0
`define ALU_SUB       4'd1
`define ALU_SLL       4'd2
`define ALU_SLT       4'd3
`define ALU_SLTU      4'd4
`define ALU_XOR       4'd5
`define ALU_SRL       4'd6
`define ALU_SRA       4'd7
`define ALU_OR        4'd8
`define ALU_AND       4'd9

//-----------------------------------------------------------------------------
// Internal multiply/divide operation encoding (decoder -> MUL/DIV unit)
//-----------------------------------------------------------------------------
`define MULDIV_OP_W   3
`define MULDIV_MUL    3'd0
`define MULDIV_MULH   3'd1
`define MULDIV_MULHSU 3'd2
`define MULDIV_MULHU  3'd3
`define MULDIV_DIV    3'd4
`define MULDIV_DIVU   3'd5
`define MULDIV_REM    3'd6
`define MULDIV_REMU   3'd7

//-----------------------------------------------------------------------------
// Machine-mode CSR addresses (subset implemented per TRM Section 10)
//-----------------------------------------------------------------------------
`define CSR_ADDR_W     12
`define CSR_MSTATUS   12'h300
`define CSR_MIE       12'h304
`define CSR_MTVEC     12'h305
`define CSR_MSCRATCH  12'h340
`define CSR_MEPC      12'h341
`define CSR_MCAUSE    12'h342
`define CSR_MTVAL     12'h343
`define CSR_MIP       12'h344
`define CSR_MHARTID   12'hF14

//-----------------------------------------------------------------------------
// Trap cause codes (mcause) - interrupt bit is mcause[31]
//-----------------------------------------------------------------------------
`define CAUSE_INTERRUPT_BIT        31

// Exceptions (mcause[31] = 0)
`define CAUSE_INSTR_ADDR_MISALIGNED  32'h0000_0000
`define CAUSE_ILLEGAL_INSTRUCTION    32'h0000_0002
`define CAUSE_BREAKPOINT             32'h0000_0003
`define CAUSE_LOAD_ADDR_MISALIGNED   32'h0000_0004
`define CAUSE_STORE_ADDR_MISALIGNED  32'h0000_0006
`define CAUSE_ECALL_M                32'h0000_000B

// Interrupts (mcause[31] = 1, low bits shown here)
`define CAUSE_MACHINE_TIMER_INT      32'h8000_0007
`define CAUSE_MACHINE_EXTERNAL_INT   32'h8000_000B

//-----------------------------------------------------------------------------
// Pipeline stage result-source select (WB stage mux control)
//-----------------------------------------------------------------------------
`define WB_SEL_W      3
`define WB_SEL_ALU    3'd0 //write back to register value comes from alu
`define WB_SEL_MEM    3'd1 //value comes from memory
`define WB_SEL_CSR    3'd2 //value comes from csr operation
`define WB_SEL_MULDIV 3'd3 //result comes from mult div unit
`define WB_SEL_PC4    3'd4     //wrote [c+4 omtp destination register

//-----------------------------------------------------------------------------
// Immediate format select (decoder -> ID-stage immediate generator)
//-----------------------------------------------------------------------------
`define IMM_TYPE_W    3
`define IMM_I         3'd0     // OP-IMM, LOAD, JALR
`define IMM_S         3'd1     // STORE
`define IMM_B         3'd2     // BRANCH
`define IMM_U         3'd3     // LUI, AUIPC
`define IMM_J         3'd4     // JAL
`define IMM_NONE      3'd5     // R-type / SYSTEM (no standard immediate)

//-----------------------------------------------------------------------------
// ALU operand-A source select (decoder -> EX stage operand mux)
//-----------------------------------------------------------------------------
`define ALU_SRC_A_W     2
`define ALU_SRC_A_RS1   2'd0   // normal register operand
`define ALU_SRC_A_PC    2'd1   // AUIPC / JAL target calc (PC + imm)
`define ALU_SRC_A_ZERO  2'd2   // LUI (result = 0 + imm)

`endif // RISCV_DEFS_V