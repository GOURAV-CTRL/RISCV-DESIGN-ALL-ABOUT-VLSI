
`include "riscv_defs.v"

module riscv_hazard
(
    // Raw rs1/rs2 fields of the instruction currently in ID (this cycle's
    // decode), extracted directly from the instruction word - used only
    // for load-use detection against the instruction currently in EX.
    input  wire [`RISCV_REG_ADDR_W-1:0]  id_rs1_addr_i,
    input  wire [`RISCV_REG_ADDR_W-1:0]  id_rs2_addr_i,

    // Instruction currently in EX (ID/EX latch contents)
    input  wire [`RISCV_REG_ADDR_W-1:0]  idex_rs1_addr_i,
    input  wire [`RISCV_REG_ADDR_W-1:0]  idex_rs2_addr_i,
    input  wire [`RISCV_REG_ADDR_W-1:0]  idex_rd_addr_i,
    input  wire                          idex_reg_write_i, //This tells us whether the EX instruction will eventually write a register.
    input  wire                          idex_mem_read_i,//This is the signal that is particularly important for the load-use hazard.
    //this signal tells wheather the instruction which is present in the execution stage is going to read or not

    // Instruction currently in MEM (EX/MEM latch contents)
    input  wire [`RISCV_REG_ADDR_W-1:0]  exmem_rd_addr_i,// the register address where the execuiton result is going to be store
    input  wire                          exmem_reg_write_i,// the status wheather it is a register write operation

    // Instruction currently in WB (MEM/WB latch contents / final result)
    input  wire [`RISCV_REG_ADDR_W-1:0]  memwb_rd_addr_i,
    input  wire                          memwb_reg_write_i,

    // ---------------- Outputs ----------------
    output wire                          load_use_stall_o,
    output wire [1:0]                    forward_a_sel_o,   // EX rs1 operand select
    output wire [1:0]                    forward_b_sel_o    // EX rs2 operand select
);

    localparam [1:0] FWD_NONE  = 2'b00; // use ID/EX-latched regfile value
    localparam [1:0] FWD_EXMEM = 2'b01; // forward from EX/MEM latch (alu_result)
    localparam [1:0] FWD_MEMWB = 2'b10; // forward from MEM/WB / writeback result

    //-------------------------------------------------------------------
    // Load-use hazard: the load in EX hasn't produced data yet (it lands
    // in WB, not MEM, in this design), so a dependent instruction right
    // behind it in ID must stall one cycle rather than be forwarded to.
    //-------------------------------------------------------------------
    assign load_use_stall_o = idex_mem_read_i && idex_reg_write_i &&
                               (idex_rd_addr_i != {`RISCV_REG_ADDR_W{1'b0}}) &&
                               ((idex_rd_addr_i == id_rs1_addr_i) ||
                                (idex_rd_addr_i == id_rs2_addr_i));


//instruction decode execute destination register address is equal to instruction decode source address
//then we need to definetly stall the pipeline
    //-------------------------------------------------------------------
    // EX-stage forwarding selects. EX/MEM (more recent) wins over MEM/WB.
    // Note: by construction, a load can never legitimately win the
    // EX/MEM path here - load_use_stall_o already caught and bubbled
    // that exact overlap one cycle earlier, before it could reach EX/MEM
    // at the same time a real dependent instruction is in EX.
    //-------------------------------------------------------------------
    assign forward_a_sel_o =
        (exmem_reg_write_i && (exmem_rd_addr_i != {`RISCV_REG_ADDR_W{1'b0}}) &&
         (exmem_rd_addr_i == idex_rs1_addr_i)) ? FWD_EXMEM :
        (memwb_reg_write_i && (memwb_rd_addr_i != {`RISCV_REG_ADDR_W{1'b0}}) &&
         (memwb_rd_addr_i == idex_rs1_addr_i)) ? FWD_MEMWB :
        FWD_NONE;

    assign forward_b_sel_o =
        (exmem_reg_write_i && (exmem_rd_addr_i != {`RISCV_REG_ADDR_W{1'b0}}) &&
         (exmem_rd_addr_i == idex_rs2_addr_i)) ? FWD_EXMEM :
        (memwb_reg_write_i && (memwb_rd_addr_i != {`RISCV_REG_ADDR_W{1'b0}}) &&
         (memwb_rd_addr_i == idex_rs2_addr_i)) ? FWD_MEMWB :
        FWD_NONE;

endmodule