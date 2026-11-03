

//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_core
#(
    parameter ADDR_WIDTH = 14   // word-address bits; must match riscv_tcm_ram.v
)
(
    input  wire                   clk_i,
    input  wire                   rst_i,
    input  wire [`RISCV_XLEN-1:0] boot_addr_i,
    input  wire                   intr_i,        // external interrupt (placeholder - no CSR/trap unit yet)

    // ---------------- Instruction memory (TCM RAM Port A) ----------------
    output wire [ADDR_WIDTH-1:0]  imem_addr_o,
    input  wire [31:0]            imem_rdata_i,

    // ---------------- Data memory (TCM RAM Port B) ----------------
    output wire [ADDR_WIDTH-1:0]  dmem_addr_o,
    output wire [31:0]            dmem_wdata_o,
    output wire [3:0]             dmem_wstrb_o,
    input  wire [31:0]            dmem_rdata_i,

    // ---------------- Data memory - external-bus decode/stall ----------------
    output wire [`RISCV_XLEN-1:0] dmem_addr_full_o, // untruncated byte address, for range decode
    output wire                   dmem_valid_o,      // a load or store is sitting in MEM this cycle
    output wire                   dmem_we_o,         // qualifies dmem_valid_o: 1 = store, 0 = load
    input  wire                   dmem_stall_i       // top: external access outstanding, freeze the pipe
);

    //=====================================================================
    // Hazard / pipe-control signals (driven by riscv_hazard.v /
    // riscv_pipe_ctrl.v below, once the wires they depend on exist)
    //=====================================================================
    wire        load_use_stall;
    wire [1:0]  forward_a_sel;
    wire [1:0]  forward_b_sel;
    wire        pc_stall;
    wire        idex_bubble;
    wire        fetch_flush;
    wire        mem_stall = dmem_stall_i;   // external (AXI-Lite) access pending - freeze the whole pipe
reg [`RISCV_XLEN-1:0]        exmem_alu_result_q;
    //=====================================================================
    // IF stage
    //=====================================================================
    wire                    fetch_redirect;
    wire [`RISCV_XLEN-1:0]  fetch_redirect_pc;
    wire [`RISCV_XLEN-1:0]  fetch_pc;
    wire [31:0]             fetch_instr;

    riscv_fetch #(.ADDR_WIDTH(ADDR_WIDTH)) u_fetch
    (
        .clk_i          (clk_i),
        .rst_i          (rst_i),
        .boot_addr_i    (boot_addr_i),
        .stall_i        (pc_stall),
        .flush_i        (fetch_flush),
        .redirect_i     (fetch_redirect),
        .redirect_pc_i  (fetch_redirect_pc),
        .imem_addr_o    (imem_addr_o),
        .imem_rdata_i   (imem_rdata_i),
        .pc_o           (fetch_pc),
        .instr_o        (fetch_instr)
    );

   
    wire [`RISCV_REG_ADDR_W-1:0] id_rs1_addr = fetch_instr[19:15];
    wire [`RISCV_REG_ADDR_W-1:0] id_rs2_addr = fetch_instr[24:20];

    //=====================================================================
    // ID stage (combinational decoder + regfile + immediate gen)
    //=====================================================================
    wire [`RISCV_REG_ADDR_W-1:0] dec_rd_addr;
    wire [`RISCV_XLEN-1:0]       dec_rs1_data, dec_rs2_data, dec_imm;
    wire [`ALU_OP_W-1:0]         dec_alu_op;
    wire [`ALU_SRC_A_W-1:0]      dec_alu_src_a_sel;
    wire                         dec_alu_src_b_imm;
    wire                         dec_mem_read, dec_mem_write;
    wire [2:0]                   dec_funct3;
    wire                         dec_branch, dec_jump, dec_jalr;
    wire                         dec_reg_write;
    wire [`WB_SEL_W-1:0]         dec_wb_sel;
    wire                         dec_muldiv_en;
    wire [`MULDIV_OP_W-1:0]      dec_muldiv_op;
    wire                         dec_csr_en, dec_system, dec_illegal;
    wire [`RISCV_XLEN-1:0]       dec_pc;

    // Register-file write port, driven by the WB stage at the bottom
    wire                          wb_reg_write_en;
    wire [`RISCV_REG_ADDR_W-1:0]  wb_rd_addr;
    wire [`RISCV_XLEN-1:0]        wb_rd_data;

    riscv_decode u_decode
    (
        .clk_i        (clk_i),
        .rst_i        (rst_i),
        .instr_i      (fetch_instr),
        .pc_i         (fetch_pc),

        .wb_rd_addr_i (wb_rd_addr),
        .wb_rd_data_i (wb_rd_data),
        .wb_rd_wren_i (wb_reg_write_en),

        .pc_o             (dec_pc),
        .rd_addr_o        (dec_rd_addr),
        .rs1_data_o       (dec_rs1_data),
        .rs2_data_o       (dec_rs2_data),
        .imm_o            (dec_imm),
        .alu_op_o         (dec_alu_op),
        .alu_src_a_sel_o  (dec_alu_src_a_sel),
        .alu_src_b_imm_o  (dec_alu_src_b_imm),
        .mem_read_o       (dec_mem_read),
        .mem_write_o      (dec_mem_write),
        .funct3_o         (dec_funct3),
        .branch_o         (dec_branch),
        .jump_o           (dec_jump),
        .jalr_o           (dec_jalr),
        .reg_write_o      (dec_reg_write),
        .wb_sel_o         (dec_wb_sel),
        .muldiv_en_o      (dec_muldiv_en),
        .muldiv_op_o      (dec_muldiv_op),
        .csr_en_o         (dec_csr_en),
        .system_o         (dec_system),
        .illegal_o        (dec_illegal)
    );

    //=====================================================================
    // ID/EX latch
    //=====================================================================
    reg [`RISCV_XLEN-1:0]        idex_pc_q;
    reg [`RISCV_REG_ADDR_W-1:0]  idex_rd_addr_q;
    reg [`RISCV_XLEN-1:0]        idex_rs1_data_q, idex_rs2_data_q, idex_imm_q;
    reg [`ALU_OP_W-1:0]          idex_alu_op_q;
    reg [`ALU_SRC_A_W-1:0]       idex_alu_src_a_sel_q;
    reg                          idex_alu_src_b_imm_q;
    reg                          idex_mem_read_q, idex_mem_write_q;
    reg [2:0]                    idex_funct3_q;
    reg                          idex_branch_q, idex_jump_q, idex_jalr_q;
    reg                          idex_reg_write_q;
    reg [`WB_SEL_W-1:0]          idex_wb_sel_q;
    reg [`RISCV_REG_ADDR_W-1:0]  idex_rs1_addr_q, idex_rs2_addr_q;

    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            idex_pc_q             <= {`RISCV_XLEN{1'b0}};
            idex_rd_addr_q         <= {`RISCV_REG_ADDR_W{1'b0}};
            idex_rs1_data_q        <= {`RISCV_XLEN{1'b0}};
            idex_rs2_data_q        <= {`RISCV_XLEN{1'b0}};
            idex_imm_q             <= {`RISCV_XLEN{1'b0}};
            idex_alu_op_q          <= {`ALU_OP_W{1'b0}};
            idex_alu_src_a_sel_q   <= {`ALU_SRC_A_W{1'b0}};
            idex_alu_src_b_imm_q   <= 1'b0;
            idex_mem_read_q        <= 1'b0;
            idex_mem_write_q       <= 1'b0;
            idex_funct3_q          <= 3'b0;
            idex_branch_q          <= 1'b0;
            idex_jump_q            <= 1'b0;
            idex_jalr_q            <= 1'b0;
            idex_reg_write_q       <= 1'b0;
            idex_wb_sel_q          <= {`WB_SEL_W{1'b0}};
            idex_rs1_addr_q        <= {`RISCV_REG_ADDR_W{1'b0}};
            idex_rs2_addr_q        <= {`RISCV_REG_ADDR_W{1'b0}};
        end
     
        
        else if (idex_bubble)
        begin
            // Synchronous bubble: zero every control field so this slot
            // behaves as a NOP (no register write, no mem access, no
            // branch/jump) while still safely advancing every cycle.
            idex_pc_q             <= {`RISCV_XLEN{1'b0}};
            idex_rd_addr_q         <= {`RISCV_REG_ADDR_W{1'b0}};
            idex_rs1_data_q        <= {`RISCV_XLEN{1'b0}};
            idex_rs2_data_q        <= {`RISCV_XLEN{1'b0}};
            idex_imm_q             <= {`RISCV_XLEN{1'b0}};
            idex_alu_op_q          <= {`ALU_OP_W{1'b0}};
            idex_alu_src_a_sel_q   <= {`ALU_SRC_A_W{1'b0}};
            idex_alu_src_b_imm_q   <= 1'b0;
            idex_mem_read_q        <= 1'b0;
            idex_mem_write_q       <= 1'b0;
            idex_funct3_q          <= 3'b0;
            idex_branch_q          <= 1'b0;
            idex_jump_q            <= 1'b0;
            idex_jalr_q            <= 1'b0;
            idex_reg_write_q       <= 1'b0;
            idex_wb_sel_q          <= {`WB_SEL_W{1'b0}};
            idex_rs1_addr_q        <= {`RISCV_REG_ADDR_W{1'b0}};
            idex_rs2_addr_q        <= {`RISCV_REG_ADDR_W{1'b0}};
        end
        else
        begin
            idex_pc_q             <= dec_pc;
            idex_rd_addr_q         <= dec_rd_addr;
            idex_rs1_data_q        <= dec_rs1_data;
            idex_rs2_data_q        <= dec_rs2_data;
            idex_imm_q             <= dec_imm;
            idex_alu_op_q          <= dec_alu_op;
            idex_alu_src_a_sel_q   <= dec_alu_src_a_sel;
            idex_alu_src_b_imm_q   <= dec_alu_src_b_imm;
            idex_mem_read_q        <= dec_mem_read;
            idex_mem_write_q       <= dec_mem_write;
            idex_funct3_q          <= dec_funct3;
            idex_branch_q          <= dec_branch;
            idex_jump_q            <= dec_jump;
            idex_jalr_q            <= dec_jalr;
            idex_reg_write_q       <= dec_reg_write;
            idex_wb_sel_q          <= dec_wb_sel;
            idex_rs1_addr_q        <= id_rs1_addr;
            idex_rs2_addr_q        <= id_rs2_addr;
        end
    end

    //=====================================================================
    // EX stage - forwarding muxes feed the ALU operands, then the same
    // combinational exec block from the multicycle version resolves the
    // ALU result and branch/jump redirect.
    //=====================================================================
    wire [`RISCV_XLEN-1:0] forward_a_data =
        (forward_a_sel == 2'b01) ? exmem_alu_result_q :
        (forward_a_sel == 2'b10) ? wb_rd_data :
        idex_rs1_data_q;

    wire [`RISCV_XLEN-1:0] forward_b_data =
        (forward_b_sel == 2'b01) ? exmem_alu_result_q :
        (forward_b_sel == 2'b10) ? wb_rd_data :
        idex_rs2_data_q;

    wire [`RISCV_XLEN-1:0] exec_alu_result;
    wire [`RISCV_XLEN-1:0] exec_pc_plus4;
    wire                    exec_redirect;
    wire [`RISCV_XLEN-1:0] exec_redirect_pc;

    riscv_exec u_exec
    (
        .pc_i             (idex_pc_q),
        .rs1_data_i       (forward_a_data),
        .rs2_data_i       (forward_b_data),
        .imm_i            (idex_imm_q),
        .alu_op_i         (idex_alu_op_q),
        .alu_src_a_sel_i  (idex_alu_src_a_sel_q),
        .alu_src_b_imm_i  (idex_alu_src_b_imm_q),
        .funct3_i         (idex_funct3_q),
        .branch_i         (idex_branch_q),
        .jump_i           (idex_jump_q),
        .jalr_i           (idex_jalr_q),
        .alu_result_o     (exec_alu_result),
        .pc_plus4_o       (exec_pc_plus4),
        .redirect_o       (exec_redirect),
        .redirect_pc_o    (exec_redirect_pc)
    );


    assign fetch_redirect    = exec_redirect & ~mem_stall;
    assign fetch_redirect_pc = exec_redirect_pc;


    
    reg [`RISCV_XLEN-1:0]        exmem_pc_plus4_q;
    reg [`RISCV_REG_ADDR_W-1:0]  exmem_rd_addr_q;
    reg [`WB_SEL_W-1:0]          exmem_wb_sel_q;
    reg                          exmem_reg_write_q;
    reg                          exmem_mem_read_q;
    reg                          exmem_mem_write_q;
    reg [2:0]                    exmem_funct3_q;
    reg [`RISCV_XLEN-1:0]        exmem_store_data_q;

    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            exmem_alu_result_q <= {`RISCV_XLEN{1'b0}};
            exmem_pc_plus4_q   <= {`RISCV_XLEN{1'b0}};
            exmem_rd_addr_q    <= {`RISCV_REG_ADDR_W{1'b0}};
            exmem_wb_sel_q     <= {`WB_SEL_W{1'b0}};
            exmem_reg_write_q  <= 1'b0;
            exmem_mem_read_q   <= 1'b0;
            exmem_mem_write_q  <= 1'b0;
            exmem_funct3_q     <= 3'b0;
            exmem_store_data_q <= {`RISCV_XLEN{1'b0}};
        end
      
        else
        begin
            exmem_alu_result_q <= exec_alu_result;
            exmem_pc_plus4_q   <= exec_pc_plus4;
            exmem_rd_addr_q    <= idex_rd_addr_q;
            exmem_wb_sel_q     <= idex_wb_sel_q;
            exmem_reg_write_q  <= idex_reg_write_q;
            exmem_mem_read_q   <= idex_mem_read_q;
            exmem_mem_write_q  <= idex_mem_write_q;
            exmem_funct3_q     <= idex_funct3_q;
            exmem_store_data_q <= forward_b_data;   // forwarded, not raw idex_rs2_data_q
        end
    end


    wire [`RISCV_XLEN-1:0] lsu_mem_addr;

    riscv_lsu u_lsu_mem
    (
        .addr_i        (exmem_alu_result_q),
        .store_data_i  (exmem_store_data_q),
        .funct3_i      (exmem_funct3_q),
        .mem_read_i    (1'b0),                  // load-data extraction happens in u_lsu_wb instead
        .mem_write_i   (exmem_mem_write_q),
        .mem_addr_o    (lsu_mem_addr),
        .mem_wdata_o   (dmem_wdata_o),
        .mem_wstrb_o   (dmem_wstrb_o),
        .mem_write_o   (),
        .mem_read_o    (),
        .mem_rdata_i   (dmem_rdata_i),          // unused here (mem_read_i tied low)
        .load_data_o   (),
        .misaligned_o  ()                       // store misalignment; wire up once riscv_csr.v exists
    );

    assign dmem_addr_o = lsu_mem_addr[ADDR_WIDTH+1:2];

 
    assign dmem_addr_full_o = lsu_mem_addr;
    assign dmem_valid_o     = exmem_mem_read_q | exmem_mem_write_q;
    assign dmem_we_o        = exmem_mem_write_q;

    //=====================================================================
    // MEM/WB latch. Also held on mem_stall: WB must not consume
    // dmem_rdata_i until riscv_tcm_top.v actually has valid data/response
    // for an external access - see riscv_tcm_top.v's mux, which only
    // presents that data on the one cycle mem_stall is released.
    //=====================================================================
    reg [`RISCV_XLEN-1:0]        memwb_alu_result_q;
    reg [`RISCV_XLEN-1:0]        memwb_pc_plus4_q;
    reg [`RISCV_REG_ADDR_W-1:0]  memwb_rd_addr_q;
    reg [`WB_SEL_W-1:0]          memwb_wb_sel_q;
    reg                          memwb_reg_write_q;
    reg                          memwb_mem_read_q;
    reg [2:0]                    memwb_funct3_q;

    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            memwb_alu_result_q <= {`RISCV_XLEN{1'b0}};
            memwb_pc_plus4_q   <= {`RISCV_XLEN{1'b0}};
            memwb_rd_addr_q    <= {`RISCV_REG_ADDR_W{1'b0}};
            memwb_wb_sel_q     <= {`WB_SEL_W{1'b0}};
            memwb_reg_write_q  <= 1'b0;
            memwb_mem_read_q   <= 1'b0;
            memwb_funct3_q     <= 3'b0;
        end
      
        else
        begin
            memwb_alu_result_q <= exmem_alu_result_q;
            memwb_pc_plus4_q   <= exmem_pc_plus4_q;
            memwb_rd_addr_q    <= exmem_rd_addr_q;
            memwb_wb_sel_q     <= exmem_wb_sel_q;
            memwb_reg_write_q  <= exmem_reg_write_q;
            memwb_mem_read_q   <= exmem_mem_read_q;
            memwb_funct3_q     <= exmem_funct3_q;
        end
    end

    //=====================================================================
    // WB stage - load-data extraction (now that dmem_rdata_i has landed,
    // see file header comment) followed by the result mux.
    //=====================================================================
    wire [`RISCV_XLEN-1:0] wb_load_data;

    riscv_lsu u_lsu_wb
    (
        .addr_i        (memwb_alu_result_q),
        .store_data_i  ({`RISCV_XLEN{1'b0}}),   // unused, no store formatting needed here
        .funct3_i      (memwb_funct3_q),
        .mem_read_i    (memwb_mem_read_q),
        .mem_write_i   (1'b0),
        .mem_addr_o    (),
        .mem_wdata_o   (),
        .mem_wstrb_o   (),
        .mem_write_o   (),
        .mem_read_o    (),
        .mem_rdata_i   (dmem_rdata_i),
        .load_data_o   (wb_load_data),
        .misaligned_o  ()                       // load misalignment; wire up once riscv_csr.v exists
    );

    riscv_writeback u_writeback
    (
        .alu_result_i   (memwb_alu_result_q),
        .mem_data_i     (wb_load_data),
        .csr_data_i     ({`RISCV_XLEN{1'b0}}),    // placeholder until riscv_csr.v exists
        .muldiv_data_i  ({`RISCV_XLEN{1'b0}}),    // placeholder until riscv_mul.v/riscv_divider.v exist
        .pc_plus4_i     (memwb_pc_plus4_q),
        .wb_sel_i       (memwb_wb_sel_q),
        .reg_write_i    (memwb_reg_write_q),
        .rd_addr_i      (memwb_rd_addr_q),
        .reg_write_o    (wb_reg_write_en),
        .rd_addr_o      (wb_rd_addr),
        .rd_data_o      (wb_rd_data)
    );

    //=====================================================================
    // Hazard detection + forwarding select
    //=====================================================================
    riscv_hazard u_hazard
    (
        .id_rs1_addr_i     (id_rs1_addr),
        .id_rs2_addr_i     (id_rs2_addr),
        .idex_rs1_addr_i   (idex_rs1_addr_q),
        .idex_rs2_addr_i   (idex_rs2_addr_q),
        .idex_rd_addr_i    (idex_rd_addr_q),
        .idex_reg_write_i  (idex_reg_write_q),
        .idex_mem_read_i   (idex_mem_read_q),
        .exmem_rd_addr_i   (exmem_rd_addr_q),
        .exmem_reg_write_i (exmem_reg_write_q),
        .memwb_rd_addr_i   (wb_rd_addr),
        .memwb_reg_write_i (wb_reg_write_en),
        .load_use_stall_o  (load_use_stall),
        .forward_a_sel_o   (forward_a_sel),
        .forward_b_sel_o   (forward_b_sel)
    );

    //=====================================================================
    // Stall / bubble / flush arbitration
    //=====================================================================
    riscv_pipe_ctrl u_pipe_ctrl
    (
        .clk_i             (clk_i),
        .rst_i             (rst_i),
        .load_use_stall_i  (load_use_stall),
        .branch_redirect_i (exec_redirect),
        .mem_stall_i       (mem_stall),
        .pc_stall_o        (pc_stall),
        .idex_bubble_o     (idex_bubble),
        .fetch_flush_o     (fetch_flush)
    );

endmodule