//-----------------------------------------------------------------------------
// riscv_regfile.v
//
// 32 x 32-bit general-purpose register file, 2 combinational read ports and
// 1 synchronous write port. x0 is hard-wired to zero (reads always return 0,
// writes to x0 are silently dropped).
//
// A same-cycle write-to-read bypass is included so that a WB-stage write and
// an ID-stage read of the same register in the same clock cycle see the new
// value immediately. This is the standard technique used to resolve the
// WB -> ID data hazard without needing an extra forwarding mux elsewhere in
// the pipeline (see TRM Section 4.1, "Data hazards").
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_regfile
(
    input  wire                          clk_i,
    input  wire                          rst_i,     // sync reset; clears regs for
                                                      // deterministic sim/bring-up.
                                                      // Not architecturally required
                                                      // (may be dropped for ASIC to
                                                      // save reset fan-out).

    // Read port 1
    input  wire [`RISCV_REG_ADDR_W-1:0]  rs1_addr_i,
    output wire [`RISCV_XLEN-1:0]        rs1_data_o,

    // Read port 2
    input  wire [`RISCV_REG_ADDR_W-1:0]  rs2_addr_i,
    output wire [`RISCV_XLEN-1:0]        rs2_data_o,

    // Write port (WB stage)
    input  wire [`RISCV_REG_ADDR_W-1:0]  rd_addr_i,
    input  wire [`RISCV_XLEN-1:0]        rd_data_i,
    input  wire                          rd_wren_i
);

    // x0 is register index 0
    localparam [`RISCV_REG_ADDR_W-1:0] REG_X0 = {`RISCV_REG_ADDR_W{1'b0}};

    reg [`RISCV_XLEN-1:0] regs [0:`RISCV_NUM_REGS-1];

    integer i;

    //-------------------------------------------------------------------
    // Synchronous write port
    // - x0 is never actually written (kept at 0 at all times)
    //-------------------------------------------------------------------
    always @(posedge clk_i)
    begin
        if (rst_i)
        begin
            for (i = 0; i < `RISCV_NUM_REGS; i = i + 1)
                regs[i] <= {`RISCV_XLEN{1'b0}};
        end
        else if (rd_wren_i && (rd_addr_i != REG_X0))
        begin
            regs[rd_addr_i] <= rd_data_i;
        end
    end

    //-------------------------------------------------------------------
    // Combinational read ports with:
    //   1) x0 hard-wired to zero
    //   2) same-cycle write-to-read bypass (WB -> ID hazard resolution)
    //-------------------------------------------------------------------
    assign rs1_data_o = (rs1_addr_i == REG_X0)                    ? {`RISCV_XLEN{1'b0}} :
                         (rd_wren_i && (rd_addr_i == rs1_addr_i)) ? rd_data_i            :
                                                                     regs[rs1_addr_i];

    assign rs2_data_o = (rs2_addr_i == REG_X0)                    ? {`RISCV_XLEN{1'b0}} :
                         (rd_wren_i && (rd_addr_i == rs2_addr_i)) ? rd_data_i            :
                                                                     regs[rs2_addr_i];

endmodule