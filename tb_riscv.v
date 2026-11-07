//-----------------------------------------------------------------------------
// tb_riscv_core.v
//
// Self-checking testbench for the 5-stage RV32I core (riscv_core) wired to
// the dual-port TCM RAM (riscv_tcm_ram). No AXI wrapper is needed: Port A of
// the RAM feeds instruction fetch, Port B is the LSU data port.
//
// Three directed tests:
//   TEST 1 - ALU ops + data-hazard forwarding (EX/MEM, MEM/WB, x0 rule)
//   TEST 2 - Load/store: byte/half/word, sign/zero extension, load-use stall
//   TEST 3 - Control flow: BEQ/BNE, JAL, JALR, AUIPC, wrong-path squash, loop
//

`timescale 1ns/1ps
`include "riscv_defs.v"

module tb_riscv_core2;

    localparam ADDR_WIDTH = 14;
    localparam [31:0] BOOT_ADDR = 32'h0000_2000;      // RISCV_BOOT_ADDRESS_DEFAULT
    localparam        BOOT_WORD = BOOT_ADDR >> 2;     // word index of first instr
    localparam [31:0] DATA_BASE = 32'h0000_1000;      // data region (word 0x400)
    localparam        DATA_WORD = DATA_BASE >> 2;

    //-------------------------------------------------------------------
    // Clock / reset
    //-------------------------------------------------------------------
    reg clk;
    reg rst;
    initial clk = 1'b0;
    always #5 clk = ~clk;                              // 100 MHz

    //-------------------------------------------------------------------
    // DUT <-> RAM wiring
    //-------------------------------------------------------------------
    wire [ADDR_WIDTH-1:0] imem_addr;
    wire [31:0]           imem_rdata;
    wire [ADDR_WIDTH-1:0] dmem_addr;
    wire [31:0]           dmem_wdata;
    wire [3:0]            dmem_wstrb;
    wire [31:0]           dmem_rdata;
    wire [31:0]           dmem_addr_full;
    wire                  dmem_valid, dmem_we;

    riscv_core #(.ADDR_WIDTH(ADDR_WIDTH)) dut
    (
        .clk_i            (clk),
        .rst_i            (rst),
        .boot_addr_i      (BOOT_ADDR),
        .intr_i           (1'b0),
        .imem_addr_o      (imem_addr),
        .imem_rdata_i     (imem_rdata),
        .dmem_addr_o      (dmem_addr),
        .dmem_wdata_o     (dmem_wdata),
        .dmem_wstrb_o     (dmem_wstrb),
        .dmem_rdata_i     (dmem_rdata),
        .dmem_addr_full_o (dmem_addr_full),
        .dmem_valid_o     (dmem_valid),
        .dmem_we_o        (dmem_we),
        .dmem_stall_i     (1'b0)                       // no external bus here
    );

    riscv_tcm_ram #(.ADDR_WIDTH(ADDR_WIDTH)) ram
    (
        .clk_i     (clk),
        .addr_a_i  (imem_addr),
        .wdata_a_i (32'd0),
        .wstrb_a_i (4'b0000),                          // Port A is read-only
        .rdata_a_o (imem_rdata),
        .addr_b_i  (dmem_addr),
        .wdata_b_i (dmem_wdata),
        .wstrb_b_i (dmem_wstrb),
        .rdata_b_o (dmem_rdata)
    );

    //-------------------------------------------------------------------
    // Tiny assembler: functions that build RV32I instruction words
    //-------------------------------------------------------------------
    function [31:0] r_type;
        input [6:0] f7; input [4:0] rs2, rs1; input [2:0] f3; input [4:0] rd;
        input [6:0] op;
        r_type = {f7, rs2, rs1, f3, rd, op};
    endfunction

    function [31:0] i_type;
        input [11:0] imm; input [4:0] rs1; input [2:0] f3; input [4:0] rd;
        input [6:0] op;
        i_type = {imm, rs1, f3, rd, op};
    endfunction

    function [31:0] s_type;
        input [11:0] imm; input [4:0] rs2, rs1; input [2:0] f3;
        s_type = {imm[11:5], rs2, rs1, f3, imm[4:0], `OPCODE_STORE};
    endfunction

    function [31:0] b_type;
        input [12:0] off; input [4:0] rs2, rs1; input [2:0] f3;
        b_type = {off[12], off[10:5], rs2, rs1, f3, off[4:1], off[11],
                  `OPCODE_BRANCH};
    endfunction

    function [31:0] u_type;
        input [19:0] imm; input [4:0] rd; input [6:0] op;
        u_type = {imm, rd, op};
    endfunction

    function [31:0] j_type;
        input [20:0] off; input [4:0] rd;
        j_type = {off[20], off[10:1], off[11], off[19:12], rd, `OPCODE_JAL};
    endfunction

    // Convenience wrappers
    function [31:0] ADDI; input [4:0] rd, rs1; input [11:0] imm;
        ADDI = i_type(imm, rs1, `F3_ADD_SUB, rd, `OPCODE_OP_IMM); endfunction
    function [31:0] SLLI; input [4:0] rd, rs1; input [4:0] sh;
        SLLI = i_type({7'b0000000, sh}, rs1, `F3_SLL, rd, `OPCODE_OP_IMM); endfunction
    function [31:0] SRLI; input [4:0] rd, rs1; input [4:0] sh;
        SRLI = i_type({7'b0000000, sh}, rs1, `F3_SRL_SRA, rd, `OPCODE_OP_IMM); endfunction
    function [31:0] SRAI; input [4:0] rd, rs1; input [4:0] sh;
        SRAI = i_type({7'b0100000, sh}, rs1, `F3_SRL_SRA, rd, `OPCODE_OP_IMM); endfunction
    function [31:0] ADD;  input [4:0] rd, rs1, rs2;
        ADD  = r_type(`F7_ADD_SRL, rs2, rs1, `F3_ADD_SUB, rd, `OPCODE_OP); endfunction
    function [31:0] SUB;  input [4:0] rd, rs1, rs2;
        SUB  = r_type(`F7_SUB_SRA, rs2, rs1, `F3_ADD_SUB, rd, `OPCODE_OP); endfunction
    function [31:0] AND_; input [4:0] rd, rs1, rs2;
        AND_ = r_type(`F7_ADD_SRL, rs2, rs1, `F3_AND, rd, `OPCODE_OP); endfunction
    function [31:0] OR_;  input [4:0] rd, rs1, rs2;
        OR_  = r_type(`F7_ADD_SRL, rs2, rs1, `F3_OR, rd, `OPCODE_OP); endfunction
    function [31:0] XOR_; input [4:0] rd, rs1, rs2;
        XOR_ = r_type(`F7_ADD_SRL, rs2, rs1, `F3_XOR, rd, `OPCODE_OP); endfunction
    function [31:0] SLT;  input [4:0] rd, rs1, rs2;
        SLT  = r_type(`F7_ADD_SRL, rs2, rs1, `F3_SLT, rd, `OPCODE_OP); endfunction
    function [31:0] LUI;  input [4:0] rd; input [19:0] imm;
        LUI  = u_type(imm, rd, `OPCODE_LUI); endfunction
    function [31:0] AUIPC; input [4:0] rd; input [19:0] imm;
        AUIPC = u_type(imm, rd, `OPCODE_AUIPC); endfunction
    function [31:0] LW;   input [4:0] rd, rs1; input [11:0] imm;
        LW   = i_type(imm, rs1, `F3_LW,  rd, `OPCODE_LOAD); endfunction
    function [31:0] LB;   input [4:0] rd, rs1; input [11:0] imm;
        LB   = i_type(imm, rs1, `F3_LB,  rd, `OPCODE_LOAD); endfunction
    function [31:0] LBU;  input [4:0] rd, rs1; input [11:0] imm;
        LBU  = i_type(imm, rs1, `F3_LBU, rd, `OPCODE_LOAD); endfunction
    function [31:0] LH;   input [4:0] rd, rs1; input [11:0] imm;
        LH   = i_type(imm, rs1, `F3_LH,  rd, `OPCODE_LOAD); endfunction
    function [31:0] LHU;  input [4:0] rd, rs1; input [11:0] imm;
        LHU  = i_type(imm, rs1, `F3_LHU, rd, `OPCODE_LOAD); endfunction
    function [31:0] SW;   input [4:0] rs2, rs1; input [11:0] imm;
        SW   = s_type(imm, rs2, rs1, `F3_SW); endfunction
    function [31:0] SH;   input [4:0] rs2, rs1; input [11:0] imm;
        SH   = s_type(imm, rs2, rs1, `F3_SH); endfunction
    function [31:0] SB;   input [4:0] rs2, rs1; input [11:0] imm;
        SB   = s_type(imm, rs2, rs1, `F3_SB); endfunction
    function [31:0] BEQ;  input [4:0] rs1, rs2; input [12:0] off;
        BEQ  = b_type(off, rs2, rs1, `F3_BEQ); endfunction
    function [31:0] BNE;  input [4:0] rs1, rs2; input [12:0] off;
        BNE  = b_type(off, rs2, rs1, `F3_BNE); endfunction
    function [31:0] JAL;  input [4:0] rd; input [20:0] off;
        JAL  = j_type(off, rd); endfunction
    function [31:0] JALR; input [4:0] rd, rs1; input [11:0] imm;
        JALR = i_type(imm, rs1, 3'b000, rd, `OPCODE_JALR); endfunction

    //-------------------------------------------------------------------
    // Program loader / test helpers
    //-------------------------------------------------------------------
    integer pw;                       // next free word while emitting
    integer k;
    integer errors;
    integer checks;

    task emit; input [31:0] instr;
        begin
            ram.mem[BOOT_WORD + pw] = instr;
            pw = pw + 1;
        end
    endtask

    task clear_ram;
        begin
            for (k = 0; k < (1 << ADDR_WIDTH); k = k + 1)
                ram.mem[k] = 32'h0000_0000;
            pw = 0;
        end
    endtask

    // Hold reset across several clock edges, then let the core run.
    task run_program; input integer cycles;
        begin
            rst = 1'b1;
            repeat (4) @(posedge clk);
            #1 rst = 1'b0;
            repeat (cycles) @(posedge clk);
            #1;
        end
    endtask

    task check_reg; input [4:0] idx; input [31:0] expected; input [255:0] name;
        reg [31:0] actual;
        begin
            actual = dut.u_decode.u_regfile.regs[idx];
            checks = checks + 1;
            if (actual !== expected) begin
                errors = errors + 1;
                $display("  [FAIL] x%0d (%0s): got 0x%08h, expected 0x%08h",
                         idx, name, actual, expected);
            end else
                $display("  [PASS] x%0d (%0s) = 0x%08h", idx, name, actual);
        end
    endtask

    task check_mem; input [31:0] byte_addr; input [31:0] expected; input [255:0] name;
        reg [31:0] actual;
        begin
            actual = ram.mem[byte_addr >> 2];
            checks = checks + 1;
            if (actual !== expected) begin
                errors = errors + 1;
                $display("  [FAIL] mem[0x%08h] (%0s): got 0x%08h, expected 0x%08h",
                         byte_addr, name, actual, expected);
            end else
                $display("  [PASS] mem[0x%08h] (%0s) = 0x%08h",
                         byte_addr, name, actual);
        end
    endtask

    //-------------------------------------------------------------------
    // Main sequence
    //-------------------------------------------------------------------
    initial begin
        errors = 0;
        checks = 0;
        rst    = 1'b1;
        #1;                                    // let RAM's initial block finish

        //===============================================================
        // TEST 1 : ALU operations and data-hazard forwarding
        //   Almost every instruction consumes the result of the one just
        //   before it, so the EX/MEM and MEM/WB forwarding paths (and the
        //   regfile write-through bypass) are all exercised.
        //===============================================================
        $display("\n=== TEST 1: ALU + data-hazard forwarding ===");
        clear_ram;
        emit(ADDI(5'd1, 5'd0, 12'd10));            // x1  = 10
        emit(ADDI(5'd2, 5'd0, 12'd20));            // x2  = 20
        emit(ADD (5'd3, 5'd1, 5'd2));              // x3  = 30  (x2: EX/MEM fwd, x1: MEM/WB fwd)
        emit(SUB (5'd4, 5'd3, 5'd1));              // x4  = 20  (x3: EX/MEM fwd)
        emit(AND_(5'd5, 5'd4, 5'd3));              // x5  = 20 & 30 = 20
        emit(OR_ (5'd6, 5'd5, 5'd1));              // x6  = 20 | 10 = 30
        emit(XOR_(5'd7, 5'd6, 5'd2));              // x7  = 30 ^ 20 = 10
        emit(SLLI(5'd8, 5'd7, 5'd4));              // x8  = 10 << 4 = 160
        emit(SRLI(5'd9, 5'd8, 5'd2));              // x9  = 160 >> 2 = 40
        emit(ADDI(5'd0, 5'd1, 12'd5));             // write to x0 -> must be ignored
        emit(ADD (5'd10, 5'd0, 5'd9));             // x10 = 0 + 40 = 40 (x0 still zero)
        emit(LUI (5'd11, 20'h12345));              // x11 = 0x12345000
        emit(ADDI(5'd11, 5'd11, 12'h678));         // x11 = 0x12345678
        emit(SLT (5'd12, 5'd1, 5'd2));             // x12 = (10 < 20) = 1
        emit(ADDI(5'd13, 5'd0, -12'sd64));         // x13 = -64
        emit(SRAI(5'd14, 5'd13, 5'd3));            // x14 = -64 >>> 3 = -8
        emit(JAL (5'd0, 21'd0));                   // halt: jump-to-self
        run_program(60);

        check_reg(5'd1,  32'd10,         "addi");
        check_reg(5'd2,  32'd20,         "addi");
        check_reg(5'd3,  32'd30,         "add, 2 forwards");
        check_reg(5'd4,  32'd20,         "sub, fwd from add");
        check_reg(5'd5,  32'd20,         "and");
        check_reg(5'd6,  32'd30,         "or");
        check_reg(5'd7,  32'd10,         "xor");
        check_reg(5'd8,  32'd160,        "slli");
        check_reg(5'd9,  32'd40,         "srli");
        check_reg(5'd0,  32'd0,          "x0 hard-wired zero");
        check_reg(5'd10, 32'd40,         "add using x0");
        check_reg(5'd11, 32'h12345678,   "lui + addi");
        check_reg(5'd12, 32'd1,          "slt");
        check_reg(5'd13, 32'hFFFFFFC0,   "addi negative");
        check_reg(5'd14, 32'hFFFFFFF8,   "srai sign-extend");

        //===============================================================
        // TEST 2 : Load / store + load-use hazard
        //   Word, half, byte stores with byte-lane strobes; signed and
        //   unsigned loads; dependent instruction directly after a load
        //   (needs a 1-cycle stall); store data that comes from a load.
        //===============================================================
        $display("\n=== TEST 2: Load/Store + load-use stall ===");
        clear_ram;
        emit(LUI (5'd1, 20'h00001));               // x1 = 0x1000 (data base)
        emit(LUI (5'd2, 20'hAABBD));               // x2 = 0xAABBD000
        emit(ADDI(5'd2, 5'd2, -12'sd803));         // x2 = 0xAABBCCDD
        emit(SW  (5'd2, 5'd1, 12'd0));             // mem[0x1000] = 0xAABBCCDD
        emit(LW  (5'd3, 5'd1, 12'd0));             // x3 = 0xAABBCCDD
        emit(ADD (5'd4, 5'd3, 5'd0));              // x4 = x3  (LOAD-USE: must stall)
        emit(LB  (5'd5, 5'd1, 12'd0));             // x5 = 0xFFFFFFDD  (sign-ext)
        emit(LBU (5'd6, 5'd1, 12'd1));             // x6 = 0x000000CC  (zero-ext)
        emit(LH  (5'd7, 5'd1, 12'd2));             // x7 = 0xFFFFAABB  (sign-ext)
        emit(LHU (5'd8, 5'd1, 12'd2));             // x8 = 0x0000AABB  (zero-ext)
        emit(ADDI(5'd9, 5'd0, 12'h055));           // x9 = 0x55
        emit(SB  (5'd9, 5'd1, 12'd4));             // mem[0x1004] byte0 = 0x55
        emit(SH  (5'd2, 5'd1, 12'd6));             // mem[0x1004] upper half = 0xCCDD
        emit(LW  (5'd10, 5'd1, 12'd4));            // x10 = 0xCCDD0055
        emit(ADDI(5'd11, 5'd10, 12'd1));           // x11 = 0xCCDD0056 (LOAD-USE again)
        emit(SW  (5'd11, 5'd1, 12'd8));            // mem[0x1008] = x11 (store data fwd)
        emit(LW  (5'd12, 5'd1, 12'd8));            // x12 = 0xCCDD0056
        emit(LW  (5'd13, 5'd1, 12'd0));            // x13 = 0xAABBCCDD
        emit(SW  (5'd13, 5'd1, 12'd12));           // store data straight from a load
        emit(LW  (5'd14, 5'd1, 12'd12));           // x14 = 0xAABBCCDD
        emit(JAL (5'd0, 21'd0));                   // halt
        run_program(70);

        check_reg(5'd1,  32'h00001000, "base addr");
        check_reg(5'd2,  32'hAABBCCDD, "li 0xAABBCCDD");
        check_reg(5'd3,  32'hAABBCCDD, "lw");
        check_reg(5'd4,  32'hAABBCCDD, "load-use stall result");
        check_reg(5'd5,  32'hFFFFFFDD, "lb sign-ext");
        check_reg(5'd6,  32'h000000CC, "lbu zero-ext");
        check_reg(5'd7,  32'hFFFFAABB, "lh sign-ext");
        check_reg(5'd8,  32'h0000AABB, "lhu zero-ext");
        check_reg(5'd10, 32'hCCDD0055, "sb + sh merged");
        check_reg(5'd11, 32'hCCDD0056, "load-use + addi");
        check_reg(5'd12, 32'hCCDD0056, "sw then lw");
        check_reg(5'd13, 32'hAABBCCDD, "lw");
        check_reg(5'd14, 32'hAABBCCDD, "lw->sw->lw");
        check_mem(DATA_BASE + 32'd0,  32'hAABBCCDD, "sw");
        check_mem(DATA_BASE + 32'd4,  32'hCCDD0055, "sb/sh strobes");
        check_mem(DATA_BASE + 32'd8,  32'hCCDD0056, "sw fwd data");
        check_mem(DATA_BASE + 32'd12, 32'hAABBCCDD, "sw of loaded data");

        //===============================================================
        // TEST 3 : Control flow
        //   Taken branch (wrong-path instrs must NOT write registers),
        //   not-taken branch, JAL link, a back-to-back-dependent loop
        //   branch (sum 1..5), AUIPC and JALR.
        //   Program byte offsets are noted so branch targets can be
        //   checked by eye.
        //===============================================================
        $display("\n=== TEST 3: Branches, JAL, JALR, loop ===");
        clear_ram;
        emit(ADDI(5'd1,  5'd0, 12'd5));            //  0: x1 = 5
        emit(ADDI(5'd2,  5'd0, 12'd5));            //  4: x2 = 5
        emit(BEQ (5'd1, 5'd2, 13'd12));            //  8: taken -> 20
        emit(ADDI(5'd3,  5'd0, 12'd111));          // 12: WRONG PATH (squash)
        emit(ADDI(5'd4,  5'd0, 12'd222));          // 16: WRONG PATH (squash)
        emit(ADDI(5'd5,  5'd0, 12'd1));            // 20: x5 = 1 (branch target)
        emit(BNE (5'd1, 5'd2, 13'd8));             // 24: not taken
        emit(ADDI(5'd6,  5'd0, 12'd2));            // 28: x6 = 2 (must execute)
        emit(JAL (5'd7, 21'd12));                  // 32: x7 = PC+4, jump -> 44
        emit(ADDI(5'd8,  5'd0, 12'd333));          // 36: WRONG PATH (squash)
        emit(ADDI(5'd9,  5'd0, 12'd444));          // 40: WRONG PATH (squash)
        emit(ADDI(5'd10, 5'd0, 12'd3));            // 44: x10 = 3 (JAL target)
        emit(ADDI(5'd12, 5'd0, 12'd0));            // 48: sum = 0
        emit(ADDI(5'd13, 5'd0, 12'd5));            // 52: i = 5
        emit(ADD (5'd12, 5'd12, 5'd13));           // 56: loop: sum += i
        emit(ADDI(5'd13, 5'd13, -12'sd1));         // 60: i--
        emit(BNE (5'd13, 5'd0, -13'sd8));          // 64: i != 0 -> 56 (uses i just written)
        emit(AUIPC(5'd14, 20'd0));                 // 68: x14 = PC
        emit(JALR(5'd15, 5'd14, 12'd16));          // 72: x15 = PC+4, jump -> 84
        emit(ADDI(5'd16, 5'd0, 12'd555));          // 76: WRONG PATH (squash)
        emit(ADDI(5'd17, 5'd0, 12'd666));          // 80: WRONG PATH (squash)
        emit(ADDI(5'd18, 5'd0, 12'd7));            // 84: x18 = 7 (JALR target)
        emit(JAL (5'd0, 21'd0));                   // 88: halt
        run_program(110);

        check_reg(5'd1,  32'd5,                   "x1");
        check_reg(5'd2,  32'd5,                   "x2");
        check_reg(5'd3,  32'd0,                   "squashed after BEQ");
        check_reg(5'd4,  32'd0,                   "squashed after BEQ");
        check_reg(5'd5,  32'd1,                   "BEQ target ran");
        check_reg(5'd6,  32'd2,                   "BNE not taken");
        check_reg(5'd7,  BOOT_ADDR + 32'd36,      "JAL link (PC+4)");
        check_reg(5'd8,  32'd0,                   "squashed after JAL");
        check_reg(5'd9,  32'd0,                   "squashed after JAL");
        check_reg(5'd10, 32'd3,                   "JAL target ran");
        check_reg(5'd12, 32'd15,                  "loop sum 1..5");
        check_reg(5'd13, 32'd0,                   "loop counter");
        check_reg(5'd14, BOOT_ADDR + 32'd68,      "auipc");
        check_reg(5'd15, BOOT_ADDR + 32'd76,      "JALR link (PC+4)");
        check_reg(5'd16, 32'd0,                   "squashed after JALR");
        check_reg(5'd17, 32'd0,                   "squashed after JALR");
        check_reg(5'd18, 32'd7,                   "JALR target ran");

        //===============================================================
        // Summary
        //===============================================================
        $display("\n----------------------------------------------");
        if (errors == 0)
            $display(" ALL TESTS PASSED  (%0d checks)", checks);
        else
            $display(" TESTS FAILED: %0d of %0d checks failed", errors, checks);
        $display("----------------------------------------------\n");
        $finish;
    end

    // Watchdog
    initial begin
        #200000;
        $display("[TB] TIMEOUT");
        $finish;
    end

endmodule