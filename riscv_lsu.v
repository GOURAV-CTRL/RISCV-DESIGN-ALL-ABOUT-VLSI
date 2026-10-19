//-----------------------------------------------------------------------------
// riscv_lsu.v
//
// MEM stage: load/store address formatting, byte/half/word lane alignment,
// sign/zero extension on loads, byte-lane write-strobe generation on
// stores, and misaligned-access detection. Drives a generic word-oriented
// memory interface (address + byte write-strobes + write data / read data)
// suitable for the TCM RAM's CPU-side port.
//
// The effective address (rs1 + imm) is computed upstream by the ALU in
// riscv_exec.v; this module only aligns and extends around whatever
// address it's given.
//
// Project : 5-Stage RISC-V (RV32IM) AXI CPU Core - TCM + AXI4 profile
// Target  : Verilog-2001, synthesizable, purely combinational
//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_lsu
(
    input  wire [`RISCV_XLEN-1:0]  addr_i,        // effective address (from EX-stage ALU)
    input  wire [`RISCV_XLEN-1:0]  store_data_i,  // rs2 value, for stores
    input  wire [2:0]              funct3_i,      // width/sign, see riscv_defs.v
    input  wire                    mem_read_i,
    input  wire                    mem_write_i,

    // ---------------- Memory-side interface (TCM CPU port) ----------------
    output wire [`RISCV_XLEN-1:0]  mem_addr_o,
    output wire [`RISCV_XLEN-1:0]  mem_wdata_o,
    output wire [3:0]              mem_wstrb_o,   // byte write-enables
    output wire                    mem_write_o,
    output wire                    mem_read_o,

    input  wire [`RISCV_XLEN-1:0]  mem_rdata_i,   // raw word read from memory

    // ---------------- To WB stage ----------------
    output wire [`RISCV_XLEN-1:0]  load_data_o,   // sign/zero-extended result
    output wire                    misaligned_o   // address misalignment (LH/SH must be
                                                   // 2-byte aligned, LW/SW must be 4-byte)
);

    wire [1:0] byte_offset = addr_i[1:0];

    //-------------------------------------------------------------------
    // Store path: byte-lane write-strobe + data shifted into position.
    // The shift is applied unconditionally; only the enabled lanes
    // (selected by wstrb) actually reach memory, so it's harmless for
    // narrower stores that only look at store_data_i's low bits.
    //-------------------------------------------------------------------
    reg  [3:0] wstrb_r;
    reg        misaligned_store;

    always @(*)
    begin
        wstrb_r          = 4'b0000;
        misaligned_store = 1'b0;
        case (funct3_i)
            `F3_SB:
                wstrb_r = 4'b0001 << byte_offset;

            `F3_SH:
            begin
                wstrb_r          = 4'b0011 << byte_offset; // here if byte offset is 00 then wstrb is 0011
                                                           // if byte offset is 01 then wstrb is 0110 that means we are selecting middle 2 bytes
                                                           // similarly when byte offset is 10 then also it works
                                                           //but if byte offset is 11 then strb value becomdes 1000 but sh needs two bytes
                misaligned_store = byte_offset[0]; // must be 2-byte aligned
                                                    //in 2 byte alignment the value should be divisible 2 or lsb be should be equal to 0
            end

            `F3_SW:
            begin
                wstrb_r          = 4'b1111;
                misaligned_store = (byte_offset != 2'b00); // must be 4-byte aligned
            end

            default: ; // not a store (mem_write_i is 0), values unused
        endcase
    end

    wire [`RISCV_XLEN-1:0] wdata = store_data_i << (byte_offset * 8);

    //-------------------------------------------------------------------
    // Load path: select the addressed byte/halfword/word out of the
    // memory's word-wide read data, then sign- or zero-extend.
    //-------------------------------------------------------------------
    reg  [`RISCV_XLEN-1:0] load_data_r;
    reg                    misaligned_load;

    always @(*)
    begin
        load_data_r     = mem_rdata_i;
        misaligned_load = 1'b0;
        case (funct3_i)
            `F3_LB:
                case (byte_offset)
                    2'b00: load_data_r = {{24{mem_rdata_i[7]}},  mem_rdata_i[7:0]};
                    2'b01: load_data_r = {{24{mem_rdata_i[15]}}, mem_rdata_i[15:8]};
                    2'b10: load_data_r = {{24{mem_rdata_i[23]}}, mem_rdata_i[23:16]};
                    2'b11: load_data_r = {{24{mem_rdata_i[31]}}, mem_rdata_i[31:24]};
                endcase

            `F3_LBU:
                case (byte_offset)
                    2'b00: load_data_r = {24'b0, mem_rdata_i[7:0]};
                    2'b01: load_data_r = {24'b0, mem_rdata_i[15:8]};
                    2'b10: load_data_r = {24'b0, mem_rdata_i[23:16]};
                    2'b11: load_data_r = {24'b0, mem_rdata_i[31:24]};
                endcase

            `F3_LH:
            begin
                misaligned_load = byte_offset[0];
                load_data_r     = byte_offset[1] ? {{16{mem_rdata_i[31]}}, mem_rdata_i[31:16]}
                                                  : {{16{mem_rdata_i[15]}}, mem_rdata_i[15:0]};
            end

            `F3_LHU:
            begin
                misaligned_load = byte_offset[0];
                load_data_r     = byte_offset[1] ? {16'b0, mem_rdata_i[31:16]}
                                                  : {16'b0, mem_rdata_i[15:0]};
            end

            `F3_LW:
            begin
                misaligned_load = (byte_offset != 2'b00);
                load_data_r     = mem_rdata_i;
            end

            default: ; // not a load (mem_read_i is 0), value unused
        endcase
    end

    //-------------------------------------------------------------------
    // Combine and drive the memory-side interface. Reads/writes are
    // suppressed on a misaligned access as a safety net; full exception
    // reporting (mcause = *_ADDR_MISALIGNED) is wired up once riscv_csr.v
    // exists.
    //-------------------------------------------------------------------
    assign misaligned_o = mem_write_i ? misaligned_store :
                           mem_read_i  ? misaligned_load  : 1'b0;

    assign mem_addr_o  = addr_i;
    assign mem_wdata_o = wdata;
    assign mem_wstrb_o = (mem_write_i && !misaligned_o) ? wstrb_r : 4'b0000;
    assign mem_write_o = mem_write_i && !misaligned_o;
    assign mem_read_o  = mem_read_i  && !misaligned_o;

    assign load_data_o = load_data_r;

endmodule