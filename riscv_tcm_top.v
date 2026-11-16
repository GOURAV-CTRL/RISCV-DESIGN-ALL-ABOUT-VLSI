//-----------------------------------------------------------------------------
// riscv_tcm_top.v
//
// Top-level SoC wrapper: riscv_core.v + riscv_tcm_ram.v, plus the two AXI
// interfaces the core-level files (riscv_tcm_ram.v's header, riscv_core.v's
// "external data accesses" comment) were written expecting a wrapper to
// add. They serve two different masters and never contend for the same
// path:
//
//   1) AXI4 SLAVE "loader" port (s_axi_*) - an external host/DMA writes
//      (or reads back, for verification) the program image into the TCM
//      while rst_cpu_i is held. It's given Port A outright for the
//      duration (the core is in reset and isn't fetching), exactly as
//      riscv_tcm_ram.v's header describes. Full AXI4, INCR bursts only,
//      fixed 32-bit beats.
//
//   2) AXI4-LITE MASTER port (m_axi_*) - the CPU's OWN data accesses,
//      whenever the LSU's effective address falls outside the TCM
//      window. Port B still serves in-window accesses directly (1 cycle,
//      no stall, unchanged from before); anything else is turned into a
//      single-beat AXI4-Lite transaction out to your axi_lite_interconnect
//      so the core can reach GPIO/UART/SPI/I2C etc. riscv_core.v doesn't
//      know or care which path it's on - it just obeys dmem_stall_i for
//      as many cycles as this transaction takes.
//

//-----------------------------------------------------------------------------
`include "riscv_defs.v"

module riscv_tcm_top
#(
    parameter ADDR_WIDTH  = 14,          // TCM word-address bits (must match riscv_tcm_ram.v)
    parameter TCM_BASE    = 32'h0000_0000,
    parameter AXI_ID_W    = 4            // s_axi_*id width (not otherwise checked/used)
)
(
    input  wire                    clk_i,
    input  wire                    rst_i,      // async reset for everything (core, RAM ports, both AXI FSMs)
    input  wire                    rst_cpu_i,  // hold the CPU in reset & give the loader Port A
    input  wire [`RISCV_XLEN-1:0]  boot_addr_i,
    input  wire                    intr_i,

    // =====================================================================
    // AXI4 SLAVE - program loader (drives TCM Port A while rst_cpu_i is 1)
    // =====================================================================
    input  wire [AXI_ID_W-1:0]     s_axi_awid,
    input  wire [31:0]             s_axi_awaddr,
    input  wire [7:0]              s_axi_awlen,    // burst length - 1 (INCR only)
    input  wire                    s_axi_awvalid,
    output wire                    s_axi_awready,

    input  wire [31:0]             s_axi_wdata,
    input  wire [3:0]              s_axi_wstrb,
    input  wire                    s_axi_wlast,
    input  wire                    s_axi_wvalid,
    output wire                    s_axi_wready,

    output wire [AXI_ID_W-1:0]     s_axi_bid,
    output wire [1:0]              s_axi_bresp,    // always 2'b00 (OKAY)
    output wire                    s_axi_bvalid,
    input  wire                    s_axi_bready,

    input  wire [AXI_ID_W-1:0]     s_axi_arid,
    input  wire [31:0]             s_axi_araddr,
    input  wire [7:0]              s_axi_arlen,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,

    output wire [AXI_ID_W-1:0]     s_axi_rid,
    output wire [31:0]             s_axi_rdata,
    output wire [1:0]              s_axi_rresp,    // always 2'b00 (OKAY)
    output wire                    s_axi_rlast,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    // =====================================================================
    // AXI4-LITE MASTER - CPU data accesses outside the TCM window
    // =====================================================================
    output wire [31:0]             m_axi_awaddr,
    output wire [2:0]              m_axi_awprot,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,

    output wire [31:0]             m_axi_wdata,
    output wire [3:0]              m_axi_wstrb,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,

    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,

    output wire [31:0]             m_axi_araddr,
    output wire [2:0]              m_axi_arprot,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,

    input  wire [31:0]             m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready
);

    localparam TCM_BYTES = (1 << ADDR_WIDTH) * 4; //16384 words * 4bytes which is our total memory capacity

    //=====================================================================
    // Core <-> RAM Port A (instruction fetch), normal-operation wiring.
    // Overridden by the loader FSM below whenever it owns the port.
    //=====================================================================
    wire [ADDR_WIDTH-1:0] core_imem_addr;
    wire [31:0]            ram_rdata_a;

    wire                   loader_active;      // this FSM owns Port A this cycle
    wire [ADDR_WIDTH-1:0]  loader_addr_a;
    wire [31:0]            loader_wdata_a;
    wire [3:0]             loader_wstrb_a;

    wire [ADDR_WIDTH-1:0]  ram_addr_a  = loader_active ? loader_addr_a  : core_imem_addr;
    wire [31:0]            ram_wdata_a = loader_active ? loader_wdata_a : 32'h0000_0000;
    wire [3:0]             ram_wstrb_a = loader_active ? loader_wstrb_a : 4'b0000;

    //=====================================================================
    // Core <-> RAM Port B (data/LSU) + external-bus decode
    //=====================================================================
    wire [ADDR_WIDTH-1:0]  core_dmem_addr;
    wire [31:0]            core_dmem_wdata;
    wire [3:0]             core_dmem_wstrb;
    wire [31:0]            ram_rdata_b;

    wire [`RISCV_XLEN-1:0] core_dmem_addr_full;
    wire                   core_dmem_valid;
    wire                   core_dmem_we;
    wire                   core_dmem_stall;
    wire [31:0]            core_dmem_rdata;

    wire in_tcm_range = (core_dmem_addr_full >= TCM_BASE) &&
                         (core_dmem_addr_full <  TCM_BASE + TCM_BYTES); // here we are basically checking for the range that is from 00000 to 65535


    wire [3:0] ram_wstrb_b = in_tcm_range ? core_dmem_wstrb : 4'b0000; // here if the instruction is in tcm range
                                                                       // using core dmem wstrb we are accessing the memory
                                                                       // or we are making wstrb to 0 and not allowing to access the mem

    riscv_core #(.ADDR_WIDTH(ADDR_WIDTH)) u_core
    (
        .clk_i             (clk_i),
        .rst_i             (rst_i | rst_cpu_i),
        .boot_addr_i       (boot_addr_i),
        .intr_i            (intr_i),

        .imem_addr_o       (core_imem_addr),
        .imem_rdata_i      (ram_rdata_a),

        .dmem_addr_o       (core_dmem_addr),
        .dmem_wdata_o      (core_dmem_wdata),
        .dmem_wstrb_o      (core_dmem_wstrb),
        .dmem_rdata_i      (core_dmem_rdata),

        .dmem_addr_full_o  (core_dmem_addr_full),
        .dmem_valid_o      (core_dmem_valid),
        .dmem_we_o         (core_dmem_we),
        .dmem_stall_i      (core_dmem_stall)
    );

    riscv_tcm_ram #(.ADDR_WIDTH(ADDR_WIDTH)) u_ram
    (
        .clk_i      (clk_i),

        .addr_a_i   (ram_addr_a),
        .wdata_a_i  (ram_wdata_a),
        .wstrb_a_i  (ram_wstrb_a),
        .rdata_a_o  (ram_rdata_a),

        .addr_b_i   (core_dmem_addr),
        .wdata_b_i  (core_dmem_wdata),
        .wstrb_b_i  (ram_wstrb_b),
        .rdata_b_o  (ram_rdata_b)
    );

//using ext_needed signal we are basically stalling the fetching operation until the peripherals is ready again

    //=====================================================================
    // AXI4-Lite master FSM - services one core data access at a time.
    //
    
    localparam S_IDLE      = 3'd0,
               S_WRITE      = 3'd1,
               S_WRITE_RESP = 3'd2,
               S_READ_ADDR  = 3'd3,
               S_READ_DATA  = 3'd4,
               S_DONE       = 3'd5;

    reg [2:0]              ext_state;
    reg [31:0]              ext_addr_q, ext_wdata_q, ext_rdata_q;
    reg [3:0]               ext_wstrb_q;
    reg                     aw_accepted_q, w_accepted_q;

    wire ext_needed = core_dmem_valid && !in_tcm_range; // here if dmem is valid and its not a memory 
                                                        //transaction that means its an external bus access
                                                        // and core gets access to external bus

    wire aw_fire = m_axi_awvalid && m_axi_awready;
    wire w_fire  = m_axi_wvalid  && m_axi_wready;

    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            ext_state     <= S_IDLE;
            aw_accepted_q <= 1'b0;
            w_accepted_q  <= 1'b0;
        end
        else
        case (ext_state)
            S_IDLE:
                if (ext_needed)
                begin
                    ext_addr_q    <= core_dmem_addr_full;
                    ext_wdata_q   <= core_dmem_wdata;
                    ext_wstrb_q   <= core_dmem_wstrb;
                    aw_accepted_q <= 1'b0;
                    w_accepted_q  <= 1'b0;
                    ext_state     <= core_dmem_we ? S_WRITE : S_READ_ADDR;
                end

            S_WRITE:
            begin
                if (aw_fire) aw_accepted_q <= 1'b1;
                if (w_fire)  w_accepted_q  <= 1'b1;
                if ((aw_accepted_q || aw_fire) && (w_accepted_q || w_fire))
                    ext_state <= S_WRITE_RESP;
            end

            S_WRITE_RESP:
                if (m_axi_bvalid)                 // bresp not checked - wire an error flag up once
                    ext_state <= S_DONE;           // riscv_csr.v exists to report a bus-error trap

            S_READ_ADDR:
                if (m_axi_arvalid && m_axi_arready)
                    ext_state <= S_READ_DATA;

            S_READ_DATA:
                if (m_axi_rvalid)
                begin
                    ext_rdata_q <= m_axi_rdata;    // rresp not checked, same note as above
                    ext_state   <= S_DONE;
                end

            S_DONE:
                ext_state <= S_IDLE;

            default:
                ext_state <= S_IDLE;
        endcase
    end

    assign core_dmem_stall = (ext_state == S_WRITE)      ||
                              (ext_state == S_WRITE_RESP) ||
                              (ext_state == S_READ_ADDR)  ||
                              (ext_state == S_READ_DATA)  ||
                              (ext_state == S_IDLE && ext_needed);

   
    reg wb_in_tcm_range_q;
    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
            wb_in_tcm_range_q <= 1'b1;
        else if (!core_dmem_stall)
            wb_in_tcm_range_q <= in_tcm_range;
    end

    wire [31:0] dmem_rdata_live = wb_in_tcm_range_q ? ram_rdata_b : ext_rdata_q;


    reg        stall_q;
    reg [31:0] wb_rdata_hold_q;
    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            stall_q         <= 1'b0;
            wb_rdata_hold_q <= 32'h0000_0000;
        end
        else
        begin
            if (core_dmem_stall && !stall_q)
                wb_rdata_hold_q <= dmem_rdata_live;   // first stall cycle - capture the live value
            stall_q <= core_dmem_stall;
        end
    end

  
    assign core_dmem_rdata = stall_q ? wb_rdata_hold_q : dmem_rdata_live;

    assign m_axi_awaddr  = ext_addr_q;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awvalid = (ext_state == S_WRITE) && !aw_accepted_q;

    assign m_axi_wdata   = ext_wdata_q;
    assign m_axi_wstrb   = ext_wstrb_q;
    assign m_axi_wvalid  = (ext_state == S_WRITE) && !w_accepted_q;

    assign m_axi_bready  = (ext_state == S_WRITE_RESP);

    assign m_axi_araddr  = ext_addr_q;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arvalid = (ext_state == S_READ_ADDR);

    assign m_axi_rready  = (ext_state == S_READ_DATA);

    //=====================================================================
    // AXI4 slave loader FSM - owns Port A whenever rst_cpu_i is held.

    //=====================================================================
    localparam L_IDLE     = 3'd0,
               L_WR_DATA   = 3'd1,
               L_WR_RESP   = 3'd2,
               L_RD_ADDR   = 3'd3,
               L_RD_DATA   = 3'd4;

    reg [2:0]              ld_state;
    reg [AXI_ID_W-1:0]      ld_id_q;
    reg [ADDR_WIDTH-1:0]    ld_addr_q;
    reg [7:0]               ld_beats_left_q;

    assign loader_active = (ld_state != L_IDLE);

    assign s_axi_awready = rst_cpu_i && (ld_state == L_IDLE) && !s_axi_arvalid;
    assign s_axi_arready = rst_cpu_i && (ld_state == L_IDLE) && !(s_axi_awvalid && s_axi_awready);

    assign s_axi_wready  = (ld_state == L_WR_DATA);
    assign loader_addr_a  = ld_addr_q;
    assign loader_wdata_a = s_axi_wdata;
    assign loader_wstrb_a = (ld_state == L_WR_DATA) ? s_axi_wstrb : 4'b0000;

    assign s_axi_bid    = ld_id_q;
    assign s_axi_bresp  = 2'b00;
    assign s_axi_bvalid = (ld_state == L_WR_RESP);

    assign s_axi_rid    = ld_id_q;
    assign s_axi_rresp  = 2'b00;
    assign s_axi_rdata  = ram_rdata_a;
    assign s_axi_rlast  = (ld_state == L_RD_DATA) && (ld_beats_left_q == 8'd0);
    assign s_axi_rvalid = (ld_state == L_RD_DATA);

    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i || !rst_cpu_i)
        begin
            // Also resets on rst_cpu_i falling (CPU released) - an
            // in-flight load left incomplete when the host de-asserts
            // rst_cpu_i is simply abandoned, same as any other AXI
            // master that stops mid-burst without a reset.
            ld_state        <= L_IDLE;
            ld_addr_q       <= {ADDR_WIDTH{1'b0}};
            ld_beats_left_q <= 8'd0;
            ld_id_q         <= {AXI_ID_W{1'b0}};
        end
        else
        case (ld_state)
            L_IDLE:
                if (s_axi_awvalid && s_axi_awready)
                begin
                    ld_id_q         <= s_axi_awid;
                    ld_addr_q       <= s_axi_awaddr[ADDR_WIDTH+1:2];
                    ld_beats_left_q <= s_axi_awlen;
                    ld_state        <= L_WR_DATA;
                end
                else if (s_axi_arvalid && s_axi_arready)
                begin
                    ld_id_q         <= s_axi_arid;
                    ld_addr_q       <= s_axi_araddr[ADDR_WIDTH+1:2];
                    ld_beats_left_q <= s_axi_arlen;
                    ld_state        <= L_RD_ADDR;
                end

            L_WR_DATA:
                if (s_axi_wvalid && s_axi_wready)
                begin
                    if (s_axi_wlast || ld_beats_left_q == 8'd0)
                        ld_state <= L_WR_RESP;
                    else
                    begin
                        ld_addr_q       <= ld_addr_q + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                        ld_beats_left_q <= ld_beats_left_q - 8'd1;
                    end
                end

            L_WR_RESP:
                if (s_axi_bready)
                    ld_state <= L_IDLE;

            // ld_addr_q is presented to Port A combinationally the cycle
            // we're in L_RD_ADDR; riscv_tcm_ram.v registers rdata_a_o one
            // cycle later, which is exactly the cycle this moves into
            // L_RD_DATA - no extra wait state needed.
            L_RD_ADDR:
                ld_state <= L_RD_DATA;

            L_RD_DATA:
                if (s_axi_rvalid && s_axi_rready)
                begin
                    if (ld_beats_left_q == 8'd0)
                        ld_state <= L_IDLE;
                    else
                    begin
                        ld_addr_q       <= ld_addr_q + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                        ld_beats_left_q <= ld_beats_left_q - 8'd1;
                        ld_state        <= L_RD_ADDR;
                    end
                end
        endcase
    end

endmodule