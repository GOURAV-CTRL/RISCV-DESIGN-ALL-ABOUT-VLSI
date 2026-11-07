

module riscv_pipe_ctrl
(
    input  wire  clk_i,
    input  wire  rst_i,

    input  wire  load_use_stall_i,   // from riscv_hazard.v
    input  wire  branch_redirect_i,  // from riscv_exec.v's redirect_o (EX stage, this cycle)
    input  wire  mem_stall_i,        // from riscv_tcm_top.v - external AXI-Lite access pending

    output wire  pc_stall_o,         // freeze fetch's PC / instr_o
    output wire  idex_bubble_o,      // force a NOP into the ID/EX latch this cycle
    output wire  fetch_flush_o       // squash the in-flight wrong-path fetch (1-cycle-delayed)
);

  
    assign pc_stall_o    = load_use_stall_i | mem_stall_i;

  
    assign idex_bubble_o = load_use_stall_i | branch_redirect_i;

    
    reg flush_q;
 //this register logic is introduced due to the latency we have in tcm
    always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
            flush_q <= 1'b0;
        else if (!mem_stall_i)
            flush_q <= branch_redirect_i;
     
    end

    assign fetch_flush_o = flush_q;

endmodule