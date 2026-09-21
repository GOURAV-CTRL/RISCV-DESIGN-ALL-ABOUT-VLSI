
`include "riscv_defs.v"
`timescale 1ns / 1ps 
////////////////////////////////////////////////////////////////////////////////// 
// Company:  
// Engineer:  
//  
// Create Date: 10.09.2026 17:24:00 
// Design Name:  
// Module Name: instruction_fetch 
// Project Name:  
// Target Devices:  
// Tool Versions:  
// Description:  
//  
// Dependencies:  
//  
// Revision: 
// Revision 0.01 - File Created 
// Additional Comments: 
//  
////////////////////////////////////////////////////////////////////////////////// 
 
//inst. fetch is basically fetching the instruction 
`include "riscv_defs.v" 
 
module instruction_fetch#(addr_width = 14)( 
    input clk_i,rst_i, 
    input [`RISCV_XLEN- 1: 0] boot_addr_i, 
    input stall_i, 
    input flush_i, 
    input redirect_i, 
    input [`RISCV_XLEN - 1 : 0] redirect_pc_i, 
     
    output [addr_width - 1 : 0] imem_addr_o, 
    input [31:0] imem_rdata_i, 
     
    output [`RISCV_XLEN - 1 : 0] pc_o, 
    output [31:0] instr_o 
     
     
    ); 
    
        reg  stall_d_q;

    wire [31:0] live_instr; 
     
    localparam [31:0] nop_instr = 32'h0000_0013; 
    //add i x0,x0,0 
     
    reg [`RISCV_XLEN - 1 : 0] pc_q; 
    reg [`RISCV_XLEN - 1 : 0] pc_d_q; 
     
    reg [31:0] instr_hold_q; // this is for holding instruction during stall stage 
     
     
    always@(posedge clk_i or posedge rst_i) 
        begin 
            if(rst_i) 
                pc_q <= boot_addr_i; 
             else if(redirect_i) 
                begin 
                    pc_q <= redirect_pc_i; //this is for branch insturctions 
                end      
              else if(!stall_i) 
                pc_q <= pc_q + 4; 
                 
            end 
            
              always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
            pc_d_q <= boot_addr_i;
        else if (!stall_i)
            pc_d_q <= pc_q;
        // else: stall - hold current pc_d_q (re-present the same PC)
    end
             
            always@(posedge clk_i or posedge rst_i) 
                begin    
                    if(rst_i) 
                        begin 
                            instr_hold_q <= nop_instr; 
                        end 
                         
                    else 
                        begin 
                            instr_hold_q <= instr_o; 
                             
                             
                            end 
                            end 
                            
                            
           always @(posedge clk_i or posedge rst_i)
    begin
        if (rst_i)
        begin
            stall_d_q    <= 1'b0;
            instr_hold_q <= nop_instr;
        end
        else
        begin
            stall_d_q    <= stall_i;
            instr_hold_q <= instr_o;
        end
    end
                             
         assign live_instr = flush_i ? nop_instr : imem_rdata_i; 
          
         assign instr_o = stall_d_q ? instr_hold_q : live_instr;    
                
      assign imem_addr_o = pc_q[addr_width+1:2];  
      
      assign pc_o = pc_d_q;          
//            assign pc_o = pc_q;          

endmodule 
