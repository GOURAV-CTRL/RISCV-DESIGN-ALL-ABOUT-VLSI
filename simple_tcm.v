
`timescale 1ns / 1ps 
////////////////////////////////////////////////////////////////////////////////// 
// Company:  
// Engineer:  
//  
// Create Date: 08.09.2026 14:45:22 
// Design Name:  
// Module Name: tcm_ram 
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
 
//64KB - 16384 
//EACH LOCATION IS GOING TO HAVE 4 BYTES OF DATA   
 
module tcm_ram#(parameter addr_width = 14) (input clk_i,input rst_i, 
 
//PORT A  
input [addr_width - 1 : 0] addr_a_i, 
input [31:0] wdata_a_i, 
input [3:0] wstrb_a_i, 
output reg [31:0] rdata_a_o , 
 
//PORT B  
//(LSU/DATA) 
input [addr_width - 1 : 0] addr_b_i, 
input [31:0] wdata_b_i, 
input [3:0] wstrb_b_i, 
output reg [31:0] rdata_b_o 
 
); 
 
localparam depth = ( 1 << addr_width); //2 ^ addr_width  
 
reg [31:0] mem [0:depth - 1]; 
 
 
integer i; 
 
always@(posedge clk_i) begin 
    if(rst_i) 
        for(i = 0;i<depth;i=i+1) 
            mem[i] <= 32'h0000_0000; 
    
   end 
    
    
   //logic for port a 
   //this port is written using axi interface 
    
   always@(posedge clk_i) 
    begin 
    if(wstrb_a_i[0]) 
        mem[addr_a_i][7:0] <= wdata_a_i[7:0]; 
    if(wstrb_a_i[1]) 
        mem[addr_a_i][15:8] <= wdata_a_i[15:8];
    if(wstrb_a_i[2]) 
        mem[addr_a_i][23:16] <= wdata_a_i[23:16]; 
    if(wstrb_a_i[3]) 
        mem[addr_a_i][31:24] <= wdata_a_i[31:24]; 
        
       rdata_a_o <= mem[addr_a_i]; //data going to if 
         
   end 
    
    //port b 
     
   always@(posedge clk_i) 
    begin 
    if(wstrb_b_i[0]) 
        mem[addr_b_i][7:0] <= wdata_b_i; 
    if(wstrb_b_i[1]) 
        mem[addr_b_i][15:8] <= wdata_b_i; 
    if(wstrb_b_i[2]) 
        mem[addr_b_i][23:16] <= wdata_b_i; 
    if(wstrb_b_i[3]) 
        mem[addr_b_i][31:24] <= wdata_b_i; 
        
       rdata_b_o <= mem[addr_b_i]; //data going to if 
         
   end 
    
    
         
     
endmodule 
