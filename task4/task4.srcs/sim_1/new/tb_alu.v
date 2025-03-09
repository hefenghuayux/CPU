`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/21 15:31:40
// Design Name: 
// Module Name: tb_alu
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


module tb_alu;
     reg signed [31:0] 	A, B;  //alu input num
     reg [4:0]  			ALUOp; //alu how to do 
    wire  signed [31:0] 	C; // alu result 
    wire [7:0] 		Zero;
initial begin
A=8'h00000001;
B=8'h40030201;
ALUOp=5'b01100;
end
 U_alu t_alu(
 .A(A),
 .B(B),
 .C(C),
 .ALUOp(ALUOp),
 .Zero(Zero)
 );
 always begin
#100
if($time>=1000)
    begin
    $finish;
    end
    end   
endmodule
