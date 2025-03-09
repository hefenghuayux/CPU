`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/12/21 15:23:04
// Design Name: 
// Module Name: tb_test
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


module tb_test;
    reg clk;
    reg rstn;
    reg [15:0]sw_i;
     wire [7:0]disp_an_o;
     wire [7:0]disp_seg_o;
     parameter T=320;
     initial clk = 0 ;
always #5 clk = ~clk;
initial begin
    rstn=1'b0;
    #8 rstn=1'b1;
    end
 initial begin
 #10;
 repeat(14)
 begin
 sw_i=15'b010000000000000;
 #10240;
  #10240;
  end

 end
    sccomp t_sccomp(
    .clk(clk),
     .rstn(rstn),
     .sw_i(sw_i),
     .disp_an_o(disp_an_o),
     .disp_seg_o(disp_seg_o)
    );
//    U_CTRL t_CTRL(
//    .Op(Op),
//    .Funct3(Funct3),
//    .Funct7(Funct7),
//    .Zero(Zero),
//    .RegWrite(RegWrite),
//    .MemWrite(MemWrite),
//    .EXTOp(EXTOp),
//    .ALUOp(ALUOp),
//    .ALUSrc(ALUSrc),
//    .DMType(DMType),
//    .WDSel(WDSel)
//    );
 always begin
#100
if($time>=409600)
    begin
    $finish;
    end
    end   
endmodule
