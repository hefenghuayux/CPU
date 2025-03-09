`timescale 1ns / 1ps

`define NPC_PLUS4   3'b000
`define NPC_BRANCH  3'b001
`define NPC_JUMP    3'b010
`define NPC_JALR 3'b100

// ALU control signal
`define ALU_NOP   3'b000 
`define ALU_ADD   3'b001
`define ALU_SUB   3'b010 
`define ALU_AND   3'b011
`define ALU_OR    3'b100

//EXT CTRL itype, stype, btype, utype, jtype
`define EXT_CTRL_ITYPE_SHAMT 6'b100000
`define EXT_CTRL_ITYPE	6'b010000
`define EXT_CTRL_STYPE	6'b001000
`define EXT_CTRL_BTYPE	6'b000100
`define EXT_CTRL_UTYPE	6'b000010
`define EXT_CTRL_JTYPE	6'b000001

`define GPRSel_RD 2'b00
`define GPRSel_RT 2'b01
`define GPRSel_31 2'b10

`define WDSel_FromALU 2'b00
`define WDSel_FromMEM 2'b01
`define WDSel_FromPC 2'b10

`define ALUOp_nop 5'b00000
`define ALUOp_lui 5'b00001
`define ALUOp_auipc 5'b00010
`define ALUOp_add 5'b00011
`define ALUOp_sub 5'b00100
`define ALUOp_bne 5'b00101
`define ALUOp_blt 5'b00110
`define ALUOp_bge 5'b00111
`define ALUOp_bltu 5'b01000
`define ALUOp_bgeu 5'b01001
`define ALUOp_slt 5'b01010
`define ALUOp_sltu 5'b01011
`define ALUOp_xor 5'b01100
`define ALUOp_or 5'b01101
`define ALUOp_and 5'b01110
`define ALUOp_sll 5'b01111
`define ALUOp_srl 5'b10000
`define ALUOp_sra 5'b10001


`define dm_word 3'b000
`define dm_halfword 3'b001
`define dm_halfword_unsigned 3'b010
`define dm_byte 3'b011
`define dm_byte_unsigned 3'b100
module sccomp(
    input clk,input rstn,input [15:0]sw_i, output [7:0]disp_an_o,output [7:0]disp_seg_o
    );
    reg[31:0]clkdiv;
    wire Clk_CPU;
    always@(posedge clk or negedge rstn) begin
        if(!rstn) clkdiv<=0;
        else clkdiv<=clkdiv+1'b1; end
//    assign Clk_CPU=(sw_i[15])? clkdiv[27] : clkdiv[25];
assign Clk_CPU=(sw_i[15])? clkdiv[25] : clkdiv[10];

    reg[63:0] display_data;  
    wire jal;
    wire [31:0] instr;
    reg[31:0] reg_data;
    reg[31:0] dmem_data;
    reg [4:0] reg_addr;
     reg [4:0]dmem_addr;
    //sw_i[5:2] 寄存器号    sw_i[9:6]dmem号
    //sw_i[14]    1;rom
    //sw_i[13]   1:寄存器模式  
    //sw_i[12]  1:dmem
    //sw_i[1] == 0  Rom自增
    always@(sw_i)begin
        reg_addr=sw_i[5:2];
        dmem_addr=sw_i[5:2];
        reg_data=U_RF.rf[reg_addr];
        dmem_data=U_DM.dmem[dmem_addr];       
    end
    //数码管显示RF/MEM中的数据
    always@(sw_i) begin
      if(sw_i[14]==1) display_data = instr;
      else  if(sw_i[13]==1)display_data={reg_addr[3:0],reg_data[27:0]};
        else if(sw_i[12]==1)display_data={dmem_addr[3:0],dmem_data[27:0]};
    end

    //例化ROM
    parameter IM_CODE_NUM = 19;
    reg [31:0] rom_addr;
    dist_mem_gen_0 U_IM(
    .a(rom_addr),
    .spo(instr)
    );  
 wire[31:0] immout;
 wire signed[31:0] aluout;
 wire branch;
 wire Zero;
 wire[2:0] NPCOp;
      always@(posedge Clk_CPU or negedge rstn) begin
        if(!rstn) begin rom_addr = 6'b0; end
        else if((!sw_i[1])&&rom_addr != IM_CODE_NUM) begin
            case(NPCOp)
                `NPC_PLUS4:rom_addr<=rom_addr+1'b1;
                `NPC_BRANCH:rom_addr<=rom_addr+immout;
                `NPC_JUMP:rom_addr<=rom_addr+immout;
                `NPC_JALR:rom_addr<=aluout;
            endcase
  end
  end
//      always@(posedge Clk_CPU or negedge rstn) begin
//        if(!rstn) begin rom_addr <= 6'b0; end
//        else if(!sw_i[1]) begin
//                       case(NPCOp)
//                `NPC_PLUS4:rom_addr<=rom_addr+1'b1;
//                `NPC_BRANCH:rom_addr<=rom_addr+immout;
//                `NPC_JUMP:rom_addr<=rom_addr+immout;
//                `NPC_JALR:rom_addr<=aluout;
//            if (rom_addr == IM_CODE_NUM) begin rom_addr <= rom_addr ;end
//        end
//        end
//        else 
//        begin
//        rom_addr <= rom_addr;
//  end
//  end

    //ctrl的接口
     wire[6:0] Op = instr[6:0];  // op
    wire[6:0] Funct7 = instr[31:25]; // funct7
    wire[2:0] Funct3 = instr[14:12]; // funct3
    wire[4:0] rs1 = instr[19:15];  // rs1
    wire[4:0] rs2 = instr[24:20];  // rs2
    wire[4:0] rd = instr[11:7];  // rd
    wire[11:0] iimm=instr[31:20];//addi 指令立即数，lw指令立即数
    wire[11:0] simm={instr[31:25],instr[11:7]}; //sw指令立即数
    wire[19:0] jimm=instr[31:12];//jal指令立即数
    wire[11:0] bimm={{{instr[31],instr[7]},instr[30:25]},instr[11:8]};
    //实例化Ctrl
    
    wire RegWrite;
    wire MemWrite;
    wire[5:0] EXTOp;
    wire[4:0] ALUOp;
    wire ALUSrc;
    wire[2:0] DMType;
    wire [1:0]WDSel;
    //CTRL的例化
    ctrl U_CTRL(
    .Op(Op),
    .Funct3(Funct3),
    .Funct7(Funct7),
    .Zero(Zero),
    .RegWrite(RegWrite),
    .MemWrite(MemWrite),
    .EXTOp(EXTOp),
    .ALUOp(ALUOp),
    .ALUSrc(ALUSrc),
    .DMType(DMType),
    .WDSel(WDSel),
    .branch(branch),
    .jal(jal),
    .NPCOp(NPCOp)
    );
      
    //reg[2:0] alu_addr = 0;
        EXT U_EXT(
        .iimm(iimm),
        .simm(simm),
        .bimm(bimm),
        .jimm(jimm),
        .EXTOp(EXTOp),
        .immout(immout)
        );
    //rf
    wire[31:0] RD1,RD2;
   wire [31:0]dmem_dout;
    reg[31:0] WD;
    reg [4:0]addr;
//    assign addr=rom_addr+1'b1;
    always@(*)
        begin
            case(WDSel)
                `WDSel_FromALU: WD=aluout;
                `WDSel_FromPC:WD=rom_addr+1;
                `WDSel_FromMEM: WD=dmem_dout;                             
            endcase
        end

    
   // parameter REG_NUM =32;

    RF U_RF(
        .clk(Clk_CPU),
        .rstn(rstn),
        .RFWr(RegWrite),
        .A1(rs1),
        .A2(rs2),
        .A3(rd),
        .WD(WD),
        .sw_i(sw_i),
        .RD1(RD1),
        .RD2(RD2)
    );


   
  
   
    wire[31:0] B ;
    assign B = (ALUSrc) ? immout : RD2;
    alu U_alu(
        .A(RD1), 
        .B(B), 
        .ALUOp(ALUOp), 
        .C(aluout),
        .Zero(Zero)
    );

    dm U_DM(
        .clk(clk),
        .rstn(rstn),
        .sw_i(sw_i),
        .DMWr(MemWrite),
        .addr(aluout),
        .din(RD2),
        .DMType(DMType),
        .dout(dmem_dout)
    );



     seg7x16 u_seg7x16(
        .clk(clk),
        .rstn(rstn),
        .i_data(display_data),
        .disp_mode(sw_i[0]),
        .o_seg(disp_seg_o),
        .o_sel(disp_an_o)
        );
    
endmodule


module ctrl(
    input [6:0] Op,  //opcode
    input [6:0] Funct7,  //funct7 
    input [2:0] Funct3,    // funct3 
    input Zero,
    output RegWrite, // control signal for register write
    output MemWrite, // control signal for memory write
    output	[5:0]EXTOp,    // control signal to signed extension
    output [4:0] ALUOp,    // ALU opertion
    output [2:0] NPCOp,    // next pc operation
    output ALUSrc,   // ALU source for b
    output [2:0] DMType, //dm r/w type
    output [1:0]WDSel,    // (register) write data selection  (MemtoReg)
    output branch,
    output jal
    );
    //R_type:
    wire rtype  = ~Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0110011
    wire r_xor=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&~Funct3[0];
    wire r_sra=rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&Funct3[0];
    wire r_sll=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&Funct3[0];
    wire i_add=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
    wire i_sub=rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
    //i_l type  
    wire itype_l  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
    wire i_lb=itype_l&~Funct3[2]& ~Funct3[1]& ~Funct3[0]; //lb 000
    wire i_lh=itype_l&~Funct3[2]& ~Funct3[1]& Funct3[0];  //lh 001
    wire i_lw=itype_l&~Funct3[2]& Funct3[1]& ~Funct3[0];  //lw 010
    // i_i type
    wire itype_r  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0010011
    wire i_addi  =  itype_r& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // addi 000 func3
    // s format
    wire stype  = ~Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//0100011
    wire i_sw   = stype&~Funct3[2]& Funct3[1]& ~Funct3[0]; // sw 010
    wire i_sb=stype& ~Funct3[2]& ~Funct3[1]&~Funct3[0];
    wire i_sh=stype&& ~Funct3[2]&~Funct3[1]&Funct3[0];
    //sb format
    wire sbtype=Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];
    wire sb_beq=sbtype&~Funct3[2]&~Funct3[1]&~Funct3[0];
    wire jtype  = Op[6]&Op[5]&~Op[4]&Op[3]&Op[2]&Op[1]&Op[0];
    //jalr
      wire itype_j = Op[6]&Op[5]&~Op[4]&~Op[3]&Op[2]&Op[1]&Op[0]; //1100111
    wire i_jalr = itype_j& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // jalr 000 func3

    assign NPCOp[0]=sb_beq&Zero;
    assign NPCOp[1]=jal;
    assign NPCOp[2]=i_jalr;

    //操作指令生成控制信号（写、MUX选择）
    assign RegWrite   = rtype | itype_r|itype_l |i_jalr|jal   ; // register write
    assign MemWrite   = stype;              // memory write
    assign ALUSrc     = itype_r | stype | itype_l|i_jalr ; // ALU B is from instruction immediate
    assign branch=sb_beq;
    //mem2reg=wdsel ,WDSel_FromALU 2'b00  WDSel_FromMEM 2'b01
    assign WDSel[0] = itype_l;   
    assign WDSel[1] = jtype|i_jalr; 
    //操作指令生成运算类型aluop
    //ALUOp_nop 5'b00000
    //ALUOp_lui 5'b00001
    //ALUOp_auipc 5'b00010
    //ALUOp_add 5'b00011
    assign ALUOp[0]= i_add  | i_addi|stype|itype_l |r_sll|r_sra|i_jalr;
    assign ALUOp[1]= i_add  | i_addi|stype|itype_l|r_sll |i_jalr;
    assign ALUOp[2]=r_xor|r_sll|sb_beq;
   assign ALUOp[3]=r_xor|r_sll;
   assign ALUOp[4]=r_sra;
    //assign EXTOp[2]=sb_beq;  
  
    
        assign EXTOp[2]=sb_beq;
    assign EXTOp[4]=itype_l|itype_r|i_jalr;
    assign EXTOp[3]=stype;
    assign EXTOp[0]=jtype;
    assign EXTOp[1]=1'b0;
   assign EXTOp[5]=1'b0;
    assign jal=jtype;
    //根据具体S和i_L指令生成DataMem数据操作类型编码
    // dm_word 3'b000
    //dm_halfword 3'b001
    //dm_halfword_unsigned 3'b010
    //dm_byte 3'b011
    //dm_byte_unsigned 3'b100
    //assign DMType[2]=i_lbu;
    //assign DMType[1]=i_lb | i_sb | i_lhu;
    assign DMType[1]=i_lb | i_sb;
    assign DMType[0]=i_lh | i_sh | i_lb | i_sb;
endmodule

module dm(
    input clk,
    input rstn,
    input DMWr,
    input [5:0]	addr,
    input [31:0] din,
    input [2:0]	DMType,
    input 	[15:0] sw_i, 
    output reg [31:0] dout 
    );
    reg [7:0] dmem[31:0];
    integer i;
    initial begin
        for(i=0;i<32;i=i+1)
            dmem[i]<=i;
    end
    always@(posedge clk,negedge rstn)begin
        if(!rstn)begin
            for(i=0;i<32;i=i+1)
                dmem[i]<=i;
        end
        else begin
            if(DMWr&&(!sw_i[1]))begin
                case(DMType)
                    `dm_byte:dmem[addr]<=din[7:0];
                    `dm_halfword:begin
                        dmem[addr]<=din[7:0];
                        dmem[addr+1]<=din[15:8];
                    end
                    `dm_word:begin
                        dmem[addr]<=din[7:0];
                        dmem[addr+1]<=din[15:8];
                        dmem[addr+2]<=din[23:16];
                        dmem[addr+3]<=din[31:24];
                    end
                endcase
            end
        end
    end
    always@(*)begin
        case(DMType)
            `dm_byte:dout={{24{dmem[addr][7]}},dmem[addr][7:0]};
            `dm_halfword:dout={{16{dmem[addr+1][7]}},dmem[addr+1][7:0],dmem[addr][7:0]};
            `dm_word:dout={dmem[addr+3][7:0],dmem[addr+2][7:0],dmem[addr+1][7:0],dmem[addr][7:0]};
        endcase
    end
endmodule
module alu(
    input signed [31:0] 	A, B,  //alu input num
    input [4:0]  			ALUOp, //alu how to do 
    output signed [31:0] 	C, // alu result 
    output [7:0] 		Zero
    );
    // ALU control code
//    `define  ALUOp_add  5'b00000
//    `define  ALUOp_sub  5'b00001
    reg signed [31:0] C_reg;
    reg [7:0] Zero_reg;
    integer x;
    always@(*)begin
        case(ALUOp)
            `ALUOp_add:C_reg=A+B;
            `ALUOp_sub:C_reg=A-B;
            `ALUOp_xor:C_reg=A^B;
            `ALUOp_sll:C_reg=A<<B;
            `ALUOp_sra:C_reg=A>>>B;            
            default: C_reg=A+B;
        endcase
        Zero_reg=(C==0)?1:0;
    end
    assign C=C_reg;
    assign Zero=Zero_reg;
endmodule

module RF(
    input	clk,							//100MHZ CLK
    input	rstn,							//reset signal
    input	RFWr,						//Rfwrite = mem2reg  
    input 	[15:0] sw_i, 		 		//sw_i[15]---sw_i[0]
    input 	[4:0] A1, A2, A3,		// Register Num 
    input 	[31:0] WD,					//Write data
    output [31:0] RD1, RD2	//Data output port
    );
    reg [31:0] rf[31:0];
    integer i;
    initial@(*)begin
        for(i=0;i<32;i=i+1)
            rf[i]<=i;
    end
    always@(posedge clk,negedge rstn)
        if(!rstn)begin
            for(i=0;i<32;i=i+1)
                rf[i]<=i;
        end
        else
            if(RFWr&&(!sw_i[1]))begin
                rf[A3]<=WD;
            end
    assign RD1=(A1!=0)?rf[A1]:0;
    assign RD2=(A2!=0)?rf[A2]:0;
    
endmodule
//EXT CTRL itype, stype, btype, utype, jtype

module EXT(
    input [4:0] iimm_shamt, //
    input [11:0]	iimm,  //instr[31:20], 12 bits
    input [11:0]	simm, //instr[31:25, 11:7], 12 bits
    input [11:0]	bimm,//instrD[31],instrD[7], instrD[30:25], instrD[11:8], 12 bits
    input [19:0]	uimm,
    input [19:0]	jimm,
    input [5:0]	 EXTOp,
    output reg [31:0] 	immout
    );
    always@(*)begin
        case (EXTOp)
		`EXT_CTRL_ITYPE_SHAMT:   immout<={27'b0,iimm_shamt[4:0]};
		`EXT_CTRL_ITYPE:	immout<={{20{iimm[11]}},iimm[11:0]};
		`EXT_CTRL_STYPE:	immout<={{20{simm[11]}},simm[11:0]};
		`EXT_CTRL_BTYPE:    immout<={{19{bimm[11]}},bimm[11:0],1'b0};
		`EXT_CTRL_UTYPE:	immout <= {uimm[19:0], 12'b0}; 
		`EXT_CTRL_JTYPE:	immout<={{11{jimm[19]}},jimm[19:0],1'b0};
		default:	        immout <= 32'b0;
	 endcase

    end
endmodule
module seg7x16(
    input clk,
    input rstn,
    input disp_mode,//0 txt,1 graph
    input [63:0] i_data,
    output [7:0] o_seg,
    output [7:0] o_sel
    );
    //分频
    reg [14:0] cnt;
    wire seg7_clk;
    always@(posedge clk,negedge rstn)
        if(!rstn)
            cnt<=0;
        else
            cnt<=cnt+1'b1;
    assign seg7_clk=cnt[14];
    //8选1
    reg[2:0] seg7_addr;
    always@(posedge seg7_clk,negedge rstn)
        if(!rstn)
            seg7_addr<=0;
        else
            seg7_addr<=seg7_addr+1'b1;
    //输出选中数码管使能信号
    reg[7:0] o_sel_r;
    always@(*)
        case(seg7_addr)
            7:o_sel_r=8'b01111111;
            6:o_sel_r=8'b10111111;
            5:o_sel_r=8'b11011111;
            4:o_sel_r=8'b11101111;
            3:o_sel_r=8'b11110111;
            2:o_sel_r=8'b11111011;
            1:o_sel_r=8'b11111101;
            0:o_sel_r=8'b11111110;
        endcase
    //8个数码管显示数字串
    reg[63:0] i_data_store;
    always@(posedge clk,negedge rstn)
        if(!rstn)
            i_data_store<=0;
        else
            i_data_store<=i_data;
    //当前一个数码管要显示的数串
    reg[7:0] seg_data_r;
    always@(*)
        //字符显示模式
        if(disp_mode==1'b0) begin
            case(seg7_addr)
                0:seg_data_r=i_data_store[3:0];
                1:seg_data_r=i_data_store[7:4];
                2:seg_data_r=i_data_store[11:8];
                3:seg_data_r=i_data_store[15:12];
                4:seg_data_r=i_data_store[19:16];
                5:seg_data_r=i_data_store[23:20];
                6:seg_data_r=i_data_store[27:24];
                7:seg_data_r=i_data_store[31:28];
            endcase end
        //图形显示模式
        else begin 
            case (seg7_addr)
                0:seg_data_r = i_data_store[7:0];
                1:seg_data_r = i_data_store[15:8];
                2:seg_data_r = i_data_store[23:16];
                3:seg_data_r = i_data_store[31:24];
                4:seg_data_r = i_data_store[39:32];
                5:seg_data_r = i_data_store[47:40];
                6:seg_data_r = i_data_store[55:48];
                7:seg_data_r = i_data_store[63:56];
            endcase end
    //要显示数字的7段码
    reg [7:0] o_seg_r;
    always@(posedge clk, negedge rstn)
        if(!rstn)
            o_seg_r<=8'hff;
        //字符模式需要译码
        else if(disp_mode==1'b0)begin
            case(seg_data_r)
                4'h0 :o_seg_r<= 8'hC0;
                4'h1:o_seg_r<= 8'hF9;
                4'h2:o_seg_r<= 8'hA4;
                4'h3:o_seg_r<= 8'hB0;
                4'h4:o_seg_r<= 8'h99;
                4'h5:o_seg_r<= 8'h92;
                4'h6:o_seg_r<= 8'h82;
                4'h7:o_seg_r<= 8'hF8;
                4'h8:o_seg_r<= 8'h80;
                4'h9:o_seg_r<= 8'h90;
                4'hA:o_seg_r<= 8'h88;
                4'hB:o_seg_r<= 8'h83;
                4'hC:o_seg_r<= 8'hC6;
                4'hD:o_seg_r<= 8'hA1;
                4'hE:o_seg_r<= 8'h86;
                4'hF:o_seg_r<= 8'h8E;
                default:o_seg_r<= 8'hFF;
            endcase end
        //图形模式直接输出
        else begin o_seg_r<=seg_data_r;end
    assign o_sel=o_sel_r;
    assign o_seg=o_seg_r;
endmodule
