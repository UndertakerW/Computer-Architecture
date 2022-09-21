`timescale 1ns/1ps

//General Registers 0-7:
`define gr0  5'b00000
`define gr1  5'b00001
`define gr2  5'b00010
`define gr3  5'b00011
`define gr4  5'b00100
`define gr5  5'b00101
`define gr6  5'b00110
`define gr7  5'b00111

module testbench;

reg clock;
reg init;
reg [319:0] i_datain;

//The CPU module
CPU cpu(.clock(clock),.init(init),.i_datain(i_datain));

initial begin
clock = 0;
init = 0;

//Test for lw, sw, Forwarding & Stalling

i_datain[319:288]={6'b001000,`gr0,`gr1,16'b1};                  //addi  gr0 + 1 -> gr1 
i_datain[287:256]={6'b001000,`gr0,`gr2,16'b10};                 //addi  gr0 + 2 -> gr2 
i_datain[255:224]={6'b000000,`gr1,`gr2,`gr4,5'b0,6'b100000};    //add   gr1 + gr2 -> gr4     (Should forward from MEM and WB)
i_datain[223:192]={6'b101011,`gr0,`gr4,16'b0};                  //sw    gr4 -> d_mem[gr0]   
i_datain[191:160]={6'b100011,`gr0,`gr5,16'b0};                  //lw    d_mem[gr0] -> gr5    
i_datain[159:128]={6'b000000,`gr2,`gr5,`gr6,5'b0,6'b100000};    //add   gr2 + gr5 -> gr6     (Should stall)


//Test for Forwarding, jr
/*
i_datain[319:288]={6'b001000,`gr0,`gr5,16'b1};                  //addi  gr0 + 1 -> gr5
i_datain[287:256]={6'b001000,`gr0,`gr5,16'b10};                 //addi  gr0 + 2 -> gr5
i_datain[255:224]={6'b001000,`gr5,`gr6,16'b0};                  //addi  gr5 + 0 -> gr6      (Should forward from MEM, rather than WB)
i_datain[223:192]={6'b000000,`gr0,`gr0,`gr0,5'b0,6'b001000};    //jr    gr0                 (Should jump to PC=0)
i_datain[191:160]={6'b001000,`gr0,`gr1,16'b1};                  //addi  gr0 + 1 -> gr1      (Should never be executed)
*/

//Test for Branch Flush
/*
i_datain[319:288]={6'b001000,`gr0,`gr1,16'b100};                 //addi  gr0 + 4 -> gr1
i_datain[287:256]={6'b101011,`gr1,`gr1,16'b100};                 //sw    gr1 -> d_mem[gr1+4] 
i_datain[255:224]={6'b000100,`gr1,`gr1,16'b1111_1111_1111_1110}; //beq   gr1, gr1 -> -2      (Should branch to PC=4)
i_datain[223:192]={6'b000000,`gr1,`gr1,`gr2,5'b0,6'b100000};     //add   gr1 + gr1 -> gr2    (Should never be executed)
i_datain[191:160]={6'b000000,`gr0,`gr1,`gr3,5'b0,6'b100000};     //add   gr0 + gr1 -> gr3    (Should never be executed)
*/

//Test for Jump Flush
/*
i_datain[319:288]={6'b001000,`gr0,`gr1,16'b100};                 //addi  gr0 + 4 -> gr1     
i_datain[287:256]={6'b101011,`gr0,`gr1,16'b0};                   //sw    gr1 -> d_mem[gr0] 
i_datain[255:224]={6'b000010,26'b0};                             //j     0                   (Should jump to PC=0)
i_datain[223:192]={6'b000000,`gr1,`gr1,`gr2,5'b0,6'b100000};     //add   gr1 + gr1 -> gr2    (Should never be executed)
i_datain[191:160]={6'b000000,`gr0,`gr1,`gr3,5'b0,6'b100000};     //add   gr0 + gr1 -> gr3    (Should never be executed)
*/

//Test for Overflow
/*
cpu.gr[1]=32'b1111_1111_1111_1111_1111_1111_1111_1111;            //gr1 = -1
cpu.gr[2]=32'b1000_0000_0000_0000_0000_0000_0000_0000;            //gr2 = -2147483648
i_datain[319:288]={6'b001000,`gr0,`gr3,16'b1};                  //addi  gr0 + 1 -> gr3
i_datain[287:256]={6'b000000,`gr1,`gr2,`gr4,5'b0,6'b100001};    //addu  gr1 + gr2 -> gr4 (no overflow)
i_datain[255:224]={6'b000000,`gr1,`gr2,`gr5,5'b0,6'b100000};    //add   gr1 + gr2 -> gr5 (overflow)
*/

//Test for add, sub, jal
/*
cpu.gr[1]=32'b0000_0000_0000_0000_0000_0000_0000_0010;            //gr1 = 2
cpu.gr[2]=32'b0000_0000_0000_0000_0000_0000_0000_0100;            //gr2 = 4
i_datain[319:288]={6'b000000,`gr1,`gr2,`gr3,5'b0,6'b100000};    //add   gr1 + gr2 -> gr3  
i_datain[287:256]={6'b000000,`gr1,`gr2,`gr4,5'b0,6'b100010};    //sub   gr1 - gr2 -> gr4  
i_datain[255:224]={6'b000011,26'b1};                            //jal   1               (Should link PC+4 and jump to PC=4)  
*/


//Test for and, nor, or, xor, bne
/*
cpu.gr[1]=32'b0000_0000_0000_0000_0000_0000_0000_0001;            //gr1 = 1
cpu.gr[2]=32'b1111_1111_1111_1111_1111_1111_1111_1111;            //gr2 = 2**32-1
i_datain[319:288]={6'b000000,`gr1,`gr2,`gr3,5'b0,6'b100100};    //and   gr1 & gr2 -> gr3
i_datain[287:256]={6'b000000,`gr1,`gr2,`gr4,5'b0,6'b100111};    //nor   gr1 nor gr2 -> gr4
i_datain[255:224]={6'b000000,`gr1,`gr2,`gr5,5'b0,6'b100101};    //or    gr1 | gr2 -> gr5
i_datain[223:192]={6'b000000,`gr1,`gr2,`gr6,5'b0,6'b100110};    //xor   gr1 | gr2 -> gr6
i_datain[191:160]={6'b000101,`gr1,`gr2,16'b1111_1111_1111_1110};//bne   gr1, gr1 -> -2      (Should branch to PC=c)
*/

//Test for andi, ori, xori, slti
/*
cpu.gr[1]=32'b0000_0000_0000_0000_0000_0000_0000_0001;            //gr1 = 1
i_datain[319:288]={6'b001100,`gr1,`gr2,16'b1111111111111111};   //andi   gr1 & 2**16-1 -> gr2
i_datain[287:256]={6'b001101,`gr1,`gr3,16'b1111111111111111};   //ori   gr1 | 2**16-1 -> gr3
i_datain[255:224]={6'b001110,`gr1,`gr4,16'b1111};               //xori   gr1 | 15 -> gr4
i_datain[223:192]={6'b001010,`gr1,`gr5,16'b1111111111111111};   //slti   gr2 < -1 -> gr5
*/

//Test for slt, sltu
/*
cpu.gr[1]=32'b0000_0000_0000_0000_0000_0000_0000_0011;            //gr1 = 3
cpu.gr[2]=32'b0000_0000_0000_0000_0000_0000_0000_0100;            //gr2 = 4
cpu.gr[3]=32'b0000_0000_0000_0000_0000_0000_0000_0111;            //gr3 = 7
i_datain[319:288]={6'b000000,`gr1,`gr2,`gr4,5'b0,6'b101010};    //slt  gr1 < gr2 -> gr4
i_datain[287:256]={6'b000000,`gr3,`gr2,`gr5,5'b0,6'b101010};    //slt  gr3 < gr2 -> gr5
i_datain[255:224]={6'b001011,`gr3,`gr6,16'b1000000000000000};   //sltiu  gr3 < 2**15 -> gr6
*/

//Test for sll, sllv
/*
cpu.gr[1]=32'b0000_0000_0000_0000_0000_0000_0000_0011;            //gr1 = 3
cpu.gr[2]=32'b0000_0000_0000_0000_0000_0000_0000_0001;            //gr2 = 1
i_datain[319:288]={32'b0};                                      //NOP
i_datain[287:256]={6'b000000,`gr0,`gr1,`gr3,5'b1,6'b000000};    //sll  gr1 << 1 -> gr3
i_datain[255:224]={6'b000000,`gr2,`gr1,`gr4,5'b1,6'b000100};    //sllv  gr1 << gr2 -> gr4
*/


//Test for srl, srlv, sra, srav
/*
cpu.gr[1]=32'b1000_0000_0000_0000_0000_0000_0000_0000;            //gr1 = 2**31
cpu.gr[2]=32'b0000_0000_0000_0000_0000_0000_0000_1001;            //gr2 = 9
cpu.gr[3]=32'b0000_0000_0000_0000_0000_0000_0000_0010;            //gr3 = 2
i_datain[319:288]={6'b000000,`gr0,`gr1,`gr4,5'b10101,6'b000010};  //srl  gr1 >> 21 -> gr4
i_datain[287:256]={6'b000000,`gr3,`gr2,`gr5,5'b10101,6'b000110};  //srlv  gr2 >> gr3 -> gr5
i_datain[255:224]={6'b000000,`gr0,`gr1,`gr6,5'b10101,6'b000011};  //sra  gr1 >>> 21 -> gr6
i_datain[223:192]={6'b000000,`gr3,`gr2,`gr7,5'b10101,6'b000111};  //srav  gr2 >>> gr3 -> gr7
*/

#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
#period $display("PC=%h regA=%h regB=%h regC=%h gr0=%h gr1=%h gr2=%h gr3=%h gr4=%h gr5=%h gr6=%h gr7=%h instruction=%b\n",cpu.PCF,cpu.reg_A,cpu.reg_B,cpu.ALUOutE,cpu.gr[0],cpu.gr[1],cpu.gr[2],cpu.gr[3],cpu.gr[4],cpu.gr[5],cpu.gr[6],cpu.gr[7],cpu.InstrD);
$display("gr31=%h\n",cpu.gr[31]);
$display("Data Memory from word 0 to word 3\n");
$display("Word0=%h\n",{cpu.d_mem[0],cpu.d_mem[1],cpu.d_mem[2],cpu.d_mem[3]});
$display("Word1=%h\n",{cpu.d_mem[4],cpu.d_mem[5],cpu.d_mem[6],cpu.d_mem[7]});
$display("Word2=%h\n",{cpu.d_mem[8],cpu.d_mem[9],cpu.d_mem[10],cpu.d_mem[11]});
$display("Word3=%h\n",{cpu.d_mem[12],cpu.d_mem[13],cpu.d_mem[14],cpu.d_mem[15]});
#period $finish;

end

parameter period=10;
always #5clock=~clock;

endmodule