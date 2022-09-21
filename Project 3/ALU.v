//This module connects ALU Control to ALU
module main(i_datain, gr1, gr2, c, zero, negative, overflow, hi, lo);

//Input from testbench
input [31:0] i_datain, gr1, gr2;

//ALU result
output [31:0] c;

//Flags
output zero, negative, overflow;

//hi & lo
output[31:0] hi, lo;

//Instruction Decoding
reg [5:0] opcode, funct;

//shamt
reg unsigned [4:0] shamt;

//Sign-extended immediate
reg [31:0] imm;

//When alusrc = 0, operand2 = gr2
//When alusrc = 1, operand2 = imm
reg [31:0] operand2;

wire [4:0] aluctr;
wire alusrc;

always @(i_datain, gr1, gr2)
begin
    imm = {{16{i_datain[15]}}, i_datain[15:0]}; //Default: Sign-extension to imm
    opcode = i_datain[31:26];
    funct = i_datain[5:0];
    shamt = i_datain[10:6];
end

control_unit ctrl(opcode, funct, aluctr, alusrc);

always @(alusrc, gr2, imm)
begin
    if(alusrc == 1'b0)
    begin  
        operand2 = gr2;
    end
    else
    begin
        operand2 = imm;
    end
end

alu testalu(aluctr, gr1, operand2, shamt, c, zero, negative, overflow, hi, lo);

endmodule


//This module is the ALU part
module alu(aluctr, gr1, operand2, shamt, c, out_zero, out_negative, out_overflow, out_hi, out_lo);

//ALU control
input [4:0] aluctr;

//ALU inputs
//operand2 = gr2 or imm
input [31:0] gr1, operand2;

//shamt
input unsigned[4:0] shamt;

//ALU result
output signed[31:0] c;

//flags
output out_zero, out_negative, out_overflow;

//hi & lo
output [31:0] out_hi, out_lo;

//Registers for operand1, operand2, and result
reg signed [31:0] reg_A, reg_B, reg_C;

//Registers for the flags
reg zero, negative, overflow;

//Register hi and lo.
//They are used for MULT, MULTU, DIV, DIVU
reg [31:0] hi, lo;

//Unsigned registers for MULTU, DIVU, SLTU, SLTIU
reg [31:0] reg_B_u, reg_A_u;

//Zero-extended imm
reg [31:0] imm0;

//Register which is used to dectect overflow
reg bit32;

always @(aluctr, gr1, operand2)

begin

    reg_A = gr1;
    reg_A_u = reg_A;
    reg_B = operand2;
    reg_B_u = reg_B;

    //add, addi
    if(aluctr == 5'b00000)
    begin
        //Overflow Detection
        {bit32, reg_C} = {reg_A[31], reg_A} + {reg_B[31], reg_B};
        overflow = bit32 ^ reg_C[31];
        zero = reg_C ? 0 : 1;
        negative = reg_C[31];
    end

    //addu, addiu
    else if(aluctr == 5'b00001)
    begin
        reg_C = reg_A + reg_B;
        zero = reg_C ? 0 : 1;
        negative = 1'b0;
        overflow = 1'b0;
    end

    //sub
    else if(aluctr == 5'b00010)
    begin
        {bit32, reg_C} = {reg_A[31], reg_A} - {reg_B[31], reg_B};
        overflow = bit32 ^ reg_C[31];
        zero = reg_C ? 0 : 1;
        negative = reg_C[31];
    end

    //subu
    else if(aluctr == 5'b00011)
    begin
        reg_C = reg_A - reg_B;
        zero = reg_C ? 0 : 1;
        negative = 1'b0;
        overflow = 1'b0;
    end

    //mult
    else if(aluctr == 5'b00100)
    begin
        {hi, lo} = reg_A * reg_B;
        overflow = 1'b0;
        zero = {hi, lo} ? 0 : 1;
        negative = hi[31];
    end

    //multu
    else if(aluctr == 5'b00101)
    begin
        {hi, lo} = reg_A_u * reg_B_u;
        overflow = 1'b0;
        zero = {hi, lo} ? 0 : 1;
        negative = 1'b0;
    end

    //div
    else if(aluctr == 5'b00110)
    begin
        if(reg_A == 32'h8000_0000 & reg_B == -1)
            overflow = 1;
        else
            overflow = 0;
        lo = reg_A / reg_B;
        hi = reg_A % reg_B;
        zero = lo ? 0 : 1;
        negative = lo[31];
    end

    //divu
    else if(aluctr == 5'b00111)
    begin
        lo = reg_A_u / reg_B_u;
        hi = reg_A_u % reg_B_u;
        overflow = 1'b0;
        zero = lo ? 0 : 1;
        negative = 1'b0;
    end

    //and
    else if(aluctr == 5'b01000)
    begin
        reg_C = reg_A & reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //nor
    else if(aluctr == 5'b01001)
    begin
        reg_C = ~ (reg_A | reg_B);
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //or
    else if(aluctr == 5'b01010)
    begin
        reg_C = reg_A | reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //xor
    else if(aluctr == 5'b01011)
    begin
        reg_C = reg_A ^ reg_B;      
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //slt, slti
    else if(aluctr == 5'b01100)
    begin
        reg_C = reg_A < reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sltu, sltiu
    else if(aluctr == 5'b01101)
    begin
        reg_C = reg_A_u < reg_B_u;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sll
    else if(aluctr == 5'b01110)
    begin
        reg_C = reg_A << shamt;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sllv
    else if(aluctr == 5'b01111)
    begin
        reg_C = reg_A << reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //srl
    else if(aluctr == 5'b10000)
    begin
        reg_C = reg_A >> shamt;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end
    
    //srlv
    else if(aluctr == 5'b10001)
    begin
        reg_C = reg_A >> reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sra
    else if(aluctr == 5'b10010)
    begin
        reg_C = reg_A >>> shamt;
        overflow = 1'b0;
        negative = reg_C[31];
        zero = reg_C ? 0 : 1;
    end

    //srav
    else if(aluctr == 5'b10011)
    begin
        reg_C = reg_A >>> reg_B;
        overflow = 1'b0;
        negative = reg_C[31];
        zero = reg_C ? 0 : 1;
    end

    //andi
    else if(aluctr == 5'b10100)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A & imm0;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //ori
    else if(aluctr == 5'b10101)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A | imm0;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //xori
    else if(aluctr == 5'b10110)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A ^ imm0;      
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

end

assign c = reg_C[31:0];
assign out_hi = hi;
assign out_lo = lo;
assign out_zero = zero;
assign out_negative = negative;
assign out_overflow = overflow;

endmodule


//This module generates the ALU control signals
module control_unit(opcode, funct, out_aluctr, out_alusrc);

//Input from the main module
input [5:0] opcode, funct;

//Output to the ALU
output [4:0] out_aluctr;
output out_alusrc;

/*
aluctr is made up
It starts from 0b0_0000
    0: add / addi / lw / sw
    1: addu / addiu
    2: sub / beq / bne
    3: subu
    4: mult
    5: multu
    6: div
    7: divu
    8: and
    9: nor
    10: or
    11: xor
    12: slt / slti
    13: sltu / sltiu
    14: sll 
    15: sllv
    16: srl 
    17: srlv
    18: sra 
    19: srav
    20: andi
    21: ori
    22: xori
*/
reg [5:0] aluctr;

//alusrc = 0 : from register
//alusrc = 1 : from extended immediate
reg alusrc;

always @(opcode, funct) 
begin

    //R-type Instructions
    if (opcode == 6'b00_0000) 
    begin
        alusrc = 1'b0;
        //ADD
        if(funct == 6'b10_0000)
        begin
            aluctr = 5'b00000;
        end   

        //ADDU
        else if(funct == 6'b10_0001)
        begin
            aluctr = 5'b00001;
        end

        //SUB
        else if(funct == 6'b10_0010)
        begin
            aluctr = 5'b00010;
        end

        //SUBU
        else if(funct == 6'b10_0011)
        begin
            aluctr = 5'b00011;
        end

        //MULT
        else if(funct == 6'b01_1000)
        begin
            aluctr = 5'b00100;
        end

        //MULTU
        else if(funct == 6'b01_1001)
        begin
            aluctr = 5'b00101;
        end

        //DIV
        else if(funct == 6'b01_1010)
        begin
            aluctr = 5'b00110;
        end

        //DIVU
        else if(funct == 6'b01_1011)
        begin
            aluctr = 5'b00111;
        end

        //AND
        else if(funct == 6'b10_0100)
        begin
            aluctr = 5'b01000;
        end

        //NOR
        else if(funct == 6'b10_0111)
        begin
            aluctr = 5'b01001;
        end

        //OR
        else if(funct == 6'b10_0101)
        begin
            aluctr = 5'b01010;
        end

        //XOR
        else if(funct == 6'b10_0110)
        begin
            aluctr = 5'b01011;
        end

        //SLT
        else if(funct == 6'b10_1010)
        begin
            aluctr = 5'b01100;
        end

        //SLTU
        else if(funct == 6'b10_1011)
        begin
            aluctr = 5'b01101;
        end

        //SLL
        else if(funct == 6'b00_0000)
        begin
            aluctr = 5'b01110;
        end

        //SLLV
        else if(funct == 6'b00_0100)
        begin
            aluctr = 5'b01111;
        end

        //SRL
        else if(funct == 6'b00_0010)
        begin
            aluctr = 5'b10000;
        end
        
        //SRLV
        else if(funct == 6'b00_0110)
        begin
            aluctr = 5'b10001;
        end

        //SRA
        else if(funct == 6'b00_0011)
        begin
            aluctr = 5'b10010;
        end

        //SRAV
        else if(funct == 6'b00_0111)
        begin
            aluctr = 5'b10011;
        end

    end

    //I-type Instructions
    else
    begin

        //BEQ, BNE
        if(opcode == 6'b00_0100 || opcode == 6'b00_0101)
        begin
            alusrc = 1'b0;
            aluctr = 5'b00010;
        end

        else
        begin

            alusrc = 1'b1;

            //ADDI
            if(opcode == 6'b00_1000)
            begin
                aluctr = 5'b00000;
            end
            
            //ADDIU
            else if(opcode == 6'b00_1001)
            begin
                aluctr = 5'b00001;
            end

            //ANDI
            else if(opcode == 6'b00_1100)
            begin
                aluctr = 5'b10100;
            end

            //ORI
            else if(opcode == 6'b00_1101)
            begin
                aluctr = 5'b10101;
            end

            //XORI
            else if(opcode == 6'b00_1110)
            begin
                aluctr = 5'b10110;
            end

            //SLTI
            else if(opcode == 6'b00_1010)
            begin
                aluctr = 5'b01100;
            end

            //SLTIU
            else if(opcode == 6'b00_1011)
            begin
                aluctr = 5'b01101;
            end

            //LW, SW
            else if(opcode == 6'b10_0011 || opcode == 6'b10_1011)
            begin
                aluctr = 5'b00000;
            end
        
        end

    end

end

assign out_aluctr = aluctr;
assign out_alusrc = alusrc;

endmodule