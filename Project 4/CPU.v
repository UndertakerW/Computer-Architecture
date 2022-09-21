`timescale 1ns / 1ps

//The main CPU module
    //Features:
    //1. Pipelined
    //2. Hazard Handling
module CPU (clock, init, i_datain);

    //The clock signal
    input wire clock;

    //The initialization signal: set the PC to be 0
    input wire init;

    //The instructions
    input [319:0] i_datain;

    //General Registers
    reg [31:0] gr[31:0];

    //Instruction Memory
    reg [7:0] i_mem[1023:0];

    //Data Memory
    reg [7:0] d_mem[1023:0];

    //Instruction Fetch Stage
    //Pipeline Registers
    reg [31:0] PC_prime;
    reg [31:0] PCF;
    reg [31:0] PCPlus4F;
    reg [31:0] InstrF;

    //Instruction Decode Stage
    //Pipeline Registers
    reg [31:0] InstrD;
    reg [31:0] PCPlus4D;
    //Wires
    wire RegWriteD;
    wire MemtoRegD;
    wire MemWriteD;
    wire [1:0] BranchD;
    wire [4:0] ALUControlD;
    wire ALUSrcD;
    wire RegDstD;
    wire [1:0] JumpD;

    //Execution Stage
    //Pipeline Registers
    reg RegWriteE;
    reg MemtoRegE;
    reg MemWriteE;
    reg [1:0] BranchE;
    reg [4:0] ALUControlE;
    reg ALUSrcE;
    reg RegDstE;
    reg [31:0] SrcAE; //gr1
    reg [31:0] SrcBE; //gr2
    reg [4:0] RsE;
    reg [4:0] RtE;
    reg [4:0] RdE;
    reg [4:0] ShamtE;
    reg signed [31:0] SignImmE;
    reg [31:0] PCPlus4E;
    reg [1:0] JumpE;
    reg [27:0] TargetE;
    //Temporary Registers
    reg [4:0] WriteRegE;
    reg [1:0] ForwardA;
    reg [1:0] ForwardB;
    reg [31:0] reg_A;
    reg [31:0] reg_B;
    reg [31:0] WriteDataE;
    reg LoadStall;
    //Wires
    wire [31:0] ALUOutE;
    wire ZeroE;
    wire NegativeE;
    wire OverflowE;

    //Memory Stage
    //Pipeline Registers
    reg RegWriteM;
    reg MemtoRegM;
    reg MemWriteM;
    reg [1:0] BranchM;
    reg ZeroM;
    reg [31:0] ALUOutM;
    reg [31:0] WriteDataM;
    reg [4:0] WriteRegM;
    reg [31:0] PCBranchM;
    //Temporary Registers
    reg PCSrcM;

    //Write Back Stage
    //Pipeline Registers
    reg RegWriteW;
    reg MemtoRegW;
    reg [31:0] ALUOutW;
    reg [31:0] ReadDataW;
    reg [4:0] WriteRegW;
    //Temporary Registers
    reg [31:0] ResultW;

    

    //Initialization: 
    //1.Set the PC to be 0
    //2.Put the instructions into the instruction memory
    always @(init)
    begin
        PCF = 32'h0000_0000;
        gr[0] = 32'h0000_0000;

        //Note: Here, I input 10 instructions each time.
        {i_mem[0], i_mem[1], i_mem[2], i_mem[3]} = i_datain[319:288];
        {i_mem[4], i_mem[5], i_mem[6], i_mem[7]} = i_datain[287:256];
        {i_mem[8], i_mem[9], i_mem[10], i_mem[11]} = i_datain[255:224];
        {i_mem[12], i_mem[13], i_mem[14], i_mem[15]} = i_datain[223:192];
        {i_mem[16], i_mem[17], i_mem[18], i_mem[19]} = i_datain[191:160];
        {i_mem[20], i_mem[21], i_mem[22], i_mem[23]} = i_datain[159:128];
        {i_mem[24], i_mem[25], i_mem[26], i_mem[27]} = i_datain[127:96];
        {i_mem[28], i_mem[29], i_mem[30], i_mem[31]} = i_datain[95:64];
        {i_mem[32], i_mem[33], i_mem[34], i_mem[35]} = i_datain[63:32];
        {i_mem[36], i_mem[37], i_mem[38], i_mem[39]} = i_datain[31:0];
    end

//Instruction Fetch

always @(PCF)
begin
    PCPlus4F = PCF + 3'b100;
end

always @(PCSrcM, JumpE, PCBranchM, PCPlus4F, TargetE, reg_A)
begin
    if (PCSrcM == 1'b1)
    begin
        PC_prime = PCBranchM;
    end
    //j or jal
    else if (JumpE == 2'b01 || JumpE == 2'b11)
    begin
        PC_prime = {PCPlus4F[31:28], TargetE};
    end
    //jr: support forwarding from MEM or WB
    else if (JumpE == 2'b10)
    begin
        PC_prime = reg_A;
    end

    else
    begin
        PC_prime = PCPlus4F;
    end
end

always @(posedge clock)
begin
    //If the pipeline is stalling, do not update PC
    if (LoadStall == 1'b1)
    begin
    end

    else
    begin
        PCF <= PC_prime;

        InstrF = {i_mem[PCF], i_mem[PCF+1], i_mem[PCF+2], i_mem[PCF+3]};

        PCPlus4D <= PCPlus4F;

        //If branch or jump, the next InstrD will be 32'h0000_0000 (NOP).
        if (PCSrcM == 1'b1 || JumpE == 2'b01 || JumpE == 2'b10 || JumpE == 2'b11)
        begin
            InstrD <= 32'h0000_0000;
        end
        
        else
        begin
            InstrD <= InstrF;
        end
    end
end



//Instruction Decode

//The control unit generates control signals
control_unit cu(InstrD[31:26], InstrD[5:0], RegWriteD, MemtoRegD, MemWriteD,
    BranchD, ALUControlD, ALUSrcD, RegDstD, JumpD);

always @(posedge clock)
begin

    if (LoadStall == 1'b1)
    begin
        RegWriteE <= 1'b0;
        MemtoRegE <= 1'b0;
        MemWriteE <= 1'b0;
        BranchE <= 2'b00;
        JumpE <= 2'b00;
    end

    else
    begin
        RegWriteE <= RegWriteD;
        MemtoRegE <= MemtoRegD;
        MemWriteE <= MemWriteD;
        BranchE <= BranchD;
        JumpE <= JumpD;
    end

    ALUControlE <= ALUControlD;
    ALUSrcE <= ALUSrcD;
    RegDstE <= RegDstD;

    TargetE <= {InstrD[25:0], 2'b00};

    SrcAE <= gr[InstrD[25:21]];
    SrcBE <= gr[InstrD[20:16]];
    ShamtE <= InstrD[10:6];

    RsE <= InstrD[25:21];
    RtE <= InstrD[20:16];
    RdE <= InstrD[15:11];

    SignImmE <= {{16{InstrD[15]}}, InstrD[15:0]};
    PCPlus4E <= PCPlus4D;
end



//Execution

//Prepare the ALU sources
always @(ALUSrcE, SrcAE, SrcBE, SignImmE, ForwardA, ForwardB, ALUOutM, ResultW)
begin
    if (ForwardA == 2'b10)
    begin
        reg_A = ALUOutM;
    end

    else if (ForwardA == 2'b01)
    begin
        reg_A = ResultW;
    end

    else
    begin
        reg_A = SrcAE;
    end

    if (ALUSrcE == 1'b0)
    begin
        if (ForwardB == 2'b10)
        begin
            reg_B = ALUOutM;
            WriteDataE = ALUOutM;
        end

        else if (ForwardB == 2'b01)
        begin
            reg_B = ResultW;
            WriteDataE = ResultW;
        end

        else
        begin
            reg_B = SrcBE;
            WriteDataE = SrcBE;
        end
    end

    else
    begin
        reg_B = SignImmE;

        if (ForwardB == 2'b10)
        begin
            WriteDataE = ALUOutM;
        end

        else if (ForwardB == 2'b01)
        begin
            WriteDataE = ResultW;
        end

        else
        begin
            WriteDataE = SrcBE;
        end
    end
end

//Select the register destination
always @(RegDstE, RtE, RdE)
begin
    if (RegDstE == 1'b0)
    begin
        WriteRegE = RtE;
    end
    else
    begin
        WriteRegE = RdE;
    end
end



//Hazard Detection

//Forwarding
always @(RegWriteM, WriteRegM, RsE, RtE, RegWriteW, WriteRegW)
begin
    if (RegWriteM == 1'b1 && WriteRegM != 5'b0 && WriteRegM == RsE)
    begin
        ForwardA = 2'b10;
    end
    else if (RegWriteW == 1'b1 && WriteRegW != 5'b0 && WriteRegW == RsE)
    begin
        ForwardA = 2'b01;
    end
    else
    begin
        ForwardA = 2'b00;
    end

    if (RegWriteM == 1'b1 && WriteRegM != 5'b0 && WriteRegM == RtE)
    begin
        ForwardB = 2'b10;
    end
    else if (RegWriteW == 1'b1 && WriteRegW != 5'b0 && WriteRegW == RtE)
    begin
        ForwardB = 2'b01;
    end
    else
    begin
        ForwardB = 2'b00;
    end
end

//Stall for LW
always @(MemtoRegE, RtE, InstrD)
begin
    if (MemtoRegE == 1'b1)
    begin
        if (RtE == InstrD[25:21] || RtE == InstrD[20:16])
        begin
            LoadStall = 1'b1;
        end
    end
    else
    begin
        LoadStall = 1'b0;
    end
end


//Jump Flush
always @(JumpE)
begin
    if (JumpE == 2'b01 || JumpE == 2'b10 || JumpE == 2'b11)
    begin
        InstrD = 32'h0000_0000;
    end
end

//Jump and Link
always @(JumpE)
begin
    if (JumpE == 2'b11)
    begin
        gr[5'b11111] = PCPlus4E;
    end
end

//The ALU
alu testalu(ALUControlE, reg_A, reg_B, ShamtE, ALUOutE, ZeroE, NegativeE, OverflowE);

//Overflow Exception
always @(posedge clock)
begin
    if (OverflowE == 1'b1)
        $display("Overflow occurs at address: %h\n", (PCPlus4E-3'b100));
end

always @(posedge clock)
begin
    if (OverflowE == 1'b1)
    begin
        RegWriteM <= 1'b0;
        MemtoRegM <= 1'b0;
        MemWriteM <= 1'b0;
        BranchM <= 1'b0;
    end
    else
    begin
        RegWriteM <= RegWriteE;
        MemtoRegM <= MemtoRegE;
        MemWriteM <= MemWriteE;
        BranchM <= BranchE;
    end
    ZeroM <= ZeroE;
    ALUOutM <= ALUOutE;
    WriteDataM <= WriteDataE;
    WriteRegM <= WriteRegE;
    PCBranchM <= ((SignImmE << 2) + PCPlus4E);
end



//Memory

//Branch Flush
always @(PCSrcM)
begin
    if (PCSrcM == 1'b1)
    begin
        RegWriteE = 1'b0;
        MemtoRegE = 1'b0;
        MemWriteE = 1'b0;
        BranchE = 2'b00;
        JumpE = 2'b00;
        InstrD = 32'h0000_0000;
    end
end

//Branch judgement
always @(BranchM, ZeroM)
begin
    if ((BranchM == 2'b01 && ZeroM == 1'b1) || (BranchM == 2'b10 && ZeroM == 1'b0))
    begin
        PCSrcM = 1'b1;
    end
    else
    begin
        PCSrcM = 1'b0;
    end
end

always @(posedge clock)
begin
    if (MemWriteM == 1'b1)
    begin
        d_mem[ALUOutM] = WriteDataM[31:24];
        d_mem[ALUOutM+1] = WriteDataM[23:16];
        d_mem[ALUOutM+2] = WriteDataM[15:8];
        d_mem[ALUOutM+3] = WriteDataM[7:0];
    end
    RegWriteW <= RegWriteM;
    MemtoRegW <= MemtoRegM;
    ALUOutW <= ALUOutM;
    ReadDataW <= {d_mem[ALUOutM],d_mem[ALUOutM],d_mem[ALUOutM+2],d_mem[ALUOutM+3]};;
    WriteRegW <= WriteRegM;
end



//Write Back

//Select the result to write back
always @(MemtoRegW, ALUOutW, ReadDataW)
begin
    if (MemtoRegW == 1'b0)
    begin
        ResultW = ALUOutW;
    end
    else
    begin
        ResultW = ReadDataW;
    end
end

//Write back occurs at negative clock edge
always @(negedge clock)
begin
    if (RegWriteW == 1'b1)
    begin
        gr[WriteRegW] <= ResultW;
    end
end

endmodule



//The ALU module
module alu(aluctr, gr1, operand2, shamt, c, out_zero, out_negative, out_overflow);

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

//Registers for operand1, operand2, and result
reg signed [31:0] reg_A, reg_B, reg_C;

//Registers for the flags
reg zero, negative, overflow;

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

    //addu, addiu, j, jr, jal
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

    //and
    else if(aluctr == 5'b00100)
    begin
        reg_C = reg_A & reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //nor
    else if(aluctr == 5'b00101)
    begin
        reg_C = ~ (reg_A | reg_B);
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //or
    else if(aluctr == 5'b00110)
    begin
        reg_C = reg_A | reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //xor
    else if(aluctr == 5'b00111)
    begin
        reg_C = reg_A ^ reg_B;      
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //slt, slti
    else if(aluctr == 5'b01000)
    begin
        reg_C = reg_A < reg_B;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sltu, sltiu
    else if(aluctr == 5'b10010)
    begin
        reg_C = reg_A_u < reg_B_u;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sll
    else if(aluctr == 5'b01001)
    begin
        reg_C = reg_B << shamt;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sllv
    else if(aluctr == 5'b01010)
    begin
        reg_C = reg_B << reg_A;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //srl
    else if(aluctr == 5'b01011)
    begin
        reg_C = reg_B >> shamt;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end
    
    //srlv
    else if(aluctr == 5'b01100)
    begin
        reg_C = reg_B >> reg_A;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //sra
    else if(aluctr == 5'b01101)
    begin
        reg_C = reg_B >>> shamt;
        overflow = 1'b0;
        negative = reg_C[31];
        zero = reg_C ? 0 : 1;
    end

    //srav
    else if(aluctr == 5'b01110)
    begin
        reg_C = reg_B >>> reg_A;
        overflow = 1'b0;
        negative = reg_C[31];
        zero = reg_C ? 0 : 1;
    end

    //andi
    else if(aluctr == 5'b01111)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A & imm0;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //ori
    else if(aluctr == 5'b10000)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A | imm0;
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

    //xori
    else if(aluctr == 5'b10001)
    begin
        imm0 = {{16{1'b0}}, reg_B[15:0]};
        reg_C = reg_A ^ imm0;      
        overflow = 1'b0;
        negative = 1'b0;
        zero = reg_C ? 0 : 1;
    end

end

assign c = reg_C[31:0];
assign out_zero = zero;
assign out_negative = negative;
assign out_overflow = overflow;

endmodule



//The Control Unit module
module control_unit(opcode, funct, out_regwrite, out_memtoreg, out_memwrite,
                    out_branch, out_aluctr, out_alusrc, out_regdst, out_jump);

//Input from ID
input [5:0] opcode, funct;

//Output to the ALU
output [4:0] out_aluctr;
output [1:0] out_branch, out_jump;
output out_regwrite, out_memtoreg, out_memwrite, out_alusrc, out_regdst;

/*
aluctr is made up
It starts from 0b00000
    0: add / addi / lw / sw
    1: addu / addiu / j / jr / jal
    2: sub / beq / bne
    3: subu
    4: and
    5: nor
    6: or
    7: xor
    8: slt / slti
    9: sll 
    10: sllv
    11: srl 
    12: srlv
    13: sra 
    14: srav
    15: andi
    16: ori
    17: xori
    18: sltu / sltiu
*/
reg [4:0] aluctr;

//alusrc = 0 : from register
//alusrc = 1 : from extended immediate
reg alusrc;

reg [1:0] branch, jump;
reg regwrite, memtoreg, memwrite, regdst;

/*
The CPU must support:
1) Data transfer instructions:
- lw, sw
2) Arithmetic instructions:
- add, sub, addu, subu
- addi, addiu
3) Logical instructions:
- and, or, nor, xor
- andi, ori
4) Shifting instructions:
- sll, srl, sra
- sllv, srlv, srav
5) Branch/Jump instructions:
- beq, bne, slt (with comparison so put it here)
- j, jr, ja
*/

always @(opcode, funct) 
begin

    //R-type Instructions

    if (opcode == 6'b00_0000 && funct != 6'b00_1000) 
    begin
        alusrc = 1'b0;
        branch = 2'b00;
        jump = 2'b00;
        regwrite = 1'b1;
        memtoreg = 1'b0;
        memwrite = 1'b0;
        regdst = 1'b1;

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

        //AND
        else if(funct == 6'b10_0100)
        begin
            aluctr = 5'b00100;
        end

        //NOR
        else if(funct == 6'b10_0111)
        begin
            aluctr = 5'b00101;
        end

        //OR
        else if(funct == 6'b10_0101)
        begin
            aluctr = 5'b00110;
        end

        //XOR
        else if(funct == 6'b10_0110)
        begin
            aluctr = 5'b00111;
        end

        //SLT
        else if(funct == 6'b10_1010)
        begin
            aluctr = 5'b01000;
        end

        //SLL
        else if(funct == 6'b00_0000)
        begin
            aluctr = 5'b01001;
        end

        //SLLV
        else if(funct == 6'b00_0100)
        begin
            aluctr = 5'b01010;
        end

        //SRL
        else if(funct == 6'b00_0010)
        begin
            aluctr = 5'b01011;
        end
        
        //SRLV
        else if(funct == 6'b00_0110)
        begin
            aluctr = 5'b01100;
        end

        //SRA
        else if(funct == 6'b00_0011)
        begin
            aluctr = 5'b01101;
        end

        //SRAV
        else if(funct == 6'b00_0111)
        begin
            aluctr = 5'b01110;
        end

        //SLTU
        else if(funct == 6'b10_1011)
        begin
            aluctr = 5'b10010;
        end

    end

    //JR
    else if(opcode == 6'b00_0000 && funct == 6'b00_1000)
    begin
        alusrc = 1'b0;
        branch = 2'b00;
        jump = 2'b10;
        regwrite = 1'b0;
        memtoreg = 1'b0;
        memwrite = 1'b0;
        regdst = 1'b1;
        aluctr = 5'b00001;
    end

    //J-type Instrucions
    //J
    else if(opcode == 6'b00_0010)
    begin
        alusrc = 1'b0;
        branch = 2'b00;
        jump = 2'b01;
        regwrite = 1'b0;
        memtoreg = 1'b0;
        memwrite = 1'b0;
        regdst = 1'b1;
        aluctr = 5'b00001;
    end

    //JAL
    else if(opcode == 6'b00_0011)
    begin
        alusrc = 1'b0;
        branch = 2'b00;
        jump = 2'b11;
        regwrite = 1'b0;
        memtoreg = 1'b0;
        memwrite = 1'b0;
        regdst = 1'b1;
        aluctr = 5'b00001;
    end
    
    //I-type Instructions
    else
    begin

        //BEQ
        if(opcode == 6'b00_0100)
        begin
            alusrc = 1'b0;
            aluctr = 5'b00010;
            branch = 2'b01;
            jump = 2'b00;
            regwrite = 1'b0;
            memtoreg = 1'b0;
            memwrite = 1'b0;
            regdst = 1'b1;
        end

        //BNE
        else if(opcode == 6'b00_0101)
        begin
            alusrc = 1'b0;
            aluctr = 5'b00010;
            branch = 2'b10;
            jump = 2'b00;
            regwrite = 1'b0;
            memtoreg = 1'b0;
            memwrite = 1'b0;
            regdst = 1'b1;
        end

        //LW
        else if(opcode == 6'b10_0011)
        begin
            aluctr = 5'b00000;
            alusrc = 1'b1;
            branch = 2'b00;
            jump = 2'b00;
            regwrite = 1'b1;
            memtoreg = 1'b1;
            memwrite = 1'b0;
            regdst = 1'b0;
        end

        //SW
        else if(opcode == 6'b10_1011)
        begin
            aluctr = 5'b00000;
            alusrc = 1'b1;
            branch = 2'b00;
            jump = 2'b00;
            regwrite = 1'b0;
            memtoreg = 1'b0;
            memwrite = 1'b1;
            regdst = 1'b0;
        end
        
        else
        begin
            alusrc = 1'b1;
            branch = 2'b00;
            jump = 2'b00;
            regwrite = 1'b1;
            memtoreg = 1'b0;
            memwrite = 1'b0;
            regdst = 1'b0;

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
                aluctr = 5'b01111;
            end

            //ORI
            else if(opcode == 6'b00_1101)
            begin
                aluctr = 5'b10000;
            end

            //XORI
            else if(opcode == 6'b00_1110)
            begin
                aluctr = 5'b10001;
            end

            //SLTI
            else if(opcode == 6'b00_1010)
            begin
                aluctr = 5'b01000;
            end

            //SLTIU
            else if(opcode == 6'b00_1011)
            begin
                aluctr = 5'b10010;
            end
        end
    end
end

assign out_aluctr = aluctr;
assign out_alusrc = alusrc;
assign out_branch = branch;
assign out_jump = jump;
assign out_regwrite = regwrite;
assign out_memtoreg = memtoreg;
assign out_memwrite = memwrite;
assign out_regdst = regdst;

endmodule