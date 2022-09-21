#include <iostream>
#include <stdio.h>
#include <memory.h>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <sstream>
#include <math.h>
#include "MIPS.h"
#include "data.h"
#include "main.h"
#include "ins.h"
using namespace std;

//Constants
extern uint32_t SIM_BASE_ADDR;
uint32_t SIM_BASE_ADDR = 0x00400000;
static unsigned int SIM_MEM_SIZE = 6 * 1024 * 1024;
static unsigned int REG_SIZE = 4;
static unsigned int TEXT_RESERVATION = 1 * 1024 * 1024;

//Registers
vector<int32_t *> regs;
int32_t * hi;
int32_t * lo;

//PC
uint32_t pc;

//Top of the dynamic data
uint32_t dynamic_data;

//Base address (real, 64-bit)
uint8_t * base_addr;

//Signal for load linked
bool LLbit;

//Function map
typedef void (*rFunc)(int32_t *, int32_t *, int32_t *, int);
typedef void (*iFunc)(int32_t *, int32_t *, int);
typedef void (*jFunc)(uint32_t);
static map<int,rFunc> rFuncMap;
static map<int,iFunc> iFuncMap;
static map<int,jFunc> jFuncMap;

//The vector to store fstream*, which correspond to files opened by SYSCALL 13-16
vector<fstream*> fileDescriptor;

/*
 * The main function
 * 0. Call the MIPS assembler
 * 1. Simulate the memory
 * 2. Simulate the registers
 * 3. Maintain a PC
 * 4. Maintain a code section
 * 5. Maintain a data section
 * 6. Maintain a stack section
 * 7. Simulate the execution
 */
int main(int argc, char** argv){
    if (argc != 3){
        cout << "Error: File name needed." << endl;
        exit(-1);
    }

    //First of all, use MIPS assembler to assemble the code
    MIPS(argc, argv);
    ifstream source(argv[1]);
    ifstream input(argv[2]);

    //Build the function map
    buildFuncMap();

    //Allocate 6MB for memory simulation
    base_addr = (uint8_t *) malloc(SIM_MEM_SIZE);

    //The address of the bottom of the stack = the address of the top of the memory
    uint32_t stack_bottom = SIM_BASE_ADDR + SIM_MEM_SIZE;

    //Allocate 4 bytes for each register
    for (unsigned int i = 0; i < 32; i++){
        regs.push_back((int32_t *) malloc(REG_SIZE));
    }
    hi = (int32_t *)malloc(REG_SIZE);
    lo = (int32_t *)malloc(REG_SIZE);

    //Initialize the LLbit
    LLbit = 0;

    //Load the machine code to the memory
    int line_number = loadText(base_addr, input);
    input.close();

    //The actual end of the text section
    uint32_t text_end = SIM_BASE_ADDR + line_number * 4;

    //The address of the static data section = base address + text reservation (1MB)
    uint32_t * data_addr = (uint32_t *) (base_addr + TEXT_RESERVATION);
    uint32_t static_bottom = SIM_BASE_ADDR + TEXT_RESERVATION;

    //Load the data to the memory
    uint32_t * data_ptr = loadData(source, data_addr);
    source.close();
    uint32_t data_end = SIM_BASE_ADDR + (((uint8_t *) data_ptr) - base_addr);
    dynamic_data = data_end;

    //Initialize registers
    initRegs(stack_bottom, static_bottom);

    //Set PC
    pc = SIM_BASE_ADDR;

    //Simulate the machine cycle
    while(pc < text_end){
        if (dynamic_data >= (uint32_t) *regs[29]){
            cout << "Stack overflow" << endl;
            exit(EXIT_FAILURE);
        }
        fetchIns();
    }

    //Terminate the program
    cout << "PC reaches the end of the text section." << endl;
    terminate(1);

    return 0;
}

/*
 * Terminate the program
 * Meanwhile output the current memory address
 */
void terminate(bool success){
    //Free the memory and the registers
    free(base_addr);
    for (int32_t * reg:regs){
        free(reg);
    }
    free(hi);
    free(lo);

    //Free the fstreams
    for (fstream * fs_ptr : fileDescriptor){
        free(fs_ptr);
    }

    cout << "Simulation terminated at address: ";
    cout << hex << pc-4 << endl;
    if (success) exit(EXIT_SUCCESS);
    else exit(EXIT_FAILURE);
}

/*
 * Initialize the registers
 */
void initRegs(uint32_t stack_bottom, uint32_t static_bottom){
    *regs[0] = 0;                        //$zero = 0
    for (int i = 1; i <= 27; i++) *regs[i] = 0;
    *regs[28] = static_bottom;           //$gp = static bottom
    *regs[29] = stack_bottom;            //$sp = static top ( = static bottom, since the stack is empty now)
    *regs[30] = stack_bottom;            //$fp = static bottom
    *regs[31] = SIM_BASE_ADDR;
    *hi = 0;
    *lo = 0;
}

/*
 * Build the function map
 */
void buildFuncMap(){
    rFuncMap.insert(make_pair(ADD.function, r_ADD));
    rFuncMap.insert(make_pair(ADDU.function, r_ADDU));
    rFuncMap.insert(make_pair(AND.function, r_AND));
    rFuncMap.insert(make_pair(DIV.function, r_DIV));
    rFuncMap.insert(make_pair(DIVU.function, r_DIVU));
    rFuncMap.insert(make_pair(MULT.function, r_MULT));
    rFuncMap.insert(make_pair(MULTU.function, r_MULTU));
    rFuncMap.insert(make_pair(NOR.function, r_NOR));
    rFuncMap.insert(make_pair(OR.function, r_OR));
    rFuncMap.insert(make_pair(SLL.function, r_SLL));
    rFuncMap.insert(make_pair(SLLV.function, r_SLLV));
    rFuncMap.insert(make_pair(SRA.function, r_SRA));
    rFuncMap.insert(make_pair(SRAV.function, r_SRAV));
    rFuncMap.insert(make_pair(SRL.function, r_SRL));
    rFuncMap.insert(make_pair(SRLV.function, r_SRLV));
    rFuncMap.insert(make_pair(SUB.function, r_SUB));
    rFuncMap.insert(make_pair(SUBU.function, r_SUBU));
    rFuncMap.insert(make_pair(XOR.function, r_XOR));
    rFuncMap.insert(make_pair(SLT.function, r_SLT));
    rFuncMap.insert(make_pair(SLTU.function, r_SLTU));
    rFuncMap.insert(make_pair(JR.function, r_JR));
    rFuncMap.insert(make_pair(JALR.function, r_JALR));
    rFuncMap.insert(make_pair(TEQ.function, r_TEQ));
    rFuncMap.insert(make_pair(TNE.function, r_TNE));
    rFuncMap.insert(make_pair(TGE.function, r_TGE));
    rFuncMap.insert(make_pair(TGEU.function, r_TGEU));
    rFuncMap.insert(make_pair(TLT.function, r_TLT));
    rFuncMap.insert(make_pair(TLTU.function, r_TLTU));
    rFuncMap.insert(make_pair(MFHI.function, r_MFHI));
    rFuncMap.insert(make_pair(MFLO.function, r_MFLO));
    rFuncMap.insert(make_pair(MTHI.function, r_MTHI));
    rFuncMap.insert(make_pair(MTLO.function, r_MTLO));
    rFuncMap.insert(make_pair(SYSCALL.function, r_SYSCALL));

    iFuncMap.insert(make_pair(ADDI.opcode, i_ADDI));
    iFuncMap.insert(make_pair(ADDIU.opcode, i_ADDIU));
    iFuncMap.insert(make_pair(ANDI.opcode, i_ANDI));
    iFuncMap.insert(make_pair(ORI.opcode, i_ORI));
    iFuncMap.insert(make_pair(XORI.opcode, i_XORI));
    iFuncMap.insert(make_pair(LUI.opcode, i_LUI));
    iFuncMap.insert(make_pair(SLTI.opcode, i_SLTI));
    iFuncMap.insert(make_pair(SLTIU.opcode, i_SLTIU));
    iFuncMap.insert(make_pair(BEQ.opcode, i_BEQ));
    iFuncMap.insert(make_pair(BGEZ.opcode, i_BGEZ));
    iFuncMap.insert(make_pair(BGEZAL.opcode, i_BGEZAL));
    iFuncMap.insert(make_pair(BGTZ.opcode, i_BGTZ));
    iFuncMap.insert(make_pair(BLEZ.opcode, i_BLEZ));
    iFuncMap.insert(make_pair(BLTZAL.opcode, i_BLTZAL));
    iFuncMap.insert(make_pair(BLTZ.opcode, i_BLTZ));
    iFuncMap.insert(make_pair(BNE.opcode, i_BNE));
    iFuncMap.insert(make_pair(TEQI.opcode, i_TEQI));
    iFuncMap.insert(make_pair(TNEI.opcode, i_TNEI));
    iFuncMap.insert(make_pair(TGEI.opcode, i_TGEI));
    iFuncMap.insert(make_pair(TGEIU.opcode, i_TGEIU));
    iFuncMap.insert(make_pair(TLTI.opcode, i_TLTI));
    iFuncMap.insert(make_pair(TLTIU.opcode, i_TLTIU));
    iFuncMap.insert(make_pair(LB.opcode, i_LB));
    iFuncMap.insert(make_pair(LBU.opcode, i_LBU));
    iFuncMap.insert(make_pair(LH.opcode, i_LH));
    iFuncMap.insert(make_pair(LHU.opcode, i_LHU));
    iFuncMap.insert(make_pair(LW.opcode, i_LW));
    iFuncMap.insert(make_pair(LWL.opcode, i_LWL));
    iFuncMap.insert(make_pair(LWR.opcode, i_LWR));
    iFuncMap.insert(make_pair(LL.opcode, i_LL));
    iFuncMap.insert(make_pair(SB.opcode, i_SB));
    iFuncMap.insert(make_pair(SH.opcode, i_SH));
    iFuncMap.insert(make_pair(SW.opcode, i_SW));
    iFuncMap.insert(make_pair(SWL.opcode, i_SWL));
    iFuncMap.insert(make_pair(SWR.opcode, i_SWR));
    iFuncMap.insert(make_pair(SC.opcode, i_SC));

    jFuncMap.insert(make_pair(J.opcode, j_J));
    jFuncMap.insert(make_pair(JAL.opcode, j_JAL));
}

/*
 * Convert a string containing machine code to a 32-bit unsigned integer.
 * eg. "00111100000001000000000001010000"
 */
uint32_t stringToUnsigned(string code){
    int len = code.length();
    uint32_t value = 0;
    for (int i = 0; i < len; i++){
        if (code[i] == '1'){
            value += pow(2, (len-1-i));
        }
    }
    return value;
}

/*
 * Load the machine code to the text section of the simulated memory
 */
int loadText(uint8_t * base_addr, istream & input){
    string line;
    uint32_t * text_ptr = (uint32_t *) base_addr;
    int line_number = 0;
    while (getline(input, line)){
        line_number++;
        if (line.length() != 32){
            cout << "Invalid MIPS code at line " << line_number << endl;
            return 0;
        }
        uint32_t value = stringToUnsigned(line);
        * text_ptr = value;
        text_ptr += 1;
    }
    return line_number;
}

/*
 * Convert a 32-bit unsigned integer to a string
 */
string binaryToString(uint32_t value){
    string str;
    for (int i = 0; i < 32; i++){
        int mask = (1 << 31);
        if (value & mask){
            str += '1';
        }
        else {
            str += '0';
        }
        value <<= 1;
    }
    return str;
}

/*
 * Fetch one instruction from the memory pointed by PC
 * Then increase PC by 4
 */
void fetchIns(){
    //To avoid infinite loop
    static int line = 0;
    if (line == 100000) {
        cout << "Simulated over 100000 instructions" << endl;
        exit(EXIT_FAILURE);
    }

    int offset = pc - SIM_BASE_ADDR;
    uint32_t value = *(uint32_t *) (base_addr+offset);
    string ins = binaryToString(value);
    pc += 4;
    decodeIns(ins);
}

/*
 * Decode the instruction
 * Sort the instructions into R-type, I-type, and J-type
 */
void decodeIns(string ins){
    unsigned int opcode = stringToUnsigned(ins.substr(0,6));
    if (opcode == 0) {
        rType(ins);
    }
    else if (opcode == 2 || opcode == 3){
        jType(ins);
    }
    else{
        iType(ins);
    }
}

/*
 * Convert a string to a signed integer
 * Note that code[0] corresponds to the sign bit
 */
int stringToSigned(string code){
    int len = code.length();
    int value = 0;
    for (int i = 1; i < len; i++){
        if (code[i] == '1'){
            value += pow(2, (len-1-i));
        }
    }
    if (code[0] == '1'){
        value = - ((1<<(len-1)) - value);
    }
    return value;
}

/*
 * Decode an I-type instruction and call the corresponding function
 */
void iType(string ins){
    int opcode = stringToUnsigned(ins.substr(0,6));
    int32_t * rs = regs[stringToUnsigned(ins.substr(6,5))];
    int32_t * rt = regs[stringToUnsigned(ins.substr(11,5))];
    int imm = stringToSigned(ins.substr(16,16)); //imm or offset
    iFuncMap[opcode](rs, rt, imm);
}

/*
 * Decode an R-type instruction and call the corresponding function
 */
void rType(string ins){
    int funct = stringToUnsigned(ins.substr(26,6));
    int32_t * rs = regs[stringToUnsigned(ins.substr(6,5))];
    int32_t * rt = regs[stringToUnsigned(ins.substr(11,5))];
    int32_t * rd = regs[stringToUnsigned(ins.substr(16,5))];
    int shamt = stringToSigned(ins.substr(21,5));
    rFuncMap[funct](rs, rt, rd, shamt);
}

/*
 * Decode a J-type instruction and call the corresponding function
 */
void jType(string ins){
    int opcode = stringToUnsigned(ins.substr(0,6));
    uint32_t target = stringToUnsigned(ins.substr(6,26));
    jFuncMap[opcode](target);
}


