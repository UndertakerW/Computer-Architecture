#include <cstdlib>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include "ins.h"
#include "main.h"

using namespace std;

/*
 * Overflow detection for ADD,ADDI,SUB
 */
bool overflow(int64_t value){
    if(value < INT_MIN || value > INT_MAX){
        LLbit = 0;
        return true;
    }
    return false;
}

/*
 * Whenever a trap exception is raised, terminate the program
 */
void trap(){
    LLbit = 0;
    cout << "Trap exception" << endl;
    terminate(0);
}

void r_ADD(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    int64_t temp = (int64_t) *rs + (int64_t) *rt;
    if (!overflow(temp)){
        *rd = temp;
    }
    else{
        printf("ADD overflow, rs : %d, rt : %d\n", *rs, *rt);
        terminate(0);
    }
}

void r_ADDU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rs + *rt;
}

void i_ADDI(int32_t * rs, int32_t * rt, int imm){
    int64_t temp = (int64_t) *rs + imm;
    if (!overflow(temp)){
        *rt = temp;
    }
    else{
        printf("ADDI overflow, rs : %d, imm : %d\n", *rs, imm);
        terminate(0);
    }
}

void i_ADDIU(int32_t * rs, int32_t * rt, int imm){
    *rt = *rs + imm;
}

void r_AND(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rs & *rt;
}

void i_ANDI(int32_t * rs, int32_t * rt, int imm){
    //In order to zero-extend the imm,
    //convert imm from int to uint16_t
    *rt = *rs & (uint16_t) imm;
}

void r_DIV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if ((*rs == INT_MIN && *rt == -1) || *rt == 0){
        LLbit = 0;
        printf("DIV overflow, rs : %d, rt : %d\n", *rs, *rt);
        terminate(0);
    }
    else{
        int quo = (*rs) / (*rt);
        int rem = (*rs) - quo * (*rt);
        *lo = quo;
        *hi = rem;
    }
}

void r_DIVU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    int quo = (*rs) / (*rt);
    int rem = (*rs) - quo * (*rt);
    *lo = quo;
    *hi = rem;
}

void r_MULT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    int64_t temp = (*rs) * (*rt);
    if (/* DISABLES CODE */ (1)){
        *hi = temp >> 32;
        *lo = temp;
    }
    else{
        printf("MULT overflow, rs : %d, rt : %d\n", *rs, *rt);
        terminate(0);
    }
}

void r_MULTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    uint64_t temp = ((uint32_t) *rs) * ((uint32_t) *rt);
    if (/* DISABLES CODE */ (1)){
        *hi = temp >> 32;
        *lo = temp;
    }
    else{
        printf("MULTU overflow, rs : %d, rt : %d\n", *rs, *rt);
        terminate(0);
    }
}

void r_NOR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = ~ (*rs | *rt);
}

void r_OR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){   
    *rd = *rs | *rt;
}

void i_ORI(int32_t * rs, int32_t * rt, int imm){
    *rt = *rs | (uint16_t) imm;
}

void r_SLL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rt << shamt;
}

void r_SLLV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rt << *rs;
}

void r_SRA(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rt >> shamt;
}

void r_SRAV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rt >> *rs;
}

void r_SRL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = ((uint32_t) *rt) >> shamt;
}

void r_SRLV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = ((uint32_t) *rt) >> *rs;
}

void r_SUB(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    int64_t temp = (int64_t) *rs - (int64_t) *rt;
    if (!overflow(temp)){
        *rd = temp;
    }
    else{
        printf("SUB overflow, rs : %d, rt : %d\n", *rs, *rt);
        terminate(0);
    }
}

void r_SUBU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *rs - *rt;
}

void r_XOR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = (*rs) xor *rt;
}

void i_XORI(int32_t * rs, int32_t * rt, int imm){
    *rt = (*rs) xor (uint16_t) imm;
}

void i_LUI(int32_t * rs, int32_t * rt, int imm){
    *rt = imm << 16;
}

void r_SLT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = ((*rs) < (*rt)) ? 1:0;
}

void r_SLTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = (((uint32_t) *rs) < ((uint32_t) *rt)) ? 1:0;
}

void i_SLTI(int32_t * rs, int32_t * rt, int imm){
    *rt = (*rs < imm) ? 1:0;
}

void i_SLTIU(int32_t * rs, int32_t * rt, int imm){
    *rt = (((uint32_t) *rs) < ((uint16_t) imm)) ? 1:0;
}

void i_BEQ(int32_t * rs, int32_t * rt, int imm){
    if(*rs == *rt) pc += imm * 4;
}

void i_BGEZ(int32_t * rs, int32_t * rt, int imm){
    if(*rs >= 0) pc += imm * 4;
}

void i_BGEZAL(int32_t * rs, int32_t * rt, int imm){
    if(*rs >= 0){
        *regs[31] = pc;
        pc += imm * 4;
    }
}

void i_BGTZ(int32_t * rs, int32_t * rt, int imm){
    if(*rs > 0) pc += imm * 4;
}

void i_BLEZ(int32_t * rs, int32_t * rt, int imm){
    if(*rs <=  0) pc += imm * 4;
}

void i_BLTZAL(int32_t * rs, int32_t * rt, int imm){
    if(*rs < 0){
        *regs[31] = pc;
        pc += imm * 4;
    }
}

void i_BLTZ(int32_t * rs, int32_t * rt, int imm){
    if(*rs < 0) pc += imm * 4;
}

void i_BNE(int32_t * rs, int32_t * rt, int imm){
    if(*rs != *rt) pc += imm * 4;
}

void j_J(uint32_t target){
    pc >>= 28;
    pc <<= 28;
    pc += target * 4;
}

void j_JAL(uint32_t target){
    *regs[31] = pc;
    pc >>= 28;
    pc <<= 28;
    pc += target * 4;
}

void r_JALR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = pc;
    pc = *rs;
}

void r_JR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    pc = *rs;
}

void r_TEQ(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(*rs == *rt) trap();
}

void i_TEQI(int32_t * rs, int32_t * rt, int imm){
    if(*rs == imm) trap();
}

void r_TNE(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(*rs != *rt) trap();
}

void i_TNEI(int32_t * rs, int32_t * rt, int imm){
    if(*rs != imm) trap();
}

void r_TGE(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(*rs >= *rt) trap();
}

void r_TGEU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(((uint32_t) *rs) >= ((uint32_t) *rt)) trap();
}

void i_TGEI(int32_t * rs, int32_t * rt, int imm){
    if(*rs >= imm) trap();
}

void i_TGEIU(int32_t * rs, int32_t * rt, int imm){
    if(((uint32_t) *rs) >= ((uint32_t) imm)) trap();
}

void r_TLT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(*rs < *rt) trap();
}

void r_TLTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    if(((uint32_t) *rs) < ((uint32_t) *rt)) trap();
}

void i_TLTI(int32_t * rs, int32_t * rt, int imm){
    if(*rs < imm) trap();
}

void i_TLTIU(int32_t * rs, int32_t * rt, int imm){
    if(((uint32_t) *rs) < ((uint32_t) imm)) trap();
}

void i_LB(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int8_t * byte_ptr = (int8_t *) base_addr + (addr - SIM_BASE_ADDR);
    *rt = * byte_ptr;
}

void i_LBU(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    uint8_t * byte_ptr = base_addr + (addr - SIM_BASE_ADDR);
    *rt = * byte_ptr;
}

void i_LH(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int16_t * hw_ptr = (int16_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *rt = *hw_ptr;
}

void i_LHU(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    uint16_t * hw_ptr = (uint16_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *rt = *hw_ptr;
}

void i_LW(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int32_t * word_ptr = (int32_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *rt = *word_ptr;
}

void i_LWL(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int remainder = addr % 4;
    int byte = 4 - remainder;
    uint8_t * reg_ptr = (uint8_t *) rt;
    for(int i = 0; i < byte; i++){
        *reg_ptr = base_addr[addr - SIM_BASE_ADDR + i];
        reg_ptr++;
    }
}

void i_LWR(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int remainder = addr % 4;
    int byte = remainder + 1;
    uint8_t * reg_ptr = ((uint8_t *) rt) + 3;
    for(int i = 0; i < byte; i++){
        *reg_ptr = base_addr[addr - SIM_BASE_ADDR - i];
        reg_ptr--;
    }
}

void i_LL(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int32_t * word_ptr = (int32_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *rt = *word_ptr;
    LLbit = 1;
}

void i_SB(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    uint8_t byte = *rt;
    base_addr[addr - SIM_BASE_ADDR] = byte;
}

void i_SH(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    uint16_t * hw_ptr = (uint16_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *hw_ptr = *rt;
}

void i_SW(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    uint32_t * word_ptr = (uint32_t *) (base_addr + (addr - SIM_BASE_ADDR));
    *word_ptr = *rt;
}

void i_SWL(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int remainder = addr % 4;
    int byte = 4 - remainder;
    uint8_t * reg_ptr = (uint8_t *) rt;
    for(int i = 0; i < byte; i++){
        base_addr[addr - SIM_BASE_ADDR + i] = *reg_ptr;
        reg_ptr++;
    }
}

void i_SWR(int32_t * rs, int32_t * rt, int imm){
    uint32_t addr = *rs + imm;
    int remainder = addr % 4;
    int byte = remainder + 1;
    uint8_t * reg_ptr = ((uint8_t *) rt) + 3;
    for(int i = 0; i < byte; i++){
        base_addr[addr - SIM_BASE_ADDR - i] = *reg_ptr;
        reg_ptr--;
    }
}

void i_SC(int32_t * rs, int32_t * rt, int imm){
    if (LLbit) {
        uint32_t addr = *rs + imm;
        uint32_t * word_ptr = (uint32_t *) (base_addr + (addr - SIM_BASE_ADDR));
        *word_ptr = *rt;
        *rt = 1;
        LLbit = 0;
    }
    else{
        *rt = 0;
    }
}

void r_MFHI(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *hi;
}

void r_MFLO(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *rd = *lo;
}

void r_MTHI(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *hi = *rs;
}

void r_MTLO(int32_t * rs, int32_t * rt, int32_t * rd, int shamt){
    *lo = *rs;
}

void r_SYSCALL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt) {
    static int fileCount = 0;
    int value = *regs[2];
    switch (value) {
    case 1:{
        cout << *regs[4] << endl;
        break;
    }
    case 4:{
        char * char_ptr = (char *) (base_addr + ((*regs[4]) - SIM_BASE_ADDR));
        string temp = char_ptr;
        cout << temp << endl;
        break;
    }
    case 5:{
        int *num = (int*) malloc(sizeof(int));
        cout << "SYSCALL 5 - Input an integer: " << endl;
        cin >> *num;
        *regs[2] = *num;
        free(num);
        break;
    }
    case 8:{
        string temp;
        cout << "SYSCALL 8 - Input a string: " << endl;
        cin >> temp;
        temp = temp.substr(0, *regs[5]);
        char * char_ptr = (char *) (base_addr + (*regs[4]-SIM_BASE_ADDR));
        for (char ch : temp){
            *char_ptr = ch;
            char_ptr++;
        }
        *char_ptr = '\0';
        break;
    }
    case 9:{
        *regs[2] = dynamic_data;
        dynamic_data += *regs[4];
        break;
    }
    case 10:{
        cout << "SYSCALL 10" << endl;
        terminate(1);
        break;
    }
    case 11:{
        cout << char (*regs[4]) << endl;
        break;
    }
    case 12:{
        char ch;
        cout << "SYSCALL 12 - Input a character: " << endl;
        cin >> ch;
        *regs[2] = (uint32_t) ch;
        break;
    }
    case 13:{
        fstream * fs_ptr;
        char * char_ptr = (char *) (base_addr + (*regs[4]-SIM_BASE_ADDR));
        string filename = char_ptr;

        //flag = 'r'
        if(*regs[5] == 0){
            fs_ptr = new fstream(filename, ios::in);
        }
        //flag = 'w'
        else if(*regs[5] == 1){
            fs_ptr = new fstream(filename, ios::out);
        }
        //flag = 'rw'
        else {
            fs_ptr = new fstream(filename, ios::in|ios::out);
        }

        if((*fs_ptr).fail()){
            cout << "Failed to open: " << filename << endl;
            terminate(0);
        }

        //Put the pointer to fs into fileDescriptor
        fileDescriptor.push_back(fs_ptr);

        //Put the file descriptor (the index of filestream) into $a0
        //The index starts from 1
        *regs[4] = (uint32_t) ++fileCount;
        break;
    }
    case 14:{
        fstream * fs_ptr;
        //File descriptor starts from 1
        //But a vector is indexed from 0
        fs_ptr = fileDescriptor[*regs[4]-1];
        string temp = "";
        char ch;
        int count = 0;

        //Read data from the input file
        while(((*fs_ptr).get(ch)) && (count < *regs[6])){
            temp += ch;
            count++;
        }

        //Write the data to the memory
        char * char_ptr = (char *) (base_addr + (*regs[5] - SIM_BASE_ADDR));
        for (char ch : temp){
            *char_ptr = ch;
            char_ptr++;
        }

        //Put the number of char into $a0
        *regs[4] = count;

        break;
    }
    case 15:{
        fstream * fs_ptr;
        fs_ptr = fileDescriptor[*regs[4]-1];

        //Read data from the memory
        char * char_ptr = (char *) (base_addr + (*regs[5] - SIM_BASE_ADDR));
        string temp = char_ptr;
        temp = temp.substr(0, *regs[6]);

        //Write the data to the output file
        (*fs_ptr) << temp;

        //Put the number of char in $a0
        *regs[4] = *regs[6];
        break;
    }
    case 16:{
        fstream * fs_ptr;
        fs_ptr = fileDescriptor[*regs[4]-1];
        (*fs_ptr).close();
        free(fs_ptr);
        fileDescriptor.erase(fileDescriptor.begin()+*regs[4]-1);
        fileCount--;
        break;
    }
    case 17:{
        cout << "SYSCALL 17" << endl;
        cout << *regs[4] << endl;
        terminate(1);
        break;
    }

    default:{
        cout << "Invalid SYSCALL code: " << value << endl;
        terminate(0);
        break;
    }
    }
}
