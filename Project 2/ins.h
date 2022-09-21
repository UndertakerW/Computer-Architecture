#ifndef INS_H
#define INS_H
#include <cstdint>

//Function prototypes

/*
 * Overflow detection for ADD,ADDI,SUB
 */
bool overflow(int64_t value);

/*
 * Whenever a trap exception is raised, terminate the program
 */
void trap();

void r_ADD(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_ADDU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_ADDI(int32_t * rs, int32_t * rt, int imm);

void i_ADDIU(int32_t * rs, int32_t * rt, int imm);

void r_AND(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_ANDI(int32_t * rs, int32_t * rt, int imm);

void r_DIV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_DIVU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_MULT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_MULTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_NOR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_OR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_ORI(int32_t * rs, int32_t * rt, int imm);

void r_SLL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SLLV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SRA(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SRAV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SRL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SRLV(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SUB(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SUBU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_XOR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_XORI(int32_t * rs, int32_t * rt, int imm);

void i_LUI(int32_t * rs, int32_t * rt, int imm);

void r_SLT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SLTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_SLTI(int32_t * rs, int32_t * rt, int imm);

void i_SLTIU(int32_t * rs, int32_t * rt, int imm);

void i_BEQ(int32_t * rs, int32_t * rt, int imm);

void i_BGEZ(int32_t * rs, int32_t * rt, int imm);

void i_BGEZAL(int32_t * rs, int32_t * rt, int imm);

void i_BGTZ(int32_t * rs, int32_t * rt, int imm);

void i_BLEZ(int32_t * rs, int32_t * rt, int imm);

void i_BLTZAL(int32_t * rs, int32_t * rt, int imm);

void i_BLTZ(int32_t * rs, int32_t * rt, int imm);

void i_BNE(int32_t * rs, int32_t * rt, int imm);

void j_J(uint32_t target);

void j_JAL(uint32_t target);

void r_JALR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_JR(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_TEQ(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_TEQI(int32_t * rs, int32_t * rt, int imm);

void r_TNE(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_TNEI(int32_t * rs, int32_t * rt, int imm);

void r_TGE(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_TGEU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_TGEI(int32_t * rs, int32_t * rt, int imm);

void i_TGEIU(int32_t * rs, int32_t * rt, int imm);

void r_TLT(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_TLTU(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void i_TLTI(int32_t * rs, int32_t * rt, int imm);

void i_TLTIU(int32_t * rs, int32_t * rt, int imm);

void i_LB(int32_t * rs, int32_t * rt, int imm);

void i_LBU(int32_t * rs, int32_t * rt, int imm);

void i_LH(int32_t * rs, int32_t * rt, int imm);

void i_LHU(int32_t * rs, int32_t * rt, int imm);

void i_LW(int32_t * rs, int32_t * rt, int imm);

void i_LWL(int32_t * rs, int32_t * rt, int imm);

void i_LWR(int32_t * rs, int32_t * rt, int imm);

void i_LL(int32_t * rs, int32_t * rt, int imm);

void i_SB(int32_t * rs, int32_t * rt, int imm);

void i_SH(int32_t * rs, int32_t * rt, int imm);

void i_SW(int32_t * rs, int32_t * rt, int imm);

void i_SWL(int32_t * rs, int32_t * rt, int imm);

void i_SWR(int32_t * rs, int32_t * rt, int imm);

void i_SC(int32_t * rs, int32_t * rt, int imm);

void r_MFHI(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_MFLO(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_MTHI(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_MTLO(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

void r_SYSCALL(int32_t * rs, int32_t * rt, int32_t * rd, int shamt);

#endif // INS_H
