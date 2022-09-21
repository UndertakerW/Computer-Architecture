#ifndef MAIN_H
#define MAIN_H
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <string>

//Global variables
extern std::vector<int32_t *> regs;
extern int32_t * hi;
extern int32_t * lo;
extern uint32_t pc;
extern uint32_t dynamic_data;
extern uint8_t * base_addr;
extern uint32_t SIM_BASE_ADDR;
extern bool LLbit;
extern std::vector<std::fstream*> fileDescriptor;

//Function Prototypes

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
int main(int argc, char** argv);

/*
 * Terminate the program
 * Meanwhile output the current memory address
 */
void terminate(bool success);

/*
 * Initialize the registers
 */
void initRegs(uint32_t stack_bottom, uint32_t static_bottom);

/*
 * Build the function map
 */
void buildFuncMap();

/*
 * Convert a string containing machine code to a 32-bit unsigned integer.
 */
uint32_t stringToUnsigned(std::string code);

/*
 * Load the machine code to the text section of the simulated memory
 */
int loadText(uint8_t * base_addr, std::istream & input);

/*
 * Convert a 32-bit unsigned integer to a string
 */
std::string binaryToString(uint32_t value);

/*
 * Fetch one instruction from the memory pointed by PC
 * Then increase PC by 4
 */
void fetchIns();

/*
 * Decode the instruction
 * Sort the instructions into R-type, I-type, and J-type
 */
void decodeIns(std::string ins);

/*
 * Convert a string to a signed integer
 * Note that code[0] corresponds to the sign bit
 */
int stringToSigned(std::string code);

/*
 * Decode an I-type instruction and call the corresponding function
 */
void iType(std::string ins);

/*
 * Decode an R-type instruction and call the corresponding function
 */
void rType(std::string ins);

/*
 * Decode a J-type instruction and call the corresponding function
 */
void jType(std::string ins);


#endif // MAIN_H
