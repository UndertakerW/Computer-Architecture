#ifndef MAIN_H
#define MAIN_H
#include <string>
#include <vector>
#include <map>

//Define a data structure for the MIPS instructions
struct Instruction{
    std::string name;
    std::string type;
    int opcode;
    int function;
    /* Notes for int function:
     * For R-type instructions, this variable stores "function"
     * For some I-type instructions, this variable stores "rt"
     * For other instructions, this variable is set to 0 and is meaningless.
     */
};


/*
 * The main function
 * This function takes the input filename and the output filename from command lines
 * and calls assembler() to start the work.
 */
int main(int argc, char** argv);


/*
 * This function takes as input a string, which contains a register name.
 * The input can be of the form either "$s0" or "100($s0)".
 * After referring to the Register Map,
 * it returns the corresponding register number.
 */
int regNameToNum(std::string name);


/*
 * This function is used to print a signed binary number.
 * The parameter num is the number to print,
 * digit is the number of digits to print,
 * and output is the output stream.
 */
void printBinary(int num, int digit, std::ostream & output);


/*
 * This function constructs the Register Map (Name -> Number),
 * which is a global variable.
 */
void constructRegMap(std::map<std::string,int> & regMap);


/*
 * This function scans through the whole input file for all the labels,
 * remembering each label and its address (line number).
 */
std::map<std::string, int> scanForLabels(std::istream & input);


/*
 * This function takes as input a line and returns the instruction.
 * Meanwhile, it cuts off the instruction from the original line for further oprations.
 */
std::string getInstruction(std::string & line);


/*
 * This function takes as input a reference to a string,
 * and erases all whitespaces, '\f', '\v', '\r', '\t', and '\n'
 * at the beginning of and the end of the string.
 */
void trim(std::string & str);


/*
 * The function splits a string with respect to the given sign,
 * and then returns a vector containing the substrings.
 */
std::vector<std::string> split(const std::string &str, const std::string &sign);


/*
 * This function takes as input an input stream and an output stream.
 * It firstly scans through the whole input file for the labels,
 * and then reads and translates the input file line by line.
 */
void assembler(std::istream & input, std::ostream & output);


/*
 * This function takes as input an R-type Instruction, a vector of arguments and an output stream.
 * It determines the opcode, rd, rs, rt, shamt, and funct for an R-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void rType(Instruction ins, std::vector<std::string> args, std::ostream & output);


/*
 * This function takes as input an I-type Instruction, a vector of arguments,
 * an output stream, a line number and a label map.
 * It determines the opcode, rt, rs, and imm (offset) for an I-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void iType(Instruction ins, std::vector<std::string> args, std::ostream & output,
           int line_number, std::map<std::string, int> labels);


/*
 * This function takes as input a J-type Instruction, a vector of arguments,
 * an output stream, and a label map.
 * It determines the opcode and target for an J-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void jType(Instruction ins, std::vector<std::string> args, std::ostream & output,
           std::map<std::string, int> labels);


/*
 * This function takes as input a string,
 * and returns whether the string is an integer.
 */
bool isInt(std::string arg);

#endif // MAIN_H
