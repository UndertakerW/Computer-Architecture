#include <iostream>
#include <stdio.h>
#include <memory.h>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <vector>
#include <string.h>
#include <map>
#include <sstream>
#include "MIPS.h"
using namespace std;

//Register names
const vector<string> REG = {"zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3",
                            "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
                            "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"};

//Register map:
//Register Name -> Register Number
static map<string,int> regMap;


//MIPS instructions from page A51 to page A71 that are not pseudoinstructions or coprocessor-instructions.
static Instruction
ADD = {"add", "R", 0, 0x20},
ADDU = {"addu", "R", 0, 0x21},
ADDI = {"addi", "I", 8, 0},
ADDIU = {"addiu", "I", 9, 0},
AND = {"and", "R", 0, 0x24},
ANDI = {"andi", "I", 0xc, 0},
CLO = {"clo", "R", 0x1c, 0x21},
CLZ = {"clz", "R", 0x1c, 0x20},
DIV = {"div", "R", 0, 0x1a},
DIVU = {"divu", "R", 0, 0x1b},
MULT = {"mult", "R", 0, 0x18},
MULTU = {"multu", "R", 0, 0x19},
MUL = {"mul", "R", 0x1c, 2},
MADD = {"madd", "R", 0x1c, 0},
MADDU = {"maddu", "R", 0x1c, 1},
MSUB = {"msub", "R", 0x1c, 4},
MSUBU = {"msubu", "R", 0x1c, 5},
NOR = {"nor", "R", 0, 0x27},
OR = {"or", "R", 0, 0x25},
ORI = {"ori", "I", 0xd, 0},
SLL = {"sll", "R", 0, 0},
SLLV = {"sllv", "R", 0, 4},
SRA = {"sra", "R", 0, 3},
SRAV = {"srav", "R", 0, 7},
SRL = {"srl", "R", 0, 2},
SRLV = {"srlv", "R", 0, 6},
SUB = {"sub", "R", 0, 0x22},
SUBU = {"subu", "R", 0, 0x23},
XOR = {"xor", "R", 0, 0x26},
XORI = {"xori", "I", 0xe, 0},
LUI = {"lui", "I", 0xf, 0},
SLT = {"slt", "R", 0, 0x2a},
SLTU = {"sltu", "R", 0, 0x2b},
SLTI = {"slti", "I", 0xa, 0},
SLTIU = {"sltiu", "I", 0xb, 0},
BEQ = {"beq", "I", 4, 0},
BGEZ = {"bgez", "I", 1, 1},
BGEZAL = {"bgezal", "I", 1, 0x11},
BGTZ = {"bgtz", "I", 7, 0},
BLEZ = {"blez", "I", 6, 0},
BLTZAL = {"bltzal", "I", 1, 0x10},
BLTZ = {"bltz", "I", 1, 0},
BNE = {"bne", "I", 5, 0},
J = {"j", "J", 2, 0},
JAL = {"jal", "J", 3, 0},
JALR = {"jalr", "R", 0, 9},
JR = {"jr", "R", 0, 8},
TEQ = {"teq", "R", 0, 0x34},
TEQI = {"teqi", "I", 1, 0xc},
TNE = {"tne", "R", 0, 0x36},
TNEI = {"tnei", "I", 1, 0xe},
TGE = {"tge", "R", 0, 0x30},
TGEU = {"tgeu", "R", 0, 0x31},
TGEI = {"tgei", "I", 1, 8},
TGEIU = {"tgeiu", "I", 1, 9},
TLT = {"tlt", "R", 0, 0x32},
TLTU = {"tltu", "R", 0, 0x33},
TLTI = {"tlti", "I", 1, 0xa},
TLTIU = {"tltiu", "I", 1, 0xb},
LB = {"lb", "I", 0x20, 0},
LBU = {"lbu", "I", 0x24, 0},
LH = {"lh", "I", 0x21, 0},
LHU = {"lhu", "I", 0x25, 0},
LW = {"lw", "I", 0x23, 0},
LWL = {"lwl", "I", 0x22, 0},
LWR = {"lwr", "I", 0x26, 0},
LL = {"ll", "I", 0x30, 0},
SB = {"sb", "I", 0x28, 0},
SH = {"sh", "I", 0x29, 0},
SW = {"sw", "I", 0x2b, 0},
SWL = {"swl", "I", 0x2a, 0},
SWR = {"swr", "I", 0x2e, 0},
SC = {"sc", "I", 0x38, 0},
MFHI = {"mfhi", "R", 0, 0x10},
MFLO = {"mflo", "R", 0, 0x12},
MTHI = {"mthi", "R", 0, 0x11},
MTLO = {"mtlo", "R", 0, 0x13};

//The collection of MIPS instructions
static vector<Instruction> Instructions =
{ADD, ADDU, ADDI, ADDIU, AND, ANDI, CLO, CLZ, DIV, DIVU, MULT, MULTU,
 MUL, MADD, MADDU, MSUB, MSUBU, NOR, OR, ORI, SLL, SLLV, SRA, SRAV,
 SRL, SRLV, SUB, SUBU, XOR, XORI, LUI, SLT, SLTU, SLTI, SLTIU, BEQ,
 BGEZ, BGEZAL, BGTZ, BLEZ, BLTZAL, BLTZ, BNE, J, JAL, JALR, JR, TEQ,
 TEQI, TNE, TNEI, TGE, TGEU, TGEI, TGEIU, TLT, TLTU, TLTI, TLTIU, LB,
 LBU, LH, LHU, LW, LWL, LWR, LL, SB, SH, SW, SWL, SWR, SC, MFHI, MFLO,
 MTHI, MTLO};


/*
 * The main function
 * This function takes the input filename and the output filename from command lines
 * and calls assembler() to start the work.
 */
int main(int argc, char** argv){
    // Check the input
    if (argc != 3){
        cout << "Error: File name needed." << endl;
        exit(-1);
    }
    constructRegMap(regMap);
    ifstream input(argv[1]);
    string input_name = argv[1];
    string output_name = argv[2];
    ofstream output;
    output.open(output_name);
    assembler(input, output);
    input.close();
    output.close();
    return 0;
}

/*
 * This function constructs the Register Map (Name -> Number),
 * which is a global variable.
 */
void constructRegMap(map<string,int> & regMap){
    for (unsigned int i=0; i < REG.size(); i++){
        regMap[REG[i]] = int(i);
    }
}

/*
 * This function takes as input a string, which contains a register name.
 * The input can be of the form either "$s0" or "100($s0)".
 * After referring to the Register Map,
 * it returns the corresponding register number.
 */
int regNameToNum(string name){
    unsigned int dollar = name.find("$");
    if (dollar == name.length()){
        cout << "Invalid argument: " << name << endl;
        return -1;
    }
    string reg = "";
    for (unsigned int i = dollar+1; i < name.length(); i++){
        if (name[i] == ')') break;
        reg += name[i];
    }
    int num;
    if(regMap.find(reg) == regMap.end()){
        cout << "Invalid register name: " << reg << endl;
        return -1;
    }
    else{
        num = regMap[reg];
        return num;
    }
}

/*
 * This function is used to print a signed binary number.
 * The parameter num is the number to print,
 * digit is the number of digits to print,
 * and output is the output stream.
 */
void printBinary(int num, int digit, ostream & output){
    bool negative = false;
    //Remember the sign of the number, and get its absolute value.
    if (num < 0){
        negative = true;
        num = -num;
    }
    //If the number is negative, get its 2's complement.
    if (negative){
        int mask = 1;
        mask <<= digit;
        mask -= 1;
        num ^= mask; //XOR to get the 1's complement
        num += 1; //+1 to get the 2's complement
    }
    /*
     * From MSB, check every bit of the number
     * If the bit is 1, output << 1
     * If the bit is 0, output << 0
     */
    int a = 1 << (digit-1); //This works as a bit-checker
    for(int i = 0; i < digit ; i++){
        if (num & a) output << 1;
        else output << 0;
        num <<= 1;
    }
}

/*
 * This function scans through the whole input file for all the labels,
 * remembering each label and its address (line number).
 */
map<string, int> scanForLabels(istream & input){
    string line;
    int line_number = 0;
    map<string, int> labels;
    while (getline(input, line)){
        trim(line);
        if(line == "") continue;
        //If there is a comment at the end of the line, cut it off.
        unsigned index = line.find("#");
        if (index < line.length()){
            line = line.substr(0, index);
        }
        //If there is a label at the beginning of the line, record its name and address.
        //Then cut it off.
        index = line.find(":");
        if (index < line.length()){
            string label = line.substr(0, index);
            labels[label] = line_number;
            line = line.substr(index+1, line.length());
        }
        trim(line);
        if(line == "") continue;
        //Get the instruction.
        string instruction = getInstruction(line);
        vector<Instruction>::iterator it;
        //Find the corresponding "Instruction" in "Instructions"
        //If found, line number++
        for (it = Instructions.begin(); it != Instructions.end(); it++){
            if (it->name == instruction){
                line_number++;
                break;
            }
        }
    }
    /*
     * Reset the input stream for the second scanning.
     */
    input.clear();
    input.seekg(0);
    return labels;
}

/*
 * This function takes as input a line and returns the instruction.
 * Meanwhile, it cuts off the instruction from the original line for further oprations.
 */
string getInstruction(string & line){
    unsigned int i;
    char ch;
    string ins = "";
    bool found = false;
    for (i = 0; i < line.length(); i++){
        ch = line[i];
        if (ch == ' ' && found) {
            line = line.substr(i+1, line.length());
            break;
        }
        else if (ch != ' ' && ch != '\t'){
            found = true;
            ins += ch;
        }
    }
    return ins;
}

/*
 * The function splits a string with respect to the given sign,
 * and then returns a vector containing the substrings.
 */
vector<string> split(const string &str, const string &sign){
    vector<string> substrings;
    if(str == "")
        return substrings;
    //Add the sign at the end of the string,
    //in order to split the last substring.
    string strs = str + sign;
    size_t pos = strs.find(sign);

    while(pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        substrings.push_back(temp);
        //Cut off the substring that is already split
        //Keep splitting in the remaining string.
        strs = strs.substr(pos+1, strs.size());
        pos = strs.find(sign);
    }

    return substrings;
}

/*
 * This function takes as input a reference to a string,
 * and erases all whitespaces, '\f', '\v', '\r', '\t', and '\n'
 * at the beginning of and the end of the string.
 */
void trim(string & str){
    string blanks("\f\v\r\t\n ");
    str.erase(0,str.find_first_not_of(blanks));
    str.erase(str.find_last_not_of(blanks) + 1);
}

/*
 * This function takes as input an input stream and an output stream.
 * It firstly scans through the whole input file for the labels,
 * and then reads and translates the input file line by line.
 */
void assembler(istream & input, ostream & output){
    //Scan through the whole input file for the labels.
    map<string, int> labels = scanForLabels(input);
    string line;
    int original_line_number = 0;
    int line_number = 0;
    while (getline(input, line)){
        original_line_number++;
        trim(line);
        if(line == "") continue;
        //If there is a comment at the end of the line, cut it off.
        unsigned index = line.find("#");
        if (index < line.length()){
            line = line.substr(0, index);
        }
        //If there is a label at the beginning of the line, cut it off.
        index = line.find(":");
        if (index < line.length()){
            line = line.substr(index+1, line.length());
        }
        trim(line);
        if(line == "") continue;
        //Get the instruction and splits the arguments.
        string instruction = getInstruction(line);
        vector<string> args = split(line, ",");
        vector<Instruction>::iterator it;
        //Find the corresponding "Instruction" in "Instructions"
        //If found, line number++
        //If not found, output an error
        bool valid = false;
        for (it = Instructions.begin(); it != Instructions.end(); it++){
            if (it->name == instruction){
                Instruction ins = *it;
                valid = true;
                //Call the function for each type.
                if (ins.type == "R") rType(ins, args, output);
                else if (ins.type == "I") iType(ins, args, output, line_number, labels);
                else if (ins.type == "J") jType(ins, args, output, labels);
                line_number++;
                break;
            }
        }
        if (!valid){
            cout << "Invalid instruction " <<  instruction << " at line " << original_line_number << endl;
        }
    }
}

/*
 * This function takes as input an R-type Instruction, a vector of arguments and an output stream.
 * It determines the opcode, rd, rs, rt, shamt, and funct for an R-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void rType(Instruction ins, vector<string> args, ostream & output){
    trim(args[0]);
    int rd;
    int rs;
    int rt;
    int shamt = 0;
    //First, classify the instructions into 3 groups with respect to the number of arguments
    //Then, classify the instructions with respect to the order of arguments.
    if (args.size() == 1){
        if (ins.name == "mfhi" || ins.name == "mflo"){
            rd = regNameToNum(args[0]);
            rs = 0;
            rt = 0;
        }
        //mthi, mtlo, jr
        else{
            rs = regNameToNum(args[0]);
            rd = 0;
            rt = 0;
        }
    }
    else if (args.size() == 2){
        trim(args[1]);
        if (ins.name == "clo" || ins.name == "clz"){
            rd = regNameToNum(args[0]);
            rs = regNameToNum(args[1]);
            rt = 0;
        }
        else if (ins.name == "jalr"){
            rs = regNameToNum(args[0]);
            rd = regNameToNum(args[1]);
            rt = 0;
        }
        //teq, tne, tge, tgeu, tlt, tltu, div, divu
        else{
            rs = regNameToNum(args[0]);
            rt = regNameToNum(args[1]);
            rd = 0;
        }
    }
    //else if (arg.size() == 3)
    else{
        trim(args[2]);
        if (ins.name == "sll" || ins.name == "sra" || ins.name == "srl"){
            rd = regNameToNum(args[0]);
            rt = regNameToNum(args[1]);
            rs = 0;
            shamt = stoi(args[2]);
        }
        else if (ins.name == "sllv" || ins.name == "srav" || ins.name == "srlv"){
            rd = regNameToNum(args[0]);
            rt = regNameToNum(args[1]);
            rs = regNameToNum(args[2]);
        }
        //add, addu, and, mul, nor, or, sub, subu, xor, slt, sltu
        else{
            rd = regNameToNum(args[0]);
            rs = regNameToNum(args[1]);
            rt = regNameToNum(args[2]);
        }
    }
    printBinary(ins.opcode, 6, output);
    printBinary(rs, 5, output);
    printBinary(rt, 5, output);
    printBinary(rd, 5, output);
    printBinary(shamt, 5, output);
    printBinary(ins.function, 6, output);
    output << endl;
}

/*
 * This function takes as input an I-type Instruction, a vector of arguments,
 * an output stream, a line number and a label map.
 * It determines the opcode, rt, rs, and imm (offset) for an I-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void iType(Instruction ins, vector<string> args, ostream & output,
           int line_number, map<string, int> labels){
    trim(args[0]);
    trim(args[1]);
    int rt;
    int rs;
    int imm; //immediate or offset
    int label_number; //The number of line where the label occurs.
    //First, classify the instructions into 2 groups with respect to the number of arguments
    //Then, classify the instructions with respect to the order of arguments.
    if (args.size() == 2){
        if (ins.name == "lui"){
            rt = regNameToNum(args[0]);
            rs = 0;
            imm = stoi(args[1]);
        }
        /* Notes for ins.function:
         * For R-type instructions, this variable stores "function"
         * For some I-type instructions, this variable stores "rt"
         * For other instructions, this variable is set to 0 and is meaningless.
         */
        else if (ins.name == "teqi" || ins.name == "tnei" || ins.name == "tgei"
                 || ins.name == "tgeiu" || ins.name == "tlti" || ins.name == "tltiu"){
            rs = regNameToNum(args[0]);
            rt = ins.function;
            imm = stoi(args[1]);
        }
        else if (ins.name == "bgez" || ins.name == "bgezal" || ins.name == "bgtz"
                 || ins.name == "blez" || ins.name == "bltzal" || ins.name == "bltz"){
            rs = regNameToNum(args[0]);
            rt = ins.function;
            if (labels.count(args[1]) == 0){
                //If the argument is not a label but an offset:
                if (isInt(args[1])) imm = stoi(args[1]);
                else {
                    imm = 0;
                    cout << "Invalid label: " << args[1] << endl;
                }
            }
            else {
                label_number = labels[args[1]];
                imm = label_number - (line_number + 1);
            }
        }
        //lb, lbu, lh, lhu, lw, lwl, lwr, ll, sb, sh, sw, swl, swr, sc
        else{
            rt = regNameToNum(args[0]);
            unsigned int paren = args[1].find("(");
            imm = stoi(args[1].substr(0,paren));
            rs = regNameToNum(args[1]);
        }
    }
    //else if (args.size() == 3)
    else{
        trim(args[2]);
        if (ins.name == "beq" || ins.name == "bne" || ins.name == "jr"){
            rs = regNameToNum(args[0]);
            rt = regNameToNum(args[1]);
            if (labels.count(args[2]) == 0){
                //If the argument is not a label but an offset:
                if (isInt(args[2])) imm = stoi(args[2]);
                else {
                    imm = 0;
                    cout << "Invalid label: " << args[2] << endl;
                }
            }
            else{
                label_number = labels[args[2]];
                imm = label_number - (line_number + 1);
            }
        }
        //addi, addiu, andi, ori, slti, sltiu
        else{
            rt = regNameToNum(args[0]);
            rs = regNameToNum(args[1]);
            imm = stoi(args[2]);
        }
    }
    printBinary(ins.opcode, 6, output);
    printBinary(rs, 5, output);
    printBinary(rt, 5, output);
    printBinary(imm, 16, output);
    output << endl;
}

/*
 * This function takes as input a J-type Instruction, a vector of arguments,
 * an output stream, and a label map.
 * It determines the opcode and target for an J-type instrucion.
 * Then, it calls printBinary() to output the machine code.
 */
void jType(Instruction ins, vector<string> args, ostream & output, map<string, int> labels){
    trim(args[0]);
    int target;
    if (labels.count(args[0]) == 0){
        //If the argument is not a label but an absolute address:
        if (isInt(args[0])) target = stoi(args[0]);
        else {
            target = 0;
            cout << "Invalid label: " << args[0] << endl;
        }
    }
    else target = labels[args[0]]; //Address = Starting Address (0x0) + The number of line where the label occurs.
    printBinary(ins.opcode, 6, output);
    printBinary(target, 26, output);
    output << endl;
}

/*
 * This function takes as input a string,
 * and returns whether the string is an integer.
 */
bool isInt(string arg){
    stringstream str(arg);
    int t;
    char p;
    if(!(str >> t)) return false;
    if(str >> p) return false;
    else return true;
}
