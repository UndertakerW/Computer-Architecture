#ifndef DATA_H
#define DATA_H
#include <string>

/*
 * Load the static data to the memory
 * The source is a MIPS assembly language file
 */
uint32_t * loadData(std::ifstream & source, uint32_t * data_addr);

/*
 * Deal with .ascii data
 */
uint32_t * asciiData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .asciiz data
 * First, insert a '\0' at the end of the string
 * Then, handle it as a .ascii data
 */
uint32_t * asciizData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .word data
 */
uint32_t * wordData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .halfword data
 */
uint32_t * halfwordData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .float data
 */
uint32_t * floatData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .byte data
 */
uint32_t * byteData(std::string &data, uint32_t * data_ptr);

/*
 * Deal with .double data
 */
uint32_t * doubleData(std::string &data, uint32_t * data_ptr);


#endif // DATA_H
