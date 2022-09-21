#include "data.h"
#include "MIPS.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;

/*
 * Load the static data to the memory
 * The source is a MIPS assembly language file
 */
uint32_t * loadData(ifstream & source, uint32_t * data_addr){
    //Build the data function map
    typedef uint32_t * (*dataFunc)(string&, uint32_t *);
    static map<string,dataFunc> dataFuncMap;

    dataFuncMap.insert(make_pair(".word", wordData));
    dataFuncMap.insert(make_pair(".byte", byteData));
    dataFuncMap.insert(make_pair(".double", doubleData));
    dataFuncMap.insert(make_pair(".float", floatData));
    dataFuncMap.insert(make_pair(".half", halfwordData));
    dataFuncMap.insert(make_pair(".ascii", asciiData));
    dataFuncMap.insert(make_pair(".asciiz", asciizData));

    //line = the original line
    //name = name of the variable
    //type = type of the data
    //data = the data
    string line;
    string name;
    string type;
    string data;
    uint32_t * data_ptr = data_addr;

    int line_number = 0;

    //Indicate if there is a .data section
    bool data_section = 0;

    while (getline(source, line)){
        line_number++;
        trim(line);

        //Data is between .data and .text
        if (line.substr(0,5) == ".data") {
            data_section = 1;
            continue;
        }
        if (line.substr(0,5) == ".text") return data_ptr;
        if (!data_section) continue;
        if (line == "") continue;

        //Find the name of the variable
        unsigned colon = line.find(":");
        if (colon < line.length()){
            name = line.substr(0, colon);
            line = line.substr(colon+1);
            trim(line);
        }
        else cout << "Invalid data at line " << line_number << endl;

        //Find the type of the data
        unsigned space = line.find(' ');
        if (space < line.length()){
            type = line.substr(0, space);
            line = line.substr(space+1);
            trim(line);
        }
        else cout << "Invalid data at line " << line_number << endl;

        //Find the data
        unsigned hash = line.find("#");
        if (hash < line.length()){
            line = line.substr(0, hash);
            trim(line);
            data = line;
        }
        data_ptr = dataFuncMap[type](data, data_ptr);
    }
    return data_ptr;
}

/*
 * Deal with .ascii data
 */
uint32_t * asciiData(string &data, uint32_t * data_ptr){
    if(data[0]!='"' || data[data.length()-1]!='"'){
        cout << "Missing quotation marks in the .data" << endl;
        return data_ptr;
    }
    int len = data.length() - 2;
    string temp = data.substr(1, len);

    char * char_ptr = (char *) data_ptr;
    for (int i = 0; i < temp.length(); i++){
        *char_ptr = temp[i];
        char_ptr++;
    }

    //Align data_ptr to a multiple of 4
    int word = len / 4;
    int byte = len % 4;
    if (byte) word++;

    //If the string is empty
    if(word == 0) word = 1;

    return data_ptr + word;
}

/*
 * Deal with .asciiz data
 * First, insert a '\0' at the end of the string
 * Then, handle it as a .ascii data
 */
uint32_t * asciizData(string &data, uint32_t * data_ptr){
    if(data[0]!='"' || data[data.length()-1]!='"'){
        cout << "Missing quotation marks in the .data" << endl;
        return data_ptr;
    }
    data = data.substr(0, data.length()-1) + '\0' + '"';
    return asciiData(data, data_ptr);
}

/*
 * Deal with .word data
 */
uint32_t * wordData(string &data, uint32_t * data_ptr){
    uint32_t * word_ptr = data_ptr;
    vector<string> words = split(data, ",");
    for (string word : words){
        trim(word);
        int num = stoi(word);
        *word_ptr = num;
        word_ptr++;
    }
    return data_ptr + words.size();
}

/*
 * Deal with .double data
 */
uint32_t * doubleData(string &data, uint32_t * data_ptr){
    stringstream dataflow;
    double num;
    dataflow << data;
    dataflow >> num;
    double * double_ptr = (double *) data_ptr;
    *double_ptr = num;
    return (data_ptr+2);
}

/*
 * Deal with .float data
 */
uint32_t * floatData(string &data, uint32_t * data_ptr){
    stringstream dataflow;
    float num;
    dataflow << data;
    dataflow >> num;
    float * float_ptr = (float *) data_ptr;
    *float_ptr = num;
    return ++data_ptr;
}

/*
 * Deal with .byte data
 */
uint32_t * byteData(string &data, uint32_t * data_ptr){
    uint8_t * byte_ptr = (uint8_t *) data_ptr;
    vector<string> bytes = split(data, ",");
    for (string byte : bytes){
        trim(byte);
        if (isInt(byte)){
            * byte_ptr = stoi(byte);
        }
        else if (byte.length() == 3){
            * byte_ptr = byte[1];
        }
        byte_ptr++;
    }

    //Align data_ptr to a multiple of 4
    int word = bytes.size() / 4;
    int byte = bytes.size() % 4;
    if(byte) word++;

    return data_ptr + word;
}

/*
 * Deal with .halfword data
 */
uint32_t * halfwordData(string &data, uint32_t * data_ptr){
    uint16_t * hw_ptr = (uint16_t *) data_ptr;
    vector<string> halfwords = split(data, ",");
    for (string halfword : halfwords){
        trim(halfword);
        if (isInt(halfword)){
            * hw_ptr = stoi(halfword);
        }
        else if(data[0]!='"' || data[data.length()-1]!='"'){
                cout << "Missing quotation marks in the .data" << endl;
                return data_ptr;
        }
        else{
            char * byte_ptr = (char *) hw_ptr;
            *byte_ptr = data[1];
            if (data.length()==4){
                *(byte_ptr + 1) = data[2];
            }
        }
        hw_ptr++;
    }

    //Align data_ptr to a multiple of 4
    int word = halfwords.size() / 2;
    int hw = halfwords.size() % 2;
    if(hw) word++;

    return data_ptr + word;
}
