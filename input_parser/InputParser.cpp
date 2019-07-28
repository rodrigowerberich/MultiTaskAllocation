#include <InputParser.h>
#include <MainOptions.h>
#include <iostream>

InputParser::InputParser(int argc, char* argv[]){
    MainOptions mo(argc, argv);
    MainOptions::Option* opt = mo.getParamFromKey("-f");
    const std::string file_name = opt ? (*opt).second : "";
    if(file_name == ""){
        std::cout << "\033[1;31mA filename is required! \033[0;36m Ex: (-f filename) \033[0m"<< std::endl;
    }
    this->file_name = file_name;
}

InputParser::~InputParser()
{
}

bool InputParser::inputValid() const{
    return this->file_name != "";
}


const std::string& InputParser::getFileName() const{
    return this->file_name;
}

