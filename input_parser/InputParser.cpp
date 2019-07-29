#include <InputParser.h>
#include <MainOptions.h>
#include <iostream>

InputParser::InputParser(int argc, char* argv[]){
    MainOptions mo(argc, argv);
    MainOptions::Option* opt_file_name = mo.getParamFromKey("-f");
    const std::string file_name = opt_file_name ? (*opt_file_name).second : "";
    if(file_name == ""){
        std::cout << "\033[1;31mA filename is required! \033[0;36m Ex: (-f filename) \033[0m"<< std::endl;
    }
    m_file_name = file_name;
    MainOptions::Option* opt_show_value = mo.getParamFromKey("-show_problem");
    m_show_problem = opt_show_value;
}

InputParser::~InputParser()
{
}

bool InputParser::inputValid() const{
    return m_file_name != "";
}


const std::string& InputParser::getFileName() const{
    return m_file_name;
}

bool InputParser::showProblem() const{
    return m_show_problem;
}

