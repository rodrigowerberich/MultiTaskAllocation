#include <parse.h>
#include <ProblemRepresentation.h>
#include <iostream>

void parse(std::tuple <std::string> values){
    using namespace std;
    auto file_name = std::get<0>(values);
    cout << "Parsing " << file_name << "...\n";
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
    }else{
        cout << "Parsed with success!!" << endl;
    }
}