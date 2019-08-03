#include <generate_alpha_tasks.h>
#include <iostream>
#include <ProblemRepresentation.h>

void generate_alpha_tasks(std::tuple <std::string> values){
    using namespace std;
    auto file_name = std::get<0>(values);
    ProblemRepresentation problemRepresentation{file_name};
    if(!problemRepresentation.isValid()){
        cout << "\033[1;31mSomething is wrong with the problem representation file\033[0m" << endl;
        cout << "\033[1;31m"<< problemRepresentation.getErrorMessage() << "\033[0m" << endl;
    }
    
}