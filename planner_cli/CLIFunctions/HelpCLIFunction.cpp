#ifndef HELPCLIFUNCTION_CPP__
#define HELPCLIFUNCTION_CPP__

#include <HelpCLIFunction.h>
#include <iostream>

HelpCLIFunction::HelpCLIFunction(std::string name, const HelpCLIFunction::FunctionContainer& functions):PlannerCLIFunction{name, 0}{
    for(const auto& name_function_pair:functions){
        m_functions.push_back(std::make_tuple(name_function_pair.first, name_function_pair.second->getNumOfArguments(), name_function_pair.second->getDescription()));
    }
}
std::string HelpCLIFunction::execute(std::deque<std::string>) const{
    using namespace std;
    cout << "-----------------------------------------------------\n";
    cout << "Displaying available commands descriptions\n";
    for(const auto& name_num_of_args_description: m_functions){
        cout << "-----------------------------------------------------\n";
        cout << "\033[34m" << std::get<0>(name_num_of_args_description) << "\033[m" << " receives " << std::get<1>(name_num_of_args_description) << " arguments\n";
        cout << "Description:\n" << std::get<2>(name_num_of_args_description) << endl;
    }
    cout << "-----------------------------------------------------\n";
    return "";
}
const std::string & HelpCLIFunction::getDescription() const{
    static const std::string description = "Help function";
    return description;
}

#endif //HELPCLIFUNCTION_CPP__