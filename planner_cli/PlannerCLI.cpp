#include <PlannerCLI.h>
#include <iostream>
#include <array>
#include <TwoDoubles.h>

void print_out(std::tuple <double, double> values){
    std::cout << std::get<0>(values) << " " << std::get<1>(values) << std::endl;
}
void sum_out(std::tuple <double, double> values){
    std::cout << std::get<0>(values) + std::get<1>(values) << std::endl;
}

PlannerCLI::PlannerCLI(int argc, char *argv[]):m_started_empty{false}{
    if(argc == 1){
        m_started_empty = true;
    }else{
        for(int i = 1; i < argc; i++){
            m_arguments.push_back(argv[i]);
        }
    }
    addCLIFunction<TwoDoubleCLIFunction>("Print out", "Prints two numbers out", print_out);
    addCLIFunction<TwoDoubleCLIFunction>("Sum out", "Prints the sum of two numbers out", sum_out);
}

int PlannerCLI::run(){
    if(m_started_empty){
        std::cout << "No argument was given, execute help function and finish" << std::endl;
        return 0;
    }

    for(const auto& planner_cli: m_functions){
        std::cout << planner_cli->getName() << std::endl;
        std::cout << planner_cli->getDescription() << std::endl;
        planner_cli->execute(m_arguments);
    }

    while (!m_arguments.empty()){
        auto argument = m_arguments.front();
        m_arguments.pop_front();
        std::cout << "Processing " << argument << std::endl;
    }
    return 0;
}
