#include <ProblemRepresentationTest.h>
#include <ProblemRepresentation.h>
#include <iostream>


bool ProblemRepresentationCorrectFileTest(){
    ProblemRepresentation representation("correct_problem_representation.json");
    if(!representation.isValid()){
        std::cout << "Failed here\n" << std::endl;
        return false;
    }
    const auto& robot_types = representation.getRobotTypes();
    if(robot_types == nullptr){
        return false;
    }
    for(const auto& robot_type: robot_types->getRobotTypes()){
        std::cout << robot_type << std::endl;
    }
    const auto& task_types = representation.getTaskTypes();
    if(task_types == nullptr){
        return false;
    }
    for(const auto& task_type: task_types->getTaskTypes()){
        std::cout << task_type << std::endl;
    }
    const auto& effort_function = representation.getEffortFunction();
    for(const auto& task_type: task_types->getTaskTypes()){
        for(const auto& robot_type: robot_types->getRobotTypes()){
            std::cout << "effort_function(" << task_type << " ,"  << robot_type << ") = " << (*effort_function)(task_type, robot_type) << std::endl;
        }
    }
    
    const auto& reward_function = representation.getRewardFunction();
    for(const auto& task_type: task_types->getTaskTypes()){
        std::cout << "reward_function(" << task_type << ") = " << (*reward_function)(task_type) << std::endl;
    }

    return true;
}

#define RUN_TEST(name_)\
    if(name_ ()){\
        std::cout << #name_ " passed!"<< std::endl;\
    }else{\
        std::cout << #name_ " failed!"<< std::endl;\
    }

void problem_definition_test(){
    RUN_TEST(ProblemRepresentationCorrectFileTest);
}
