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

    const auto& robots = representation.getRobots();
    for (const auto & robot: *robots) {
        std::cout << "Robot " << robot->getName() << " is of type " << robot->getType() << " at position (" << robot->getPosition().getX() << ", " << robot->getPosition().getY() << ")" << std::endl;
    }

    const auto& robot_real = robots->getRobot("Bruce");
    if( robot_real != nullptr){
        std::cout << "Robot " << robot_real->getName() << " is real! It is of type " << robot_real->getType() << " at position (" << robot_real->getPosition().getX() << ", " << robot_real->getPosition().getY() << ")" << std::endl;
    }else{
        std::cout << "I'm missing bruce!\n";
    }
    const auto& robot_fake = robots->getRobot("Banana");
    if( robot_fake == nullptr){
        std::cout << "No Banana. The world is at peace\n";
    }else{
        std::cout << "FUUUUCK WHY ARE BANANAS HERE????\n";
    }

    const auto& tasks = representation.getTasks();
    for (const auto & task: *tasks) {
        std::cout << "Task " << task->getName() << " is of type " << task->getType() << " at position (" << task->getPosition().getX() << ", " << task->getPosition().getY() << ")";
        std::cout << " prerequisites are: ";
        for(const auto& prerequisite: task->getPrerequisites()){
            std::cout << prerequisite << ", ";
        }
        auto prerequisite_test = "Task4";
        std::cout << std::endl;
        std::cout << "Is " << prerequisite_test << " a prerequisite of " << task->getName() << "? " << (task->hasPrerequisite(prerequisite_test)? "Yes": "No") << std::endl;
    }

    const auto& missions = representation.getMissions();
    for( const auto& mission: *missions ){
        std::cout << "Mission " << mission->getName() << " has a priority of " << mission->getPriority() << " with a deadline of " << mission->getDeadline() << " seconds" << std::endl;
        std::cout << " mission tasks are:  \n";
        for(const auto& task_id: mission->getTasks()){
            const auto& task = tasks->getTask(task_id);
            if(task == nullptr){
                return false;
            }else{
                std::cout << task->getName() << ", ";
            }
        }
        std::cout << std::endl; 
    }
    
    const auto& search_area = representation.getSearchArea();
    std::cout << search_area->toStringRepresentation() << std::endl; 
    std::cout << "Search area contains point ""0,0"" " << search_area->containsPoint({0,0}) << std::endl;
    std::cout << "Search area contains point ""-7,3"" " << search_area->containsPoint({-7,3}) << std::endl;
    std::cout << "Search area contains point ""8,-2"" " << search_area->containsPoint({8,-2}) << std::endl;
    std::cout << "Search area contains point ""4,4.5"" " << search_area->containsPoint({4,4.5}) << std::endl;


    const auto& obstructed_area = representation.getObstructedArea();
    std::cout << obstructed_area->toStringRepresentation() << std::endl;
    std::cout << "Obstructed area contains point ""0,0"" " << obstructed_area->containsPoint({0,0}) << std::endl;
    std::cout << "Obstructed area contains point ""-7,3"" " << obstructed_area->containsPoint({-7,3}) << std::endl;
    std::cout << "Obstructed area contains point ""8,-2"" " << obstructed_area->containsPoint({8,-2}) << std::endl;
    std::cout << "Obstructed area contains point ""4,4.5"" " << obstructed_area->containsPoint({4,4.5}) << std::endl;
    std::cout << "Obstructed area contains point ""2,3"" " << obstructed_area->containsPoint({2,3}) << std::endl;

    const auto& connectivity_function = representation.getConnectivityFunction();
    std::cout << connectivity_function->toStringRepresentation() << std::endl;
    std::cout << "Connectivity function value at ""0,0"" " <<   (*connectivity_function)({0,0}) << std::endl;
    std::cout << "Connectivity function value at ""0,0"" " <<   (*connectivity_function)({0,0}) << std::endl;
    std::cout << "Connectivity function value at ""-7,3"" " <<  (*connectivity_function)({-7,3}) << std::endl;
    std::cout << "Connectivity function value at ""8,-2"" " <<  (*connectivity_function)({8,-2}) << std::endl;
    std::cout << "Connectivity function value at ""4,4.5"" " << (*connectivity_function)({4,4.5}) << std::endl;
    std::cout << "Connectivity function value at ""2,3"" " <<   (*connectivity_function)({2,3}) << std::endl;

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
