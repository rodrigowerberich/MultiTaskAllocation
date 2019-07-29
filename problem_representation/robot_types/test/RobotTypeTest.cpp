#include <RobotTypeTest.h>
#include <RobotTypesSet.h>
#include <RobotTypesFactory.h>
#include <iostream>

static bool robot_type_test_set(){
    RobotTypesSet robot_types;
    robot_types.addRobotTypes("banana");
    robot_types.addRobotTypes("typhoon");
    robot_types.addRobotTypes("rool the dice");
    for(const auto& type: robot_types.getRobotTypes()){
        std::cout << type << std::endl;
    }
    return true;
}

static bool robot_type_test_factory(){
    auto robot_types = RobotTypesFactory::make(R"({"robot_types":["banana","typhoon","rool the dice"]})");
    for(const auto& type: robot_types->getRobotTypes()){
        std::cout << type << std::endl;
    }
    return true;
}

void robot_type_test(){
    robot_type_test_set();
    robot_type_test_factory();
}
