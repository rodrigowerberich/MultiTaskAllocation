#include <TaskTypeTest.h>
#include <TaskTypesSet.h>
#include <TaskTypesFactory.h>
#include <iostream>

bool task_type_test_set(){
    TaskTypesSet task_types;
    task_types.addTaskTypes("A1");
    task_types.addTaskTypes("A2");
    task_types.addTaskTypes("A3");
    for(const auto& type: task_types.getTaskTypes()){
        std::cout << type << std::endl;
    }
    return true;
}

bool task_type_test_factory(){
    auto task_types = TaskTypesFactory::make(R"({"task_types":["A1","A2","A3"]})");
    for(const auto& type: task_types->getTaskTypes()){
        std::cout << type << std::endl;
    }
    return true;
}

void task_type_test(){
    task_type_test_set();
    task_type_test_factory();
}
