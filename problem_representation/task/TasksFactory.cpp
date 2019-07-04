#include <TasksFactory.h>
#include <TaskFactory.h>
#include <Tasks.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>
#include <ProblemJsonNamesDefinitions.h>

std::unique_ptr<Tasks> TasksFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto task = make(json);
    JSON::deleteObject(json);
    return task;
}

std::unique_ptr<Tasks> TasksFactory::make(JsonObject& json){
    if(!json.hasItem(TASKS)){
        return nullptr;
    }
    auto tasks_array = json.getArray(TASKS);
    auto tasks = std::make_unique<Tasks>();

    for (size_t i = 0; i < tasks_array.size(); i++){
        auto task_json = tasks_array.getObject(i);
        auto task = TaskFactory::make(task_json);
        if( task == nullptr ) {
            return nullptr;
        }else{
            tasks->push_back(std::move(task));
        }
    }
    return tasks;
}