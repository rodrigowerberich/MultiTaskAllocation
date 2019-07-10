#include <TaskFactory.h>
#include <Task2d.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>

#define TASK_ID "id"
#define TASK_TYPE "type"
#define TASK_POSITION "position"
#define TASK_PREREQUISITES "prerequisites"

std::unique_ptr<Task> TaskFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto task = make(json);
    JSON::deleteObject(json);
    return task;
}

std::unique_ptr<Task> TaskFactory::make(JsonObject& json){
    if(!json.hasItem(TASK_ID) || !json.hasItem(TASK_TYPE) || !json.hasItem(TASK_POSITION) || !json.hasItem(TASK_PREREQUISITES)){
        return nullptr;
    }
    
    auto task_id = json.getString(TASK_ID);
    auto task_type = json.getString(TASK_TYPE);
    auto task_position_json = json.getObject(TASK_POSITION);
    auto task_position = PositionFactory::make(task_position_json);
    auto task_prerequisites_array = json.getArray(TASK_PREREQUISITES);
    auto task_prerequisite = std::set<std::string>();

    for (size_t i = 0; i < task_prerequisites_array.size(); i++){
        task_prerequisite.insert(task_prerequisites_array.getString(i));
    }

    return std::make_unique<Task2d>(task_id, task_type, task_position, task_prerequisite);
}
