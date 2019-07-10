#include <MissionFactory.h>
#include <MissionImpl.h>
#include <limits>
#include <iostream>

#define MISSION_ID "id"
#define MISSION_PRIORITY "priority"
#define MISSION_DEADLINE "deadline"
#define MISSION_TASKS "tasks"

std::unique_ptr<Mission> MissionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto task = make(json);
    JSON::deleteObject(json);
    return task;
}

std::unique_ptr<Mission> MissionFactory::make(JsonObject& json){
    std::cout << !json.hasItem(MISSION_ID) << !json.hasItem(MISSION_PRIORITY) << !json.hasItem(MISSION_DEADLINE) << !json.hasItem(MISSION_TASKS) <<std::endl;
    if(!json.hasItem(MISSION_ID) || !json.hasItem(MISSION_PRIORITY) || !json.hasItem(MISSION_DEADLINE) || !json.hasItem(MISSION_TASKS)){
        std::cout << json.toString() << std::endl;
        return nullptr;
    }
    
    auto task_id = json.getString(MISSION_ID);
    auto task_priority = json.getInt(MISSION_PRIORITY);
    auto task_deadline = json.getDouble(MISSION_DEADLINE);
    auto task_tasks = std::set<std::string>();
    auto task_tasks_array = json.getArray(MISSION_TASKS);
    for (size_t i = 0; i < task_tasks_array.size(); i++){
        task_tasks.insert(task_tasks_array.getString(i));
    }

    return std::make_unique<MissionImpl>(task_id, task_priority, task_deadline, task_tasks);
}
