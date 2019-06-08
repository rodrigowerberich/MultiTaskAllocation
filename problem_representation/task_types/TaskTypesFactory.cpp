#include <TaskTypesFactory.h>
#include <JSON.h>
#include <TaskTypesSet.h>

std::unique_ptr<TaskTypes> TaskTypesFactory::make(const std::string& json){
    auto json_object = JSON::parseObject(json);
    if(json_object.isValid()){
        auto task_types = make(json_object);
        JSON::deleteObject(json_object);
        return task_types;
    }
    return NULL;
}

std::unique_ptr<TaskTypes> TaskTypesFactory::make(JsonObject& json){
    if(!json.hasItem("task_types")){
        return NULL;
    }
    auto task_types = std::make_unique<TaskTypesSet>();
    auto task_types_array = json.getArray("task_types");
    for (size_t i = 0; i < task_types_array.size(); i++){
        const auto& type = task_types_array.getString(i);
        task_types->addTaskTypes(type);
    }
    return task_types;
}
