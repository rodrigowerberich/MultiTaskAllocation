#include <RobotTypesFactory.h>
#include <JSON.h>
#include <RobotTypesSet.h>

std::unique_ptr<RobotTypes> RobotTypesFactory::make(const std::string& json){
    auto json_object = JSON::parseObject(json);
    if(json_object.isValid()){
        auto robot_types = make(json_object);
        JSON::deleteObject(json_object);
        return robot_types;
    }
    return NULL;
}

std::unique_ptr<RobotTypes> RobotTypesFactory::make(JsonObject& json){
    if(!json.hasItem("robot_types")){
        return NULL;
    }
    auto robot_types = std::make_unique<RobotTypesSet>();
    auto robot_types_array = json.getArray("robot_types");
    for (size_t i = 0; i < robot_types_array.size(); i++){
        const auto& type = robot_types_array.getString(i);
        robot_types->addRobotTypes(type);
    }
    return robot_types;
}
