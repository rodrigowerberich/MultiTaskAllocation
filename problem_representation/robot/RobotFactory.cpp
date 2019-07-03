#include <RobotFactory.h>
#include <Robot2d.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>

#define ROBOT_NAME "name"
#define ROBOT_TYPE "type"
#define ROBOT_POSITION "position"

std::unique_ptr<Robot> RobotFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto robot = make(json);
    JSON::deleteObject(json);
    return robot;
}

std::unique_ptr<Robot> RobotFactory::make(JsonObject& json){
    if(!json.hasItem(ROBOT_NAME) || !json.hasItem(ROBOT_TYPE) || !json.hasItem(ROBOT_POSITION)){
        return nullptr;
    }
    
    auto robot_name = json.getString(ROBOT_NAME);
    auto robot_type = json.getString(ROBOT_TYPE);
    auto robot_position_json = json.getObject(ROBOT_POSITION);
    auto robot_position = PositionFactory::make(robot_position_json);

    return std::make_unique<Robot2d>(robot_name, robot_type, robot_position);
}
