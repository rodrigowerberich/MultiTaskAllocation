#include <RobotsFactory.h>
#include <RobotFactory.h>
#include <Robots.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>
#include <ProblemJsonNamesDefinitions.h>

std::unique_ptr<Robots> RobotsFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto robot = make(json);
    JSON::deleteObject(json);
    return robot;
}

std::unique_ptr<Robots> RobotsFactory::make(JsonObject& json){
    if(!json.hasItem(ROBOTS)){
        return nullptr;
    }
    auto robots_array = json.getArray(ROBOTS);
    auto robots = std::make_unique<Robots>();

    for (size_t i = 0; i < robots_array.size(); i++){
        auto robot_json = robots_array.getObject(i);
        auto robot = RobotFactory::make(robot_json);
        if( robot == nullptr ) {
            return nullptr;
        }else{
            robots->push_back(std::move(robot));
        }
    }
    return robots;
}