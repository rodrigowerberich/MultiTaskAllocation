#include <AreaFactory.h>
#include <GeometricArea.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>
#include <GeometricObjectFactory.h>

PtrArea AreaFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto obstructed_area = make(json);
    JSON::deleteObject(json);
    return obstructed_area;
}

static PtrArea makeGeometricArea(JsonObject& json){
    if(!json.hasItem("obstacles")){
        return nullptr;
    }
    auto obstacles_json_array = json.getArray("obstacles");
    std::vector<PtrGeometricObject> obstacles;
    for (size_t i = 0; i < obstacles_json_array.size(); i++){
        auto obstacle_json = obstacles_json_array.getObject(i);
        auto geometric_obstacle = GeometricObjectFactory::make(obstacle_json);
        if(geometric_obstacle){
            obstacles.push_back(std::move(geometric_obstacle));
        }else{
            return nullptr;
        }
    }
    return std::make_unique<GeometricArea>(obstacles);
}

PtrArea AreaFactory::make(JsonObject& json){
    if(!json.hasItem("type")){
        return nullptr;
    }
    auto type = json.getString("type");

    if( type == "geometric"){
        return makeGeometricArea(json);
    }else{
        return nullptr;
    }
    return nullptr;
}
