#include <ObstructedAreaFactory.h>
#include <ObstructedAreaCollection.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>
#include <GeometricObjectFactory.h>

PtrObstructedArea ObstructedAreaFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto obstructed_area = make(json);
    JSON::deleteObject(json);
    return obstructed_area;
}

static PtrObstructedArea makeGeometricObstructedArea(JsonObject& json){
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
    return std::make_unique<ObstructedAreaCollection>(obstacles);
}

PtrObstructedArea ObstructedAreaFactory::make(JsonObject& json){
    if(!json.hasItem(OBSTRUCTED_AREA)){
        return nullptr;
    }
    auto obstructed_area_json_object = json.getObject(OBSTRUCTED_AREA);
    auto type = obstructed_area_json_object.getString("type");

    if( type == "geometric"){
        return makeGeometricObstructedArea(obstructed_area_json_object);
    }else{
        return nullptr;
    }
    return nullptr;
}
