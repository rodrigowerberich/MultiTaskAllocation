#include <ObstructedAreaFactory.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>
#include <GeometricObjectFactory.h>
#include <AreaFactory.h>

PtrObstructedArea ObstructedAreaFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto obstructed_area = make(json);
    JSON::deleteObject(json);
    return obstructed_area;
}

PtrObstructedArea ObstructedAreaFactory::make(JsonObject& json){
    if(!json.hasItem(OBSTRUCTED_AREA)){
        return nullptr;
    }
    auto obstructed_area_json_object = json.getObject(OBSTRUCTED_AREA);
    auto area = AreaFactory::make(obstructed_area_json_object);
    if(area){
        return std::make_unique<ObstructedArea>(std::move(area));
    }
    return nullptr;
}
