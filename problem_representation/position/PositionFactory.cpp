#include <PositionFactory.h>
#include <Position.h>
#include <limits>
#include <iostream>

#define COORDINATE_TYPE "type"
#define COORDINATE_2D "2d"
#define COORDINATE_X "x"
#define COORDINATE_Y "y"

Position2d PositionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto robot = make(json);
    JSON::deleteObject(json);
    return robot;
}

Position2d PositionFactory::make(JsonObject& json){
    if(!json.hasItem(COORDINATE_TYPE) || !json.hasItem(COORDINATE_X) || !json.hasItem(COORDINATE_Y)){
        return {0,0};
    }
    
    auto type = json.getString(COORDINATE_TYPE);
    if ( type != COORDINATE_2D ){
        return {0,0};
    }
    auto coordinate_x = json.getDouble(COORDINATE_X);
    auto coordinate_y = json.getDouble(COORDINATE_Y);

    return {coordinate_x, coordinate_y};
}
