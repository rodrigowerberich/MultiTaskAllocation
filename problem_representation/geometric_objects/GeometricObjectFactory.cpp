#include <GeometricObjectFactory.h>
#include <CircleGeometricObject.h>
#include <RectangleGeometricObject.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>


std::unique_ptr<GeometricObject> GeometricObjectFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto reward_function = make(json);
    JSON::deleteObject(json);
    return reward_function;
}

static std::unique_ptr<GeometricObject> make_rectangle(JsonObject& json){
    auto does_not_have_position_key =   !json.hasItem("bottom_left") && 
                                        !json.hasItem("top_left") && 
                                        !json.hasItem("bottom_right") && 
                                        !json.hasItem("top_right") && 
                                        !json.hasItem("center"); 
                                        
    if(does_not_have_position_key || !json.hasItem("width") || !json.hasItem("height")){
        return nullptr;
    }
    auto width = json.getDouble("width");
    auto height = json.getDouble("height");
    Position2d bottom_left{0};
    if( json.hasItem("bottom_left") ){
        bottom_left = PositionFactory::make(json.getObject("bottom_left"));
    }else if( json.hasItem("top_left") ){
        bottom_left = PositionFactory::make(json.getObject("top_left"));
        bottom_left = bottom_left - Position2d{height,0}; 
    }else if( json.hasItem("bottom_right") ){
        bottom_left = PositionFactory::make(json.getObject("top_left"));
        bottom_left = bottom_left - Position2d{0,width}; 
    }else if( json.hasItem("top_right") ){
        bottom_left = PositionFactory::make(json.getObject("top_left"));
        bottom_left = bottom_left - Position2d{height,width}; 
    }else if( json.hasItem("center") ){
        bottom_left = PositionFactory::make(json.getObject("center"));
        bottom_left = bottom_left - Position2d{height/2.0,width/2.0}; 
    }else{ 
        return nullptr;
    }
    return std::make_unique<RectangleGeometricObject>(bottom_left, width, height);
}

static std::unique_ptr<GeometricObject> make_circle(JsonObject& json){
    if(!json.hasItem("center") || !json.hasItem("radius")){
        return nullptr;
    }
    auto center = PositionFactory::make(json.getObject("center"));
    auto radius = json.getDouble("radius");
    return std::make_unique<CircleGeometricObject>(center, radius);
}

std::unique_ptr<GeometricObject> GeometricObjectFactory::make(JsonObject& json){
    if(!json.hasItem("type")){
        return nullptr;
    }
    auto geometric_object_type = json.getString("type");
    if( geometric_object_type == "rectangle" ){
        return make_rectangle(json);
    }else if( geometric_object_type == "circle" ){
        return make_circle(json);
    }
    return nullptr;
}
