#include <EffortFunctionFactory.h>
#include <EffortFunctionMap.h>
#include <limits>

std::unique_ptr<EffortFunction> EffortFunctionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto effort_function = make(json);
    JSON::deleteObject(json);
    return effort_function;
}

static bool is_effort_function_json(const JsonObject& json_object){
    return  json_object.isValid() &&
            json_object.hasItem("task_type") && 
            json_object.hasItem("robot_type") &&
            json_object.hasItem("function");
}

static float parse_function_json(const JsonObject& json_object){
    if(!json_object.isValid() || !json_object.hasItem("type")){
        return std::numeric_limits<float>::quiet_NaN();
    }
    auto type = json_object.getString("type");
    if(type == "simple_value"){
        return json_object.getDouble("value");
    }else if(type == "infinity"){
        return std::numeric_limits<float>::infinity();
    }
    return std::numeric_limits<float>::quiet_NaN();    
}

std::unique_ptr<EffortFunction> EffortFunctionFactory::make(JsonObject& json){
    if(!json.hasItem("effort_function")){
        return nullptr;
    }
    auto json_array = json.getArray("effort_function");

    if(json_array.size() == 0){
        return nullptr;
    }
    auto effort_function = std::make_unique<EffortFunctionMap>();
    for(size_t i=0; i < json_array.size(); i++){
        auto effort_json_object = json_array.getObject(i);
        if(is_effort_function_json(effort_json_object)){
            auto task_type = effort_json_object.getString("task_type");
            auto robot_type = effort_json_object.getString("robot_type");
            auto function_value = parse_function_json(effort_json_object.getObject("function"));
            if(function_value != std::numeric_limits<float>::quiet_NaN()){
                effort_function->addMapping(task_type, robot_type, function_value);
            }
        }
    }
    return effort_function;
}
