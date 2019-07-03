#include <EffortFunctionFactory.h>
#include <EffortFunctionMap.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>

std::unique_ptr<EffortFunction> EffortFunctionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto effort_function = make(json);
    JSON::deleteObject(json);
    return effort_function;
}

static float parse_function_json(const JsonObject& json_object, const std::string& name){
    auto string_value = json_object.getString(name);
    if( string_value == "infinity" ){
        return std::numeric_limits<float>::infinity();
    }else{
        return json_object.getDouble(name);
    }
}

std::unique_ptr<EffortFunction> EffortFunctionFactory::make(JsonObject& json){
    if(!json.hasItem(EFFORT_FUNCTION) || !json.hasItem(ROBOT_TYPES) || !json.hasItem(TASK_TYPES)){
        return nullptr;
    }
    auto task_type_json_array = json.getArray(TASK_TYPES);
    auto robot_type_json_array = json.getArray(ROBOT_TYPES);
    auto effort_function_json_object = json.getObject(EFFORT_FUNCTION);
    auto effort_function = std::make_unique<EffortFunctionMap>();
    for (size_t i = 0; i < task_type_json_array.size(); i++) {
        auto task_type = task_type_json_array.getString(i);
        auto json_object =  effort_function_json_object.getObject(task_type);
        for (size_t j = 0; j < robot_type_json_array.size(); j++) {
            auto robot_type = robot_type_json_array.getString(j);
            auto value = parse_function_json(json_object, robot_type);
            effort_function->addMapping(task_type, robot_type, value);
        }
    }
    return effort_function;
}
