#include <RewardFunctionFactory.h>
#include <RewardFunctionMap.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>


std::unique_ptr<RewardFunction> RewardFunctionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto reward_function = make(json);
    JSON::deleteObject(json);
    return reward_function;
}

static float parse_function_json(const JsonObject& json_object, const std::string& name){
    auto string_value = json_object.getString(name);
    if( string_value == "infinity" ){
        return std::numeric_limits<float>::infinity();
    }else{
        return json_object.getDouble(name);
    }
}

std::unique_ptr<RewardFunction> RewardFunctionFactory::make(JsonObject& json){
    if(!json.hasItem(REWARD_FUNCTION) || !json.hasItem(TASK_TYPES)){
        return nullptr;
    }
    auto task_type_json_array = json.getArray(TASK_TYPES);
    auto reward_function_json_object = json.getObject(REWARD_FUNCTION);
    auto reward_function = std::make_unique<RewardFunctionMap>();
    for (size_t i = 0; i < task_type_json_array.size(); i++) {
        auto task_type = task_type_json_array.getString(i);
        auto value = parse_function_json(reward_function_json_object, task_type);
        reward_function->addMapping(task_type, value);
    }
    return reward_function;
}
