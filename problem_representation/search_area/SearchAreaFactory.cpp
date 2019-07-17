#include <SearchAreaFactory.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>


std::unique_ptr<SearchArea> SearchAreaFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto search_area = make(json);
    JSON::deleteObject(json);
    return search_area;
}

std::unique_ptr<SearchArea> SearchAreaFactory::make(JsonObject& json){
    // TODO Implementar
    // if(!json.hasItem(REWARD_FUNCTION) || !json.hasItem(TASK_TYPES)){
    //     return nullptr;
    // }
    // auto task_type_json_array = json.getArray(TASK_TYPES);
    // auto search_area_json_object = json.getObject(REWARD_FUNCTION);
    // auto search_area = std::make_unique<SearchAreaMap>();
    // for (size_t i = 0; i < task_type_json_array.size(); i++) {
    //     auto task_type = task_type_json_array.getString(i);
    //     auto value = parse_function_json(search_area_json_object, task_type);
    //     search_area->addMapping(task_type, value);
    // }
    // return search_area;
}
