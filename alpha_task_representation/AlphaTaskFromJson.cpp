#include <AlphaTaskToJson.h>
#include <AlphaTasks.h>
#include <JSON.h>
#include <PositionFactory.h>

namespace jsonifier{

template <> AlphaTask fromJson(const std::string& json){
    constexpr auto task_item_name = "task";
    constexpr auto positions_item_name = "positions";
    auto json_object = JSON::parseObject(json);
    if (!json_object.isValid() || !json_object.hasItem(task_item_name) || !json_object.hasItem(positions_item_name)){
        JSON::deleteObject(json_object);
        return {};
    }
    std::vector<Position2d> positions;
    auto json_array = json_object.getArray(positions_item_name);
    for(size_t i = 0; i < json_array.size(); i++){
        auto position_json_object = json_array.getObject(i);
        positions.push_back(PositionFactory::make(position_json_object));
    }
    AlphaTask new_alpha_task { json_object.getString(task_item_name), positions };
    JSON::deleteObject(json_object);
    return new_alpha_task;
}

template <> AlphaTasks fromJson(const std::string& json){
    constexpr auto alpha_tasks_item = "alpha_tasks";
    auto json_object = JSON::parseObject(json);
    if (!json_object.isValid() || !json_object.hasItem(alpha_tasks_item)){
        JSON::deleteObject(json_object);
        return {};
    }
    AlphaTasks new_alpha_tasks;
    auto json_array = json_object.getArray(alpha_tasks_item);
    for(size_t i = 0; i < json_array.size(); i++){
        new_alpha_tasks.push_back(fromJson<AlphaTask>(json_array.getObject(i).toStringUnformatted()));
    }
    JSON::deleteObject(json_object);
    return new_alpha_tasks;
}

}