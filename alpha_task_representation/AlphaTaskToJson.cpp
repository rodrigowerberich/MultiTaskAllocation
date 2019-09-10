#ifndef ALPHATASKTOJSON_CPP__
#define ALPHATASKTOJSON_CPP__

#include <AlphaTaskToJson.h>
#include <JSON.h>

namespace jsonifier{

std::string prettify(const std::string& json){
    auto json_object = JSON::parseObject(json);
    if(json_object.isValid()){
        auto pretty = json_object.toString();
        JSON::deleteObject(json_object);
        return pretty;
    }else{
        JSON::deleteObject(json_object);
        auto json_array = JSON::parseArray(json);
        auto pretty = json_array.toString();
        JSON::deleteArray(json_array);
        return pretty;
    }
}


template <> std::string toJson(const AlphaTask& alpha_task){
    auto json = JSON::createObject();
    json.setString("task", alpha_task.getTask());
    auto positions_array = JSON::createArray();
    for(const auto& p: alpha_task){
        auto position_json = JSON::createObject();
        position_json.setString("type", "2d");
        position_json.setDouble("x", p.x);
        position_json.setDouble("y", p.y);

        positions_array.addObject(position_json);
    }
    json.setArray("positions", positions_array);
    auto json_representation = json.toStringUnformatted();
    JSON::deleteObject(json);
    return json_representation;
}

template <> std::string toJson(const AlphaTasks& alpha_tasks){
    auto json_array = JSON::createArray();
    for(const auto& alpha_task: alpha_tasks){
        json_array.addObject(JSON::parseObject(toJson(alpha_task)));
    }
    auto json = JSON::createObject();
    json.setArray("alpha_tasks", json_array);
    auto json_representation = json.toStringUnformatted();
    JSON::deleteObject(json);
    return json_representation;
}

}

#endif //ALPHATASKTOJSON_CPP__