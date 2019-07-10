#include <MissionsFactory.h>
#include <MissionFactory.h>
#include <Missions.h>
#include <limits>
#include <iostream>
#include <PositionFactory.h>
#include <ProblemJsonNamesDefinitions.h>

std::unique_ptr<Missions> MissionsFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto mission = make(json);
    JSON::deleteObject(json);
    return mission;
}

std::unique_ptr<Missions> MissionsFactory::make(JsonObject& json){
    if(!json.hasItem(MISSIONS)){
        std::cout << "Failed here 1\n";
        return nullptr;
    }
    auto missions_array = json.getArray(MISSIONS);
    auto missions = std::make_unique<Missions>();

    for (size_t i = 0; i < missions_array.size(); i++){
        auto mission_json = missions_array.getObject(i);
        auto mission = MissionFactory::make(mission_json);
        if( mission == nullptr ) {
            std::cout << "Failed here 2\n";
            return nullptr;
        }else{
            missions->push_back(std::move(mission));
        }
    }
    return missions;
}