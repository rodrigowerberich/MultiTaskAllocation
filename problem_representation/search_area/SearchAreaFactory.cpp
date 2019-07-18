#include <SearchAreaFactory.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>
#include <GeometricObjectFactory.h>


std::unique_ptr<SearchArea> SearchAreaFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto search_area = make(json);
    JSON::deleteObject(json);
    return search_area;
}

std::unique_ptr<SearchArea> SearchAreaFactory::make(JsonObject& json){
    if(!json.hasItem(SEARCH_AREA)){
        return nullptr;
    }
    auto search_area_json = json.getObject(SEARCH_AREA);
    auto geometric = GeometricObjectFactory::make(search_area_json);
    
    if(geometric){
        return std::make_unique<SearchArea>(std::move(geometric));
    }else{
        return nullptr;
    }
}
