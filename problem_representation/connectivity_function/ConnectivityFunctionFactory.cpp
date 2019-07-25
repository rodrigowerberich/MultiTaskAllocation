#include <ConnectivityFunctionFactory.h>
#include <limits>
#include <iostream>
#include <ProblemJsonNamesDefinitions.h>
#include <GeometricObjectFactory.h>
#include <AreaFactory.h>

PtrConnectivityFunction ConnectivityFunctionFactory::make(const std::string& json_str){
    auto json = JSON::parseObject(json_str);
    auto connectivity_function = make(json);
    JSON::deleteObject(json);
    return connectivity_function;
}

PtrConnectivityFunction ConnectivityFunctionFactory::make(JsonObject& json){
    if(!json.hasItem(CONNECTIVITY_FUNCTION)){
        return nullptr;
    }
    auto connectivity_function_json_object = json.getObject(CONNECTIVITY_FUNCTION);
    auto area = AreaFactory::make(connectivity_function_json_object);
    if(area){
        return std::make_unique<ConnectivityFunction>(std::move(area));
    }
    return nullptr;
}
