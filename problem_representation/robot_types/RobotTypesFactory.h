#ifndef ROBOTTYPESFACTORY_H__
#define ROBOTTYPESFACTORY_H__

#include <RobotTypes.h>
#include <JSON.h>
#include <memory>

class RobotTypesFactory{
public:
    static std::unique_ptr<RobotTypes> make(const std::string& json);
    static std::unique_ptr<RobotTypes> make(JsonObject& json);
};

#endif //ROBOTTYPESFACTORY_H__