#ifndef ROBOTFACTORY_H__
#define ROBOTFACTORY_H__

#include <Robot.h>
#include <JSON.h>
#include <memory>

class RobotFactory{
public:
    static std::unique_ptr<Robot> make(const std::string& json);
    static std::unique_ptr<Robot> make(JsonObject& json);
};

#endif //ROBOTFACTORY_H__