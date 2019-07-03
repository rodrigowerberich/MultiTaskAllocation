#ifndef ROBOTSFACTORY_H__
#define ROBOTSFACTORY_H__

#include <Robots.h>
#include <JSON.h>
#include <memory>

class RobotsFactory{
public:
    static std::unique_ptr<Robots> make(const std::string& json);
    static std::unique_ptr<Robots> make(JsonObject& json);
};

#endif //ROBOTSFACTORY_H__