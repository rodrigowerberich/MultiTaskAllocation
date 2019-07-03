#ifndef POSITIONFACTORY_H__
#define POSITIONFACTORY_H__

#include <Position.h>
#include <JSON.h>

class PositionFactory{
public:
    static Position2d make(const std::string& json);
    static Position2d make(JsonObject& json);
};

#endif //POSITIONFACTORY_H__