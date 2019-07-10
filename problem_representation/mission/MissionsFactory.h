#ifndef MISSIONSFACTORY_H__
#define MISSIONSFACTORY_H__

#include <Missions.h>
#include <JSON.h>
#include <memory>

class MissionsFactory{
public:
    static std::unique_ptr<Missions> make(const std::string& json);
    static std::unique_ptr<Missions> make(JsonObject& json);
};

#endif //MISSIONSFACTORY_H__