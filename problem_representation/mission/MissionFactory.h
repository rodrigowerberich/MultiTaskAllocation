#ifndef MISSIONFACTORY_H__
#define MISSIONFACTORY_H__

#include <Mission.h>
#include <JSON.h>
#include <memory>

class MissionFactory{
public:
    static std::unique_ptr<Mission> make(const std::string& json);
    static std::unique_ptr<Mission> make(JsonObject& json);
};

#endif //MISSIONFACTORY_H__