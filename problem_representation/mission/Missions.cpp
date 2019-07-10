#include <Missions.h>

const std::unique_ptr<Mission> & Missions::getMission(const std::string& mission_id){
    return getValueByName(mission_id);
}