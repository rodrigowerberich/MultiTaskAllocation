#ifndef ROBOTTYPESSET_H__
#define ROBOTTYPESSET_H__

#include <RobotTypes.h>
#include <set>

class RobotTypesSet: public RobotTypes{
private:
    std::set<RobotType> m_robot_types;
public:
    std::vector<RobotType> getRobotTypes();
    bool hasRobotType(const RobotType& type);
    bool addRobotTypes(const RobotType& type);
};

#endif //ROBOTTYPESSET_H__