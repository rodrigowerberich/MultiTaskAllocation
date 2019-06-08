#ifndef ROBOTTYPES_H__
#define ROBOTTYPES_H__

#include <vector>
#include <string>

typedef std::string RobotType;

class RobotTypes{
public:
    virtual std::vector<RobotType> getRobotTypes() = 0;
    virtual bool hasRobotType(const RobotType& type) = 0;
    virtual bool addRobotTypes(const RobotType& type) = 0;
    ~RobotTypes(){}
};


#endif //ROBOTTYPES_H__