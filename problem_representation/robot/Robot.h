#ifndef ROBOT_H__
#define ROBOT_H__

#include <string>
#include <RobotTypes.h>
#include <Position.h>

class Robot{
public:
    virtual const std::string & getName() const = 0;
    virtual const RobotType & getType() const = 0;
    virtual const Position2d & getPosition() const = 0;
    ~Robot(){}
};
#endif // ROBOT_H__