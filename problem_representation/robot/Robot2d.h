#ifndef ROBOT_2D_H__
#define ROBOT_2D_H__

#include <Robot.h>

class Robot2d: public Robot {
private:
    std::string m_name;
    RobotType m_type;
    Position2d m_position;
public:
    Robot2d(const std::string & name, const RobotType & type, const Position2d & position);
    const std::string & getName() const;
    const RobotType & getType() const;
    const Position2d & getPosition() const;
};

#endif