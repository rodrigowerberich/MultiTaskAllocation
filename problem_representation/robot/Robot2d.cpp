#include <Robot2d.h>

Robot2d::Robot2d(const std::string & name, const RobotType & type, const Position2d & position):
    m_name {name},
    m_type {type},
    m_position {position}
 {}
const std::string & Robot2d::getName() const{
    return m_name;
}
const RobotType & Robot2d::getType() const{
    return m_type;
}
const Position2d & Robot2d::getPosition() const{
    return m_position;
}