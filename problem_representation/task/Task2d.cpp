#include <Task2d.h>

Task2d::Task2d(const std::string & name, const TaskType & type, const Position2d & position, const std::set<std::string> & prerequisites):
    m_name {name},
    m_type {type},
    m_position {position},
    m_prerequisites {prerequisites}
 {}
const std::string & Task2d::getName() const{
    return m_name;
}
const TaskType & Task2d::getType() const{
    return m_type;
}
const Position2d & Task2d::getPosition() const{
    return m_position;
}

const std::set<std::string> & Task2d::getPrerequisites() const{
    return m_prerequisites;
}

bool Task2d::hasPrerequisite(const std::string& task) const{
    return m_prerequisites.count(task) > 0;
}
