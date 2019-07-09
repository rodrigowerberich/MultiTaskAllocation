#ifndef TASK_2D_H__
#define TASK_2D_H__

#include <Task.h>

class Task2d: public Task {
private:
    std::string m_name;
    TaskType m_type;
    Position2d m_position;
    std::set<std::string> m_prerequisites;
public:
    Task2d(const std::string & name, const TaskType & type, const Position2d & position, const std::set<std::string> & prerequisites);
    const std::string & getName() const;
    const TaskType & getType() const;
    const Position2d & getPosition() const;
    const std::set<std::string> & getPrerequisites() const;
    bool hasPrerequisite(const std::string& task) const;
};

#endif