#ifndef TASK_H__
#define TASK_H__

#include <string>
#include <TaskTypes.h>
#include <Position.h>
#include <set>

class Task{
public:
    virtual const std::string & getId() const = 0;
    virtual const TaskType & getType() const = 0;
    virtual const Position2d & getPosition() const = 0;
    virtual const std::set<std::string> & getPrerequisites() const = 0;
    virtual bool hasPrerequisite(const std::string& task) const = 0;   
    ~Task(){}
};
#endif // TASK_H__