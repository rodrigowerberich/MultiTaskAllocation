#ifndef TASKTYPES_H__
#define TASKTYPES_H__

#include <vector>
#include <string>

typedef std::string TaskType;

class TaskTypes{
public:
    virtual std::vector<TaskType> getTaskTypes() = 0;
    virtual bool hasTaskType(const TaskType& type) = 0;
    virtual bool addTaskTypes(const TaskType& type) = 0;
    ~TaskTypes(){}
};


#endif //TASKTYPES_H__