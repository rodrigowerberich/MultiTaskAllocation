#ifndef TASKTYPESSET_H__
#define TASKTYPESSET_H__

#include <TaskTypes.h>
#include <set>

class TaskTypesSet: public TaskTypes{
private:
    std::set<TaskType> m_Task_types;
public:
    std::vector<TaskType> getTaskTypes();
    bool hasTaskType(const TaskType& type);
    bool addTaskTypes(const TaskType& type);
};


#endif //TASKTYPESSET_H__