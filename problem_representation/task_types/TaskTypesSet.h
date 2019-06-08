#ifndef TASKTYPESSET_H__
#define TASKTYPESSET_H__

#include <TaskTypes.h>
#include <set>

class TaskTypesSet: public TaskTypes{
private:
    std::set<RobotType> m_Task_types;
public:
    std::vector<RobotType> getTaskTypes();
    bool hasTaskType(const RobotType& type);
    bool addTaskTypes(const RobotType& type);
};


#endif //TASKTYPESSET_H__