#ifndef EFFORTFUNCTION_H__
#define EFFORTFUNCTION_H__

#include <RobotTypes.h>
#include <TaskTypes.h>

class EffortFunction{
public:
    virtual float operator()(const TaskType& task_type, const RobotType& robot_type) const = 0;
    ~EffortFunction(){}
};


#endif //EFFORTFUNCTION_H__