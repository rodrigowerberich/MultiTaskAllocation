#ifndef REWARDFUNCTION_H__
#define REWARDFUNCTION_H__

#include <TaskTypes.h>

class RewardFunction{
public:
    virtual float operator()(const TaskType& task_type) const = 0;
    ~RewardFunction(){}
};


#endif //REWARDFUNCTION_H__