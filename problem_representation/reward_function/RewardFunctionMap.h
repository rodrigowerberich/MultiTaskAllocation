#ifndef REWARDFUNCTIONMAP_H__
#define REWARDFUNCTIONMAP_H__

#include <map>
#include <RewardFunction.h>

class RewardFunctionMap: public RewardFunction{
private:
    std::map<TaskType, float> m_reward_map;
public:
    float operator()(const TaskType& task_type) const;
    bool addMapping(const TaskType& task_type, float value);
};

#endif //REWARDFUNCTIONMAP_H__