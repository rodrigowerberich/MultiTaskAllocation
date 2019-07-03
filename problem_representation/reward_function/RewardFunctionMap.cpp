#include <RewardFunctionMap.h>
#include <limits>
#include <iostream>

float RewardFunctionMap::operator()(const TaskType& task_type) const{
    if(m_reward_map.count(task_type) > 0){
        return m_reward_map.find(task_type)->second;
    }
    return std::numeric_limits<float>::quiet_NaN();
}

bool RewardFunctionMap::addMapping(const TaskType& task_type, float value){
    m_reward_map[task_type] = value;
    return true;
}
