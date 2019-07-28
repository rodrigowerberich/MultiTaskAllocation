#include <RewardFunctionMap.h>
#include <limits>
#include <iostream>

float RewardFunctionMap::operator()(const TaskType& task_type) const{
    if(m_reward_map.count(task_type) > 0){
        return m_reward_map.find(task_type)->second;
    }
    return -1;
}

bool RewardFunctionMap::addMapping(const TaskType& task_type, float value){
    if(value < 0){
        return false;
    }
    m_reward_map[task_type] = value;
    return true;
}
