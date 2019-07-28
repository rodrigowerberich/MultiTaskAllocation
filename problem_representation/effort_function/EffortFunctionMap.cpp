#include <EffortFunctionMap.h>
#include <limits>
#include <iostream>

float EffortFunctionMap::operator()(const TaskType& task_type, const RobotType& robot_type) const{
    if(m_effort_map.count(task_type) > 0){
        if(m_effort_map.find(task_type)->second.count(robot_type) > 0){
            return m_effort_map.find(task_type)->second.find(robot_type)->second;
        }
    }
    return -1;
}

bool EffortFunctionMap::addMapping(const TaskType& task_type, const RobotType& robot_type, float value){
    if(value < 0){
        return false;
    }
    m_effort_map[task_type][robot_type] = value;
    return true;
}
