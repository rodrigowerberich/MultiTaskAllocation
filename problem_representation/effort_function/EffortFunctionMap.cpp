#include <EffortFunctionMap.h>
#include <limits>

float EffortFunctionMap::operator()(const TaskType& task_type, const RobotType& robot_type){
    if(m_effort_map.count(task_type) > 0){
        if(m_effort_map[task_type].count(task_type) > 0){
            return m_effort_map[task_type][robot_type];
        }
    }else{
        return std::numeric_limits<float>::quiet_NaN();
    }
}
