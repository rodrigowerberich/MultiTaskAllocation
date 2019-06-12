#ifndef EFFORTFUNCTIONMAP_H__
#define EFFORTFUNCTIONMAP_H__

#include <map>
#include <EffortFunction.h>

class EffortFunctionMap: public EffortFunction{
private:
    std::map<TaskType, std::map<RobotType, float>> m_effort_map;
public:
    float operator()(const TaskType& task_type, const RobotType& robot_type) const;
    bool addMapping(const TaskType& task_type, const RobotType& robot_type, float value);
};

#endif //EFFORTFUNCTIONMAP_H__