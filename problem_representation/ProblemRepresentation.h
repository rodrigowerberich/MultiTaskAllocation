#ifndef PROBLEMREPRESENTATION_H__
#define PROBLEMREPRESENTATION_H__

#include <string>
#include <RobotTypes.h>
#include <TaskTypes.h>
#include <EffortFunction.h>
#include <RewardFunction.h>
#include <memory>
#include <JSON.h>

class ProblemRepresentation{
private:
    bool parseJsonRepresentation(JsonObject json_object);
    std::unique_ptr<RobotTypes> m_robot_types;
    std::unique_ptr<TaskTypes> m_task_types;
    std::unique_ptr<EffortFunction> m_effort_function;
    std::unique_ptr<RewardFunction> m_reward_function;
    // Robots m_robots;
    // Tasks m_tasks;
    // Missions m_missions;
    // SearchArea m_search_area;
    // Connectivity_function m_connectivity_function;
    bool m_is_valid;
public:
    ProblemRepresentation(std::string file_name);
    ~ProblemRepresentation(){}
    bool isValid();
    const std::unique_ptr<RobotTypes>& getRobotTypes();
    const std::unique_ptr<TaskTypes>& getTaskTypes();
    const std::unique_ptr<EffortFunction>& getEffortFunction();
    const std::unique_ptr<RewardFunction>& getRewardFunction();
};

#endif //PROBLEMREPRESENTATION_H__