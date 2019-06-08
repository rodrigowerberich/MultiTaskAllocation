#ifndef PROBLEMREPRESENTATION_H__
#define PROBLEMREPRESENTATION_H__

#include <string>
#include <RobotTypes.h>
#include <TaskTypes.h>
#include <memory>

class ProblemRepresentation{
private:
    std::unique_ptr<RobotTypes> m_robot_types;
    std::unique_ptr<TaskTypes> m_task_types;
    // EffortFunction m_effort_function;
    // RewardFunction m_reward_function;
    // Robots m_robots;
    // Tasks m_tasks;
    // Missions m_missions;
    // SearchArea m_search_area;
    // Connectivity_function m_connectivity_function;
public:
    ProblemRepresentation(std::string file_name);
    ~ProblemRepresentation();
};

ProblemRepresentation::ProblemRepresentation(std::string file_name){
}

ProblemRepresentation::~ProblemRepresentation()
{
}


#endif //PROBLEMREPRESENTATION_H__