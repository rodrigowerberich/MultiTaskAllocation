#ifndef PROBLEMREPRESENTATION_H__
#define PROBLEMREPRESENTATION_H__

#include <string>
#include <RobotTypes.h>
#include <TaskTypes.h>
#include <EffortFunction.h>
#include <RewardFunction.h>
#include <Robots.h>
#include <Tasks.h>
#include <Missions.h>
#include <SearchArea.h>
#include <memory>
#include <JSON.h>

class ProblemRepresentation{
private:
    bool parseJsonRepresentation(JsonObject json_object);
    std::unique_ptr<RobotTypes> m_robot_types;
    std::unique_ptr<TaskTypes> m_task_types;
    std::unique_ptr<EffortFunction> m_effort_function;
    std::unique_ptr<RewardFunction> m_reward_function;
    std::unique_ptr<Robots> m_robots;
    std::unique_ptr<Tasks> m_tasks;
    std::unique_ptr<Missions> m_missions;
    std::unique_ptr<SearchArea> m_search_area;
    // Connectivity_function m_connectivity_function;
    bool m_is_valid;
public:
    ProblemRepresentation(std::string file_name);
    ~ProblemRepresentation(){}
    bool isValid() const;
    const std::unique_ptr<RobotTypes>& getRobotTypes() const;
    const std::unique_ptr<TaskTypes>& getTaskTypes() const;
    const std::unique_ptr<EffortFunction>& getEffortFunction() const;
    const std::unique_ptr<RewardFunction>& getRewardFunction() const;
    const std::unique_ptr<Robots>& getRobots() const;
    const std::unique_ptr<Tasks>& getTasks() const;
    const std::unique_ptr<Missions>& getMissions() const;
    const std::unique_ptr<SearchArea>& getSearchArea() const;    
};

#endif //PROBLEMREPRESENTATION_H__