// #ifndef ALPHATASKREPRESENTATION_H__
// #define ALPHATASKREPRESENTATION_H__

// #ifndef PROBLEMREPRESENTATION_H__
// #define PROBLEMREPRESENTATION_H__

// #include <string>
// #include <memory>
// #include <JSON.h>

// class ProblemRepresentation{
// private:
//     bool parseJsonRepresentation(JsonObject json_object);
//     std::unique_ptr<AlphaTasks> m_alpha_tasks;
//     std::unique_ptr<RobotTypes> m_robot_types;
//     std::unique_ptr<TaskTypes> m_task_types;
//     std::unique_ptr<EffortFunction> m_effort_function;
//     std::unique_ptr<RewardFunction> m_reward_function;
//     std::unique_ptr<Robots> m_robots;
//     std::unique_ptr<Tasks> m_tasks;
//     std::unique_ptr<Missions> m_missions;
//     std::unique_ptr<SearchArea> m_search_area;
//     std::unique_ptr<ObstructedArea> m_obstructed_area;
//     std::unique_ptr<ConnectivityFunction> m_connectivity_function;
//     void evaluateProblemRepresentation();
//     bool m_is_valid;
//     std::string m_error_message;
// public:
//     ProblemRepresentation(std::string file_name);
//     ~ProblemRepresentation(){}
//     bool isValid() const;
//     const std::unique_ptr<RobotTypes>& getRobotTypes() const;
//     const std::unique_ptr<TaskTypes>& getTaskTypes() const;
//     const std::unique_ptr<EffortFunction>& getEffortFunction() const;
//     const std::unique_ptr<RewardFunction>& getRewardFunction() const;
//     const std::unique_ptr<Robots>& getRobots() const;
//     const std::unique_ptr<Tasks>& getTasks() const;
//     const std::unique_ptr<Missions>& getMissions() const;
//     const std::unique_ptr<SearchArea>& getSearchArea() const;
//     const std::unique_ptr<ObstructedArea>& getObstructedArea() const;
//     const std::unique_ptr<ConnectivityFunction>& getConnectivityFunction() const;
//     const std::string& getErrorMessage() const;
//     template <typename RendererType>
//     void draw(const RendererType & renderer) const{
//         renderer.draw(m_search_area);
//         renderer.draw(m_obstructed_area);
//         renderer.draw(m_robots);
//         renderer.draw(m_tasks);
//         renderer.draw(m_connectivity_function);
//     }
// };

// #endif //PROBLEMREPRESENTATION_H__

// #endif //ALPHATASKREPRESENTATION_H__