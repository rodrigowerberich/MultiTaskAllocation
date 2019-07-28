#include <ProblemRepresentation.h>
#include <fstream>
#include <iostream>
#include <JSON.h>
#include <RobotTypesFactory.h>
#include <TaskTypesFactory.h>
#include <EffortFunctionFactory.h>
#include <RewardFunctionFactory.h>
#include <RobotsFactory.h>
#include <TasksFactory.h>
#include <MissionsFactory.h>
#include <SearchAreaFactory.h>
#include <ObstructedAreaFactory.h>
#include <ConnectivityFunctionFactory.h>
#include <algorithm>
#include <ProblemJsonNamesDefinitions.h>
#include <tuple>

static const std::string problem_json_items[] = {
    ROBOT_TYPES,
    TASK_TYPES,
    EFFORT_FUNCTION,
    REWARD_FUNCTION,
    ROBOTS,
    TASKS,
    MISSIONS,
    SEARCH_AREA,
    OBSTRUCTED_AREA,
    CONNECTIVITY_FUNCTION
};


static std::string read_file(std::string file_name){
    try{
        std::ifstream file(file_name);
        std::string content;
        while(file.good()){
            std::string line;
            file >> line;
            content.append(line);
        }
        file.close();
        return content;   
    }
    catch(const std::ios_base::failure& e){
        return "";
    }
}

static bool problem_representation_json_is_valid(const JsonObject& json_object){
    return  json_object.isValid() &&
            std::all_of(
                std::begin(problem_json_items), 
                std::end(problem_json_items), 
                [&json_object](const std::string & key){ return json_object.hasItem(key); });
}

#define CREATE_REPRESENTATION(variable_, name_)\
    m_## variable_ = name_## Factory::make(json_object);\
    if(m_## variable_ == nullptr){\
        std::cout << #variable_ " failed\n";\
        m_error_message = #variable_ + std::string{" parsing failed\n"};\
        return false;\
    }

bool ProblemRepresentation::parseJsonRepresentation(JsonObject json_object){
    if(!problem_representation_json_is_valid(json_object)){
        return false;
    }
    CREATE_REPRESENTATION(robot_types, RobotTypes);
    CREATE_REPRESENTATION(task_types, TaskTypes);
    CREATE_REPRESENTATION(effort_function, EffortFunction);
    CREATE_REPRESENTATION(reward_function, RewardFunction);
    CREATE_REPRESENTATION(robots, Robots);
    CREATE_REPRESENTATION(tasks, Tasks);
    CREATE_REPRESENTATION(missions, Missions);
    CREATE_REPRESENTATION(search_area, SearchArea);
    CREATE_REPRESENTATION(obstructed_area, ObstructedArea);
    CREATE_REPRESENTATION(connectivity_function, ConnectivityFunction);
    return true;
} 

static std::tuple<bool, std::string> checkIfAllRobotAndTaskTypesHaveAnEffortFunctionValue(const std::vector<RobotType>& robot_types,const std::vector<TaskType>& task_types, const std::unique_ptr<EffortFunction> & effort_function){
    for(const auto& robot_type: robot_types){
        for(const auto& task_type: task_types){
            if((*effort_function)(task_type, robot_type) == -1){
                return std::make_tuple(false, "Missing value for effort function with entries "+robot_type+" and "+task_type);
            }
        }
    }
    return std::make_tuple(true, "");
}

static std::tuple<bool, std::string> checkIfAllTaskTypesHaveARewardFunctionValue(const std::vector<TaskType>& task_types, const std::unique_ptr<RewardFunction> & reward_function){
    for(const auto& task_type: task_types){
        if((*reward_function)(task_type) == -1){
            return std::make_tuple(false, "Missing value for reward function with entry "+task_type);
        }
    }
    return std::make_tuple(true, "");;
}

static std::tuple<bool, std::string> checkIfRobotsHaveValidTypesAndAreInsideSearchAreaAndOutsideObstacles(const std::unique_ptr<RobotTypes>& robot_types, const std::unique_ptr<Robots> & robots, const std::unique_ptr<SearchArea> & search_area, const std::unique_ptr<ObstructedArea>& obstructed_area){
    for(const auto& robot: *robots){
        if(!robot_types->hasRobotType(robot->getType())){
            return std::make_tuple(false, "Robot "+robot->getName()+" with invalid type "+robot->getType());
        }else if(!search_area->containsPoint(robot->getPosition())){
            return std::make_tuple(false, "Robot "+robot->getName()+" is outside valid search area");
        }else if(obstructed_area->containsPoint(robot->getPosition())){
            return std::make_tuple(false, "Robot "+robot->getName()+" is colliding with an obstacle");
        }
    }
    return std::make_tuple(true, "");
}

static std::tuple<bool, std::string> checkIfTasksHaveValidTypesAndAreInsideSearchAreAndOutsideObstacles(const std::unique_ptr<TaskTypes>& task_types, const std::unique_ptr<Tasks> & tasks, const std::unique_ptr<SearchArea> & search_area, const std::unique_ptr<ObstructedArea>& obstructed_area){
    for(const auto& task: *tasks){
        if(!task_types->hasTaskType(task->getType())){
            return std::make_tuple(false, "Task "+task->getName()+" with invalid type "+task->getType());
        }else if(!search_area->containsPoint(task->getPosition())){
            return std::make_tuple(false, "Task "+task->getName()+" is outside valid search area");
        }else if(obstructed_area->containsPoint(task->getPosition())){
            return std::make_tuple(false, "Task "+task->getName()+" is colliding with an obstacle");
        }
        for(const auto& prerequisite: task->getPrerequisites()){
            if(tasks->getTask(prerequisite) == nullptr){
                return std::make_tuple(false, "Task "+task->getName()+" has a prerequisite that is not a valid task: "+prerequisite);
            }
        }
    }
    return std::make_tuple(true, "");
}

static std::tuple<bool, std::string> checkIfTasksHaveCircularDependence(const std::unique_ptr<Tasks> & tasks){
    for(const auto& task: *tasks){
        std::set<std::string> prerequisite_list;
        bool is_valid;
        tie(prerequisite_list, is_valid) =  Tasks::generatePrerequisiteList(tasks, task, task);
        if(!is_valid){
            return std::make_tuple(false, task->getName()+" has a circular dependence");
        }
    }
    return std::make_tuple(true, "");
}

static std::tuple<bool, std::string> checkIfTaskAreUniqueAndExistInMissions(const std::unique_ptr<Tasks> & tasks, const std::unique_ptr<Missions> & missions){
    std::set<std::string> task_already_looked;
    for(const auto& task: *tasks){
        if(task_already_looked.count(task->getName()) == 0){
            task_already_looked.insert(task->getName());
            std::set<std::string> prerequisite_list;
            auto mission_iterator = std::find_if(missions->begin(), missions->end(), [&task](const auto& mission){return mission->hasTask(task->getName());});
            if(mission_iterator == missions->end()){
                return std::make_tuple(false, "Task "+task->getName()+" is not used in any mission");
            }
            const auto& task_mission = *mission_iterator;
            tie(prerequisite_list, std::ignore) =  Tasks::generatePrerequisiteList(tasks, task, task);
            for(const auto& prerequisite_task_name: prerequisite_list){
                task_already_looked.insert(prerequisite_task_name);
                if(!task_mission->hasTask(prerequisite_task_name)){
                    return std::make_tuple(false, "All Tasks prerequisites must be in the same mission: \""+prerequisite_task_name+"\" prerequisite of \""+task->getName()+"\" is missing from its mission");
                }
            }
        }
    }
    for(const auto& mission: *missions){
        for(const auto& task_name: mission->getTasks()){
            if(tasks->getTask(task_name) == nullptr){
                return std::make_tuple(false, "Mission "+mission->getName()+" has a task that is not a valid task: "+task_name);
            }
            for(const auto& other_mission:* missions){
                if(other_mission != mission && other_mission->hasTask(task_name)){
                    return std::make_tuple(false, "Mission "+mission->getName()+" shares \""+task_name+"\" with mission "+other_mission->getName());
                }
            }
        }
    }
    return std::make_tuple(true, "");
}

template <typename T1, typename T2, typename T3, typename... T>
bool validateProblem(T1& is_valid, T2& error_message, T3 function, const T& ... args){
    tie(is_valid, error_message) = function(args...);
    return is_valid;
} 

void ProblemRepresentation::evaluateProblemRepresentation(){
    validateProblem(m_is_valid, m_error_message, checkIfAllRobotAndTaskTypesHaveAnEffortFunctionValue, m_robot_types->getRobotTypes(), m_task_types->getTaskTypes(), m_effort_function) &&
    validateProblem(m_is_valid, m_error_message, checkIfAllTaskTypesHaveARewardFunctionValue, m_task_types->getTaskTypes(), m_reward_function) &&
    validateProblem(m_is_valid, m_error_message, checkIfRobotsHaveValidTypesAndAreInsideSearchAreaAndOutsideObstacles, m_robot_types, m_robots, m_search_area, m_obstructed_area) &&
    validateProblem(m_is_valid, m_error_message, checkIfTasksHaveValidTypesAndAreInsideSearchAreAndOutsideObstacles, m_task_types, m_tasks, m_search_area, m_obstructed_area) &&
    validateProblem(m_is_valid, m_error_message, checkIfTasksHaveCircularDependence, m_tasks) &&
    validateProblem(m_is_valid, m_error_message, checkIfTaskAreUniqueAndExistInMissions, m_tasks, m_missions);
}


ProblemRepresentation::ProblemRepresentation(std::string file_name):m_error_message{""}{
    auto file_content = read_file(file_name);
    auto json_object = JSON::parseObject(file_content);
    m_is_valid = parseJsonRepresentation(json_object);
    JSON::deleteObject(json_object);
    if(m_is_valid){
        evaluateProblemRepresentation();
    }else{
        m_error_message = "Problem parsing file, file is not a valid JSON!";
    }

}
bool ProblemRepresentation::isValid() const{
    return m_is_valid;
}
const std::unique_ptr<RobotTypes>& ProblemRepresentation::getRobotTypes() const{
    return m_robot_types;
}
const std::unique_ptr<TaskTypes>& ProblemRepresentation::getTaskTypes() const{
    return m_task_types;
}
const std::unique_ptr<EffortFunction>& ProblemRepresentation::getEffortFunction() const{
    return m_effort_function;
}
const std::unique_ptr<RewardFunction>& ProblemRepresentation::getRewardFunction() const{
    return m_reward_function;
}
const std::unique_ptr<Robots>& ProblemRepresentation::getRobots() const{
    return m_robots;
}
const std::unique_ptr<Tasks>& ProblemRepresentation::getTasks() const{
    return m_tasks;
}
const std::unique_ptr<Missions>& ProblemRepresentation::getMissions() const{
    return m_missions;
}
const std::unique_ptr<SearchArea>& ProblemRepresentation::getSearchArea() const{
    return m_search_area;
}    
const std::unique_ptr<ObstructedArea>& ProblemRepresentation::getObstructedArea() const{
    return m_obstructed_area;
}
const std::unique_ptr<ConnectivityFunction>& ProblemRepresentation::getConnectivityFunction() const{
    return m_connectivity_function;
}
const std::string& ProblemRepresentation::getErrorMessage() const{
    return m_error_message;
}
