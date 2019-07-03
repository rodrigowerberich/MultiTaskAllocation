#include <ProblemRepresentation.h>
#include <fstream>
#include <iostream>
#include <JSON.h>
#include <RobotTypesFactory.h>
#include <TaskTypesFactory.h>
#include <EffortFunctionFactory.h>
#include <RewardFunctionFactory.h>
#include <algorithm>
#include <ProblemJsonNamesDefinitions.h>

static const std::string problem_json_items[] = {
    ROBOT_TYPES,
    TASK_TYPES,
    EFFORT_FUNCTION,
    REWARD_FUNCTION,
    // "robots",
    // "tasks",
    // "missions",
    // "search_area",
    // "obstructed_area",
    // "connectivity_function"
};


static std::string read_file(std::string file_name){
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
        std::cout << #variable_ "failed\n";\
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
    return true;
}


ProblemRepresentation::ProblemRepresentation(std::string file_name){
    auto file_content = read_file(file_name);
    auto json_object = JSON::parseObject(file_content);
    m_is_valid = parseJsonRepresentation(json_object);
    JSON::deleteObject(json_object);

}
bool ProblemRepresentation::isValid(){
    return m_is_valid;
}
const std::unique_ptr<RobotTypes>& ProblemRepresentation::getRobotTypes(){
    return m_robot_types;
}
const std::unique_ptr<TaskTypes>& ProblemRepresentation::getTaskTypes(){
    return m_task_types;
}
const std::unique_ptr<EffortFunction>& ProblemRepresentation::getEffortFunction(){
    return m_effort_function;
}
const std::unique_ptr<RewardFunction>& ProblemRepresentation::getRewardFunction(){
    return m_reward_function;
}