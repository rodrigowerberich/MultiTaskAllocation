#include <TaskTypesSet.h>
#include <algorithm>


std::vector<RobotType> TaskTypesSet::getTaskTypes(){
    std::vector<RobotType> vector_form;
    std::for_each(m_Task_types.begin(), m_Task_types.end(), [&vector_form](const RobotType& type){vector_form.push_back(type);});
    return vector_form;
}
bool TaskTypesSet::hasTaskType(const RobotType& type){
    return std::any_of(m_Task_types.begin(), m_Task_types.end(), [&type](const auto& Task_type){return type==Task_type;});
}
bool TaskTypesSet::addTaskTypes(const RobotType& type){
    m_Task_types.insert(type);
    return true;
}