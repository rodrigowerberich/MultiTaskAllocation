#include <TaskTypesSet.h>
#include <algorithm>


std::vector<TaskType> TaskTypesSet::getTaskTypes(){
    std::vector<TaskType> vector_form;
    std::for_each(m_Task_types.begin(), m_Task_types.end(), [&vector_form](const TaskType& type){vector_form.push_back(type);});
    return vector_form;
}
bool TaskTypesSet::hasTaskType(const TaskType& type){
    return std::any_of(m_Task_types.begin(), m_Task_types.end(), [&type](const auto& Task_type){return type==Task_type;});
}
bool TaskTypesSet::addTaskTypes(const TaskType& type){
    m_Task_types.insert(type);
    return true;
}