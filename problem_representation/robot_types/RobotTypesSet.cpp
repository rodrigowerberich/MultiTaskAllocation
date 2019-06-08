#include <RobotTypesSet.h>
#include <algorithm>


std::vector<RobotType> RobotTypesSet::getRobotTypes(){
    std::vector<RobotType> vector_form;
    std::for_each(m_robot_types.begin(), m_robot_types.end(), [&vector_form](const RobotType& type){vector_form.push_back(type);});
    return vector_form;
}
bool RobotTypesSet::hasRobotType(const RobotType& type){
    return std::any_of(m_robot_types.begin(), m_robot_types.end(), [&type](const auto& robot_type){return type==robot_type;});
}
bool RobotTypesSet::addRobotTypes(const RobotType& type){
    m_robot_types.insert(type);
    return true;
}