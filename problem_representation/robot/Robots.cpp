#include <Robots.h>

static std::unique_ptr<Robot> m_null = nullptr;

const std::unique_ptr<Robot> & Robots::getRobot(const std::string& robot_name){
    if(m_name_map.count(robot_name) > 0){
        return m_robots[m_name_map[robot_name]];
    }else{
        return m_null;
    }
    // for (const auto & robot: *this) {
    //     if(robot->getName() == robot_name){
    //         return robot;
    //     }
    // }  
    // return m_null;
}