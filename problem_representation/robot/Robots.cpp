#include <Robots.h>

static std::unique_ptr<Robot> m_null = nullptr;

const std::unique_ptr<Robot> & Robots::getRobot(const std::string& robot_name){
    for (const auto & robot: *this) {
        if(robot->getName() == robot_name){
            return robot;
        }
    }  
    return m_null;
}