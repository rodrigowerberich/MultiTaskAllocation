#include <Robots.h>

const std::unique_ptr<Robot> & Robots::getRobot(const std::string& robot_name){
    return getValueByName(robot_name);
}