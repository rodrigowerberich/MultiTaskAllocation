#ifndef ROBOTS_H__
#define ROBOTS_H__

#include <vector>
#include <Robot.h>
#include <memory>
#include <algorithm>

// using Robots = std::vector<std::unique_ptr<Robot>>;

class Robots: public std::vector<std::unique_ptr<Robot>>{
public:
    const std::unique_ptr<Robot> & getRobot(const std::string& robot_name);
};


#endif // ROBOTS_H__