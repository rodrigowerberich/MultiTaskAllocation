#ifndef ROBOTS_H__
#define ROBOTS_H__

#include <vector>
#include <Robot.h>
#include <memory>
#include <algorithm>
#include <map>
#include <MappedVectorOfPointers.h>

class Robots: public MappedVectorOfPointers<Robot>{
public:
    const std::unique_ptr<Robot> & getRobot(const std::string& robot_name);
};


#endif // ROBOTS_H__