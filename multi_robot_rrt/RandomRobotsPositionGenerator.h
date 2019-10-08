#ifndef RANDOMROBOTSPOSITIONGENERATOR_H__
#define RANDOMROBOTSPOSITIONGENERATOR_H__

#include <RobotsPosition.h>

class RandomRobotsPositionGenerator{
private:
    size_t m_num_of_robots;
    double m_min_x;
    double m_max_x;
    double m_min_y;
    double m_max_y;
public:
    RandomRobotsPositionGenerator(size_t num_of_robots, double min_x, double max_x, double min_y, double max_y);
    RobotsPosition operator()(size_t) const;
};


#endif //RANDOMROBOTSPOSITIONGENERATOR_H__