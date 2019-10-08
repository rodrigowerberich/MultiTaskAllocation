#include <RandomRobotsPositionGenerator.h>
#include <random>

RandomRobotsPositionGenerator::RandomRobotsPositionGenerator(size_t num_of_robots, double min_x, double max_x, double min_y, double max_y):
    m_num_of_robots{num_of_robots},
    m_min_x{min_x},
    m_max_x{max_x},
    m_min_y{min_y},
    m_max_y{max_y}{}

RobotsPosition RandomRobotsPositionGenerator::operator()(size_t) const{
    static std::random_device rd;
    static std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist_x(m_min_x, m_max_x);
    std::uniform_real_distribution<> dist_y(m_min_y, m_max_y);
    RobotsPosition p(m_num_of_robots);
    for(int robot_i = 0; robot_i < m_num_of_robots; robot_i++){
        p[robot_i] = {dist_x(e2), dist_y(e2)};
    }
    return p;
}
