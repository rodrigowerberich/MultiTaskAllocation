#ifndef ROBOTSPOSITION_H__
#define ROBOTSPOSITION_H__

#include <TrigDefinitions.h>

class RobotsPosition{
private:
    static Point NullPoint; 
    static Point PlaceHolderPoint; 
    size_t m_num_of_robots;
    Points m_points;
public:
    RobotsPosition(size_t num_of_robots = 0, const Points& points = Points());
    Point& operator[](size_t i);
    const Point& operator[](size_t i) const;
    size_t size() const;
    Points::iterator begin() noexcept;
    Points::const_iterator begin() const noexcept;
    Points::iterator end() noexcept;
    Points::const_iterator end() const noexcept;
    const Points& asPoints() const;
    static double distance(const RobotsPosition& rp1, const RobotsPosition& rp2);
};

std::ostream& operator<<(std::ostream& os, const RobotsPosition& rp);

#endif //ROBOTSPOSITION_H__