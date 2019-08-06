#ifndef POINTMAP_H__
#define POINTMAP_H__

#include <vector>
#include <tuple>
#include <TrigDefinitions.h>
#include <iterator>

class PointMap{
private:
    std::vector<Point> m_points;
    std::vector<std::vector<size_t>> m_points_mapping;
    std::vector<size_t> m_points_ordering;
    double m_x_min;
    double m_x_max;
    double m_y_min;
    double m_y_max;
    size_t m_N;
    size_t m_M;
    size_t calculateIndexX(const Point& p);
    size_t calculateIndexY(const Point& p);
    size_t calculateIndex(const Point& p);
    bool insideMap(const Point& p);

public:
    PointMap(double x_min, double x_max, double y_min, double y_max, size_t N, size_t M);
    ~PointMap(){}
    bool insert(const Point& p);
    bool insert(double x, double y);
    bool insert(std::tuple<double, double>);
    template <typename T>
    bool insert(const T& p){
        return insert(p[0], p[1]);
    }
    // double distanceToNearestPoint(const Point& p);
    Points pointsInRange(const Point& p, double radius);
    bool hasPointInRange(const Point& p, double radius);
    std::vector<Point>::iterator begin(){return m_points.begin();}
    std::vector<Point>::iterator end(){return m_points.end();}
    size_t size(){return m_points.size();}
    const Point & operator[](size_t i){ return m_points[i]; }
    void show();
};

#endif //POINTMAP_H__