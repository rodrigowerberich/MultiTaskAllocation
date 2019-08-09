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
    PointI nearestPointSmall(const Point& p);
    PointI nearestPointBig(const Point& p);
public:
    PointMap(){}
    PointMap(double x_min, double x_max, double y_min, double y_max, size_t N, size_t M);
    ~PointMap(){}
    bool insert(const Point& p);
    bool insert(double x, double y);
    bool insert(std::tuple<double, double>);
    template <typename T>
    bool insert(const T& p){
        return insert(p[0], p[1]);
    }
    template <typename Iterator>
    bool insert(const Iterator& begin, const Iterator& end){
        Iterator curr = begin;
        while(curr != end){
            insert(*curr);
            ++curr;
        }
        return true;
    }
    // double distanceToNearestPoint(const Point& p);
    PointI nearestPointIndex(const Point& p);
    Point nearestPoint(const Point& p);
    Points pointsInRange(const Point& p, double radius);
    PointsI pointsInRangeByIndex(const Point& p, double radius);
    bool hasPointInRange(const Point& p, double radius);
    std::vector<Point>::iterator begin(){return m_points.begin();}
    std::vector<Point>::iterator end(){return m_points.end();}
    std::vector<Point>::const_iterator begin() const{return m_points.begin();}
    std::vector<Point>::const_iterator end() const{return m_points.end();}
    size_t size(){return m_points.size();}
    const Point & operator[](size_t i) const{ return m_points[i]; }
    void show();
    double getXMin() const { return m_x_min; }
    double getXMax() const { return m_x_max; }
    double getYMin() const { return m_y_min; }
    double getYMax() const { return m_y_max; }
    size_t getM() const { return m_N; }
    size_t getN() const { return m_M; }

};

#endif //POINTMAP_H__