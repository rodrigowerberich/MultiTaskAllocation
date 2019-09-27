#ifndef DISTANCEMAP_H__
#define DISTANCEMAP_H__

#include <vector>
#include <tuple>
#include <TrigDefinitions.h>
#include <iterator>
#include <PointMap.h>

class DistanceMap{
private:
    PointMap m_point_map;
    EdgesI m_edges;
    double m_x_min;
    double m_x_max;
    double m_y_min;
    double m_y_max;
    size_t m_N;
    size_t m_M;
    std::vector<std::vector<size_t>> m_edges_mapping;
    size_t calculateIndexX(const Point& p) const;
    size_t calculateIndexY(const Point& p) const;
    size_t calculateIndex(const Point& p) const;
    bool insideMap(const Point& p) const;
    EdgeI nearestEdgeSmall(const Point& p) const;
    EdgeI nearestEdgeBig(const Point& p) const;
public:
    DistanceMap(){}
    DistanceMap(double x_min, double x_max, double y_min, double y_max, size_t N, size_t M);
    ~DistanceMap(){}
    bool insert(const Point& p);
    bool insert(double x, double y);
    bool insert(std::tuple<double, double>);
    template <typename T>
    bool insert(const T& p){
        return insert(p[0], p[1]);
    }
    template <typename Iterator>
    bool insert(const Iterator& begin, const Iterator& end){
        return m_point_map.insert(begin, end);
    }
    bool insertEdge(const EdgeI& edge);
    bool removeEdge(const EdgeI& edge);
    // double distanceToNearestPoint(const Point& p);
    EdgeI nearestEdgeIndex(const Point& p) const;
    PointI nearestPointIndex(const Point& p) const;
    Point nearestPoint(const Point& p) const;
    Points pointsInRange(const Point& p, double radius);
    PointsI pointsInRangeByIndex(const Point& p, double radius) const;
    bool hasPointInRange(const Point& p, double radius);
    std::vector<Point>::iterator begin(){return m_point_map.begin();}
    std::vector<Point>::iterator end(){return m_point_map.end();}
    std::vector<Point>::const_iterator begin() const{return m_point_map.begin();}
    std::vector<Point>::const_iterator end() const{return m_point_map.end();}
    std::vector<Point>::reference back() { return m_point_map.back(); }
    std::vector<Point>::const_reference back() const { return m_point_map.back(); }
    size_t size() const {return m_point_map.size();}
    const Point & operator[](size_t i) const{ return m_point_map[i]; }
    void show();
    double getXMin() const { return m_x_min; }
    double getXMax() const { return m_x_max; }
    double getYMin() const { return m_y_min; }
    double getYMax() const { return m_y_max; }
    size_t getM() const { return m_N; }
    size_t getN() const { return m_M; }
};

#endif //DISTANCEMAP_H__