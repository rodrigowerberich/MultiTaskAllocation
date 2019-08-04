#ifndef TrigDefinitions_h_
#define TrigDefinitions_h_

#include <Vector2d.h>
#include <array>
#include <vector>
#include <algorithm>

using Point = Vector2d<double>;
using Points = std::vector<Point>;
using Triangle = std::array<Point,3>;
using Triangles = std::vector<Triangle>;
using Edge = std::array<Point,2>;
using Edges = std::vector<Edge>;
typedef struct Circle{
    Point center;
    double radius;
}Circle;

using PointI = int;
using PointsI = std::vector<PointI>;
using TriangleI = std::array<PointI,3>;
using TrianglesI = std::vector<TriangleI>;
using EdgeI = std::array<PointI,2>;
using EdgesI = std::vector<EdgeI>;

TriangleI PI2TI(const PointsI& point);

template<class C, class T>
bool inside(const C& container,const T& val){
    return std::find(std::begin(container), std::end(container), val) != std::end(container);
}

std::ostream& operator<<(std::ostream& os, const Edge& e);
std::ostream& operator<<(std::ostream& os, const Edges& es);
std::ostream& operator<<(std::ostream& os, const Points& ps);
std::ostream& operator<<(std::ostream& os, const Triangle& t);
std::ostream& operator<<(std::ostream& os, const Triangles& ts);
std::ostream& operator<<(std::ostream& os, const EdgeI& e);
std::ostream& operator<<(std::ostream& os, const EdgesI& es);
std::ostream& operator<<(std::ostream& os, const PointsI& ps);
std::ostream& operator<<(std::ostream& os, const TriangleI& t);
std::ostream& operator<<(std::ostream& os, const TrianglesI& ts);

#endif