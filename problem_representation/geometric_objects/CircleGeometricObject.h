#ifndef CIRCLEGEOMETRICAREA_H__
#define CIRCLEGEOMETRICAREA_H__

#include <GeometricObject.h>
#include <Position.h>

class CircleGeometricObject: public GeometricObject{
private:
    Position2d m_center;
    double m_radius;
public:
    CircleGeometricObject(const Position2d & center, double radius);
    bool containsPoint(const Position2d & position) const;
    const std::string & toStringRepresentation() const;
};

#endif //CIRCLEGEOMETRICAREA_H__