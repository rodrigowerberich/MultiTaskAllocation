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
    bool containsPoint(const Position2d & position) const override;
    std::string toStringRepresentation() const override;
    std::unique_ptr<GeometricObject> clone() const override;
};

#endif //CIRCLEGEOMETRICAREA_H__