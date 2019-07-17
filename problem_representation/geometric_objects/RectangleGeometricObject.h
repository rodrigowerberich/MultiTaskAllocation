#ifndef RECTANGLEGEOMETRICOBJECT_H__
#define RECTANGLEGEOMETRICOBJECT_H__

#include <GeometricObject.h>
#include <Position.h>

class RectangleGeometricObject: public GeometricObject{
private:
    Position2d m_bottom_left_corner;
    double m_width;
    double m_height;
public:
    RectangleGeometricObject(const Position2d & bottom_left_corner, double width, double height);
    bool containsPoint(const Position2d & position) const override;
    std::string toStringRepresentation() const override;
};

#endif //RECTANGLEGEOMETRICOBJECT_H__