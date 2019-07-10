#ifndef RECTANGLEGEOMETRICAREA_H__
#define RECTANGLEGEOMETRICAREA_H__

#include <GeometricObject.h>
#include <Position.h>

class RectangleGeometricObject: public GeometricObject{
private:
    Position2d m_bottom_left_corner;
    double m_width;
    double m_height;
public:
    RectangleGeometricObject(const Position2d & bottom_left_corner, double width, double height);
    bool containsPoint(const Position2d & position) const;
    const std::string & toStringRepresentation() const;
};

#endif //RECTANGLEGEOMETRICAREA_H__