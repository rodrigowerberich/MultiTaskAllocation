#ifndef RECTANGLEGEOMETRICOBJECT_H__
#define RECTANGLEGEOMETRICOBJECT_H__

#include <GeometricObject.h>
#include <Position.h>
#include <Rectangle.h>

class RectangleGeometricObject: public GeometricObject{
private:
    Position2d m_bottom_left_corner;
    double m_width;
    double m_height;
    drawable::Rectangle m_rect;
public:
    RectangleGeometricObject(const Position2d & bottom_left_corner, double width, double height);
    bool containsPoint(const Position2d & position) const override;
    std::string toStringRepresentation() const override;
    std::unique_ptr<GeometricObject> clone() const override;
    const drawable::Drawable* getDrawable() const override;
};

#endif //RECTANGLEGEOMETRICOBJECT_H__