#ifndef RENDERER_H__
#define RENDERER_H__

namespace renderer{
template <typename DrawableType, typename RendererType>
void draw(const DrawableType& drawable, const RendererType& renderer);
}

namespace drawable{
// Forward declarations    
class Rectangle;
class Circle;
class Line;
class Lines;
class Point;
class Points;
class NamedPoint;

class Renderer{
public:
    virtual void drawRectangle(const drawable::Rectangle& rect) const = 0;
    virtual void drawCircle(const drawable::Circle& circle) const = 0;
    virtual void drawLine(const drawable::Line& rect) const = 0;
    virtual void drawLines(const drawable::Lines& lines) const = 0;
    virtual void drawPoint(const drawable::Point& point) const = 0;
    virtual void drawPoints(const drawable::Points& points) const = 0;
    virtual void drawNamedPoint(const drawable::NamedPoint& named_point) const = 0;
};
}

#endif //RENDERER_H__