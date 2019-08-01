#ifndef CIRCLE_H__
#define CIRCLE_H__

#include <Drawable.h>
#include <Color.h>

namespace drawable{
class Circle: public drawable::Drawable{
private:
    double m_center_x;
    double m_center_y;
    double m_radius;
    drawable::Color m_color;
public:
    Circle(double center_x, double center_y, double radius, drawable::Color color = drawable::Color::Blue);
    template <typename T>
    Circle(T center, double radius, drawable::Color color = drawable::Color::Blue):Circle(center[0], center[1], radius, color){}
    ~Circle(){}
    virtual void draw(const Renderer& renderer) const override;
    double getCenterX() const;
    double getCenterY() const;
    double getRadius() const;
    drawable::Color getColor() const;
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
};

}


#endif //CIRCLE_H__