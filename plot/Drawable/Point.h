#ifndef POINT_H__
#define POINT_H__

#include <Drawable.h>
#include <Color.h>

namespace drawable{
class Point: public drawable::Drawable{
private:
    double m_x;
    double m_y;
    drawable::Color m_color;
public:
    Point(double x, double y, drawable::Color color = drawable::Color::Green);
    template <typename T>
    Point(T point, drawable::Color color = drawable::Color::Green):Point(point[0], point[1], color){}
    ~Point(){}
    virtual void draw(const Renderer& renderer) const override;
    double getX() const;
    double getY() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_color = color;};
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
};
}

#endif //POINT_H__