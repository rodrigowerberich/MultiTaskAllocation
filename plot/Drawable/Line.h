#ifndef LINE_H__
#define LINE_H__

#include <Drawable.h>
#include <Color.h>

namespace drawable{
class Line: public drawable::Drawable{
private:
    double m_p1_x;
    double m_p1_y;
    double m_p2_x;
    double m_p2_y;
    drawable::Color m_color;
public:
    Line(double x1, double y1, double x2, double y2, drawable::Color color = drawable::Color::Black);
    template <typename T>
    Line(T point1, T point2, drawable::Color color = drawable::Color::Black):Line(point1[0], point1[1], point2[0], point2[1], color){}
    ~Line(){}
    virtual void draw(const Renderer& renderer) const override;
    double getX1() const;
    double getY1() const;
    double getX2() const;
    double getY2() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_color = color;};
    virtual bool addDrawable(const Drawable*) {return false;}
    virtual bool removeDrawable(const Drawable*){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
    virtual BoundingBox getBoundingBox() const;
};
}

#endif //LINE_H__