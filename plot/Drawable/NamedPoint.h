#ifndef NAMEDPOINT_H__
#define NAMEDPOINT_H__

#include <Renderer.h>
#include <Drawable.h>
#include <Color.h>
#include <Point.h>
#include <string>

namespace drawable{
class NamedPoint: public drawable::Drawable{
private:
    drawable::Point m_point;
    std::string m_name;
public:
    NamedPoint(drawable::Point point, std::string name);
    NamedPoint(double x, double y, std::string name, drawable::Color color = drawable::Color::Green): NamedPoint{drawable::Point{x,y,color}, name}{}
    template <typename T>
    NamedPoint(T point, std::string name, drawable::Color color = drawable::Color::Green):NamedPoint(point[0], point[1], name, color){}
    ~NamedPoint(){}
    virtual void draw(const Renderer& renderer) const override;
    double getX() const;
    double getY() const;
    std::string getName() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_point.setColor(color);}
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
    virtual BoundingBox getBoundingBox() const;
};
}

#endif //NAMEDPOINT_H__