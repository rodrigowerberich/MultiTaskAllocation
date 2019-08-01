#ifndef RECTANGLE_H__
#define RECTANGLE_H__

#include <Drawable.h>
#include <Color.h>

namespace drawable{
class Rectangle: public drawable::Drawable{
private:
    double m_bottom_left_x;
    double m_bottom_left_y;
    double m_width;
    double m_height;
    drawable::Color m_color;
public:
    Rectangle(double x, double y, double width, double height, drawable::Color color = drawable::Color::Red);
    template <typename T>
    Rectangle(T pos, double width, double height, drawable::Color color = drawable::Color::Red):Rectangle(pos[0], pos[1], width, height, color){}
    ~Rectangle(){}
    virtual void draw(const Renderer& renderer) const override;
    double getBottomLeftX() const;
    double getBottomLeftY() const;
    double getWidth() const;
    double getHeight() const;
    drawable::Color getColor() const;
    virtual void setColor(drawable::Color color){m_color = color;};
    virtual bool addDrawable(const Drawable* drawable) {return false;}
    virtual bool removeDrawable(const Drawable* drawable){return false;}
    virtual std::unique_ptr<Drawable> clone() const;
};

}

#endif //RECTANGLE_H__