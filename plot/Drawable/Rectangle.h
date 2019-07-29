#ifndef RECTANGLE_H__
#define RECTANGLE_H__

#include <Color.h>

namespace Drawable{
class Rectangle{
private:
    double m_bottom_left_x;
    double m_bottom_left_y;
    double m_width;
    double m_height;
    Drawable::Color m_color;
public:
    Rectangle(double x, double y, double width, double height, Drawable::Color color = Drawable::Color::Red);
    template <typename T>
    Rectangle(T pos, double width, double height, Drawable::Color color = Drawable::Color::Red):Rectangle(pos[0], pos[1], width, height, color){}
    ~Rectangle(){}
    double getBottomLeftX() const;
    double getBottomLeftY() const;
    double getWidth() const;
    double getHeight() const;
    Drawable::Color getColor() const;
};

}

#endif //RECTANGLE_H__