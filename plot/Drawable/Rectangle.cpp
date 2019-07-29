#include <Rectangle.h>

Drawable::Rectangle::Rectangle(double x, double y, double width, double height, Drawable::Color color):m_bottom_left_x{x}, m_bottom_left_y{y}, m_width{width}, m_height{height},m_color{color}{
}

double Drawable::Rectangle::getBottomLeftX() const{
    return m_bottom_left_x;
}
double Drawable::Rectangle::getBottomLeftY() const{
    return m_bottom_left_y;
}
double Drawable::Rectangle::getWidth() const{
    return m_height;
}
double Drawable::Rectangle::getHeight() const{
    return m_width;
}

Drawable::Color Drawable::Rectangle::getColor() const{
    return m_color;
}
