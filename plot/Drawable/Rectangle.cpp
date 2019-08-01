#include <Rectangle.h>

drawable::Rectangle::Rectangle(double x, double y, double width, double height, drawable::Color color):m_bottom_left_x{x}, m_bottom_left_y{y}, m_width{width}, m_height{height},m_color{color}{
}
void drawable::Rectangle::draw(const drawable::Renderer& renderer) const{
    renderer.drawRectangle(*this);
}
double drawable::Rectangle::getBottomLeftX() const{
    return m_bottom_left_x;
}
double drawable::Rectangle::getBottomLeftY() const{
    return m_bottom_left_y;
}
double drawable::Rectangle::getWidth() const{
    return m_width;
}
double drawable::Rectangle::getHeight() const{
    return m_height;
}

drawable::Color drawable::Rectangle::getColor() const{
    return m_color;
}
