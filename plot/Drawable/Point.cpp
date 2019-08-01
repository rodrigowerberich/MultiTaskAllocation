#include <Point.h>

drawable::Point::Point(double x, double y, drawable::Color color):m_x{x}, m_y{y},m_color{color}{
}
void drawable::Point::draw(const drawable::Renderer& renderer) const{
    renderer.drawPoint(*this);
}
double drawable::Point::getX() const{
    return m_x;
}
double drawable::Point::getY() const{
    return m_y;
}
drawable::Color drawable::Point::getColor() const{
    return m_color;
}
std::unique_ptr<drawable::Drawable> drawable::Point::clone() const{
    return std::make_unique<drawable::Point>(m_x, m_y, m_color);
}
