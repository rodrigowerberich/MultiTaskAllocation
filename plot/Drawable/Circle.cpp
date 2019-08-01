#include <Circle.h>

drawable::Circle::Circle(double center_x, double center_y, double radius, drawable::Color color):m_center_x{center_x}, m_center_y{center_y}, m_radius{radius},m_color{color}{
}
void drawable::Circle::draw(const drawable::Renderer& renderer) const{
    renderer.drawCircle(*this);
}
double drawable::Circle::getCenterX() const{
    return m_center_x;
}
double drawable::Circle::getCenterY() const{
    return m_center_y;
}
double drawable::Circle::getRadius() const{
    return m_radius;
}
drawable::Color drawable::Circle::getColor() const{
    return m_color;
}
std::unique_ptr<drawable::Drawable> drawable::Circle::clone()  const{
    return std::make_unique<drawable::Circle>(m_center_x, m_center_y, m_radius, m_color);
}
