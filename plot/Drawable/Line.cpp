#include <Line.h>

drawable::Line::Line(double x1, double y1, double x2, double y2, drawable::Color color):m_p1_x{x1}, m_p1_y{y1}, m_p2_x{x2}, m_p2_y{y2},m_color{color}{
}
void drawable::Line::draw(const drawable::Renderer& renderer) const{
    renderer.drawLine(*this);
}
double drawable::Line::getX1() const{
    return m_p1_x;
}
double drawable::Line::getY1() const{
    return m_p1_y;
}
double drawable::Line::getX2() const{
    return m_p2_x;
}
double drawable::Line::getY2() const{
    return m_p2_y;
}
drawable::Color drawable::Line::getColor() const{
    return m_color;
}
std::unique_ptr<drawable::Drawable> drawable::Line::clone() const{
    return std::make_unique<drawable::Line>(m_p1_x, m_p1_y, m_p2_x, m_p2_y, m_color);
}
drawable::BoundingBox drawable::Line::getBoundingBox() const{
    return {std::min(m_p1_x,m_p2_x), std::min(m_p1_y,m_p2_y), std::max(m_p1_x,m_p2_x), std::max(m_p1_y,m_p2_y)};
}
