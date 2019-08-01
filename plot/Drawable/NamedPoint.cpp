#include <NamedPoint.h>

drawable::NamedPoint::NamedPoint(drawable::Point point, std::string name):m_point{point}, m_name{name}{}
double drawable::NamedPoint::getX() const{
    return m_point.getX();
}
void drawable::NamedPoint::draw(const drawable::Renderer& renderer) const{
    renderer.drawNamedPoint(*this);
}
double drawable::NamedPoint::getY() const{
    return m_point.getY();
}
std::string drawable::NamedPoint::getName() const{
    return m_name;
}
drawable::Color drawable::NamedPoint::getColor() const{
    return m_point.getColor();
}
std::unique_ptr<drawable::Drawable> drawable::NamedPoint::clone()  const{
    return std::make_unique<drawable::NamedPoint>(m_point, m_name);
}
