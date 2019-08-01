#include <RectangleGeometricObject.h>
#include <JSON.h>

RectangleGeometricObject::RectangleGeometricObject(const Position2d & bottom_left_corner, double width, double height):
    m_bottom_left_corner{bottom_left_corner},
    m_rect{m_bottom_left_corner, m_width, m_height}
{
    m_width = (width > 0.0)?width:0.0; 
    m_height = (height > 0.0)?height:0.0; 
    m_rect = {m_bottom_left_corner, m_width, m_height};
}
bool RectangleGeometricObject::containsPoint(const Position2d & position) const{
    bool isInX = m_bottom_left_corner.getX() < position.getX() && position.getX() < m_bottom_left_corner.getX()+m_width;
    bool isInY = m_bottom_left_corner.getY() < position.getY() && position.getY() < m_bottom_left_corner.getY()+m_height;
    return isInX && isInY;
}
std::string RectangleGeometricObject::toStringRepresentation() const{
    auto json = JSON::createObject();
    json.setString("type","rectangle");
    auto position_json = JSON::createObject();
    position_json.setString("type", "2d");
    position_json.setDouble("x", m_bottom_left_corner.getX());
    position_json.setDouble("y", m_bottom_left_corner.getY());
    json.setObject("bottom_left", position_json);
    json.setDouble("width", m_width);
    json.setDouble("height", m_height);
    auto representation = json.toStringUnformatted();
    JSON::deleteObject(json);
    return representation;
}

std::unique_ptr<GeometricObject> RectangleGeometricObject::clone() const{
    return std::make_unique<RectangleGeometricObject>(m_bottom_left_corner, m_width, m_height);
}

const drawable::Drawable* RectangleGeometricObject::getDrawable() const{
    return &m_rect;
}
