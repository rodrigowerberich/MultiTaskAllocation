#include <CircleGeometricObject.h>
#include <JSON.h>

CircleGeometricObject::CircleGeometricObject(const Position2d & center, double radius):
    m_center{center}
{
    m_radius = (radius > 0.0)?radius:0.0; 
}
bool CircleGeometricObject::containsPoint(const Position2d & position) const{
    return Position2d::euclideanDistance(m_center, position) < m_radius;
}
std::string CircleGeometricObject::toStringRepresentation() const{
    auto json = JSON::createObject();
    json.setString("type","circle");
    auto position_json = JSON::createObject();
    position_json.setString("type", "2d");
    position_json.setDouble("x", m_center.getX());
    position_json.setDouble("y", m_center.getY());
    json.setObject("center", position_json);
    json.setDouble("radius", m_radius);
    auto representation = json.toStringUnformatted();
    JSON::deleteObject(json);
    return representation;
}

std::unique_ptr<GeometricObject> CircleGeometricObject::clone() const {
    return std::make_unique<CircleGeometricObject>(m_center, m_radius);
}
