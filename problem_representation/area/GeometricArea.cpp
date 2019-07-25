#include <GeometricArea.h>
#include <algorithm>

GeometricArea::GeometricArea(const std::vector<PtrGeometricObject> & areas){
    for(auto& area:areas){
        m_areas.push_back(area->clone());
    } 
}

std::string GeometricArea::toStringRepresentation() const{
    std::string aux = "{\"list:\":[";
    for (const auto & area : m_areas){
        aux += area->toStringRepresentation();
        aux += ",";
    }
    aux.erase(aux.end()-1);
    aux += "]}";
    return aux;
}
bool GeometricArea::containsPoint(const Position2d & pos) const{
    return std::any_of(m_areas.begin(), m_areas.end(), [&pos](auto& area){ return area->containsPoint(pos); });
}