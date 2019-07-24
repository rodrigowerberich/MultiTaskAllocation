#include <ObstructedAreaCollection.h>
#include <algorithm>

ObstructedAreaCollection::ObstructedAreaCollection(const std::vector<PtrGeometricObject> & obstructed_areas){
    for(auto& obstructed_area:obstructed_areas){
        m_obstructed_areas.push_back(obstructed_area->clone());
    } 
}

std::string ObstructedAreaCollection::toStringRepresentation() const{
    std::string aux = "{\"list:\":[";
    for (const auto & obstructed_area : m_obstructed_areas){
        aux += obstructed_area->toStringRepresentation();
        aux += ",";
    }
    aux.erase(aux.end()-1);
    aux += "]}";
    return aux;
}
bool ObstructedAreaCollection::containsPoint(const Position2d & pos) const{
    return std::any_of(m_obstructed_areas.begin(), m_obstructed_areas.end(), [&pos](auto& obstructed_area){ return obstructed_area->containsPoint(pos); });
}