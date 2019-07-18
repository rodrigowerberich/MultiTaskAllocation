#include <SearchArea.h>

SearchArea::SearchArea(std::unique_ptr<GeometricObject> && geometric_representation):
m_geometric_representation(std::move(geometric_representation))
{

}

bool SearchArea::containsPoint(const Position2d & position) const{
    return m_geometric_representation->containsPoint(position);
}
std::string SearchArea::toStringRepresentation() const{
    return m_geometric_representation->toStringRepresentation();
}
