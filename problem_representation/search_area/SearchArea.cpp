#include <SearchArea.h>
#include <GnuPlotRenderer.h>

SearchArea::SearchArea(std::unique_ptr<GeometricObject> && geometric_representation):
m_geometric_representation(std::move(geometric_representation))
{
    // m_geometric_representation->getDrawable()->setColor(drawable::Color::Blue);
}

bool SearchArea::containsPoint(const Position2d & position) const{
    return m_geometric_representation->containsPoint(position);
}
std::string SearchArea::toStringRepresentation() const{
    return m_geometric_representation->toStringRepresentation();
}

const drawable::Drawable* SearchArea::getDrawable() const{
    return m_geometric_representation->getDrawable();
}


namespace renderer{
template <>
void draw(const std::unique_ptr<SearchArea>& search_area, const GnuPlotRenderer& renderer){
    auto clone = search_area->getDrawable()->clone();
    clone->setColor(drawable::Color::Blue);
    clone->draw(renderer);
}
}

