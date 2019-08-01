#include <ObstructedArea.h>
#include <GnuPlotRenderer.h>

ObstructedArea::ObstructedArea(PtrArea && area):m_area{std::move(area)}{
}


const drawable::Drawable* ObstructedArea::getDrawable() const{
    return m_area->getDrawable();
}


namespace renderer{
template <>
void draw(const std::unique_ptr<ObstructedArea>& obstructed_area, const GnuPlotRenderer& renderer){
    auto clone = obstructed_area->getDrawable()->clone();
    clone->setColor(drawable::Color::Red);
    clone->draw(renderer);
}
}