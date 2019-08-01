#include <ObstructedArea.h>
#include <GnuPlotRenderer.h>

ObstructedArea::ObstructedArea(PtrArea && area):m_area{std::move(area)}{
    // area->getDrawable()->setColor(drawable::Color::Red);
}


const drawable::Drawable* ObstructedArea::getDrawable() const{
    return m_area->getDrawable();
}


namespace renderer{
template <>
void draw(const std::unique_ptr<ObstructedArea>& drawable, const GnuPlotRenderer& renderer){
    drawable->getDrawable()->draw(renderer);
}
}