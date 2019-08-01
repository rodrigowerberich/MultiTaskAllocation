#include <ObstructedArea.h>
#include <GnuPlotRenderer.h>

const drawable::Drawable* ObstructedArea::getDrawable() const{
    return m_area->getDrawable();
}


namespace renderer{
template <>
void draw(const std::unique_ptr<ObstructedArea>& drawable, const GnuPlotRenderer& renderer){
    drawable->getDrawable()->draw(renderer);
}
}