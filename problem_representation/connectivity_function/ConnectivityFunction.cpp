#include <ConnectivityFunction.h>
#include <GnuPlotRenderer.h>

ConnectivityFunction::ConnectivityFunction(PtrArea && area):m_area{std::move(area)}{

}

const drawable::Drawable* ConnectivityFunction::getDrawable() const{
    return m_area->getDrawable();
}

namespace renderer{
template <>
void draw(const std::unique_ptr<ConnectivityFunction>& connectivity_function, const GnuPlotRenderer& renderer){
    auto clone = connectivity_function->getDrawable()->clone();
    clone->setColor(drawable::Color::Violet);
    clone->draw(renderer);
}
}